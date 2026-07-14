"""plugin_registry.py — discover, register and manage plugin instances.

Supports TWO instantiation modes per plugin class (controlled by `multiple`):
  - single-instance (services/tools, multiple=False): instance_id == name.
    Enabling twice is idempotent. registry.get("ICPRegistration") returns it.
  - multi-instance (sources/displays, multiple=True): each enable() mints a
    fresh instance_id ("Name#2", "Name#3", ...). One per robot, like RViz2.

The catalog (name -> class) is always single-entry per plugin type. The live
_instances dict is keyed by instance_id. get() is backwards-compatible: it
first tries instance_id, then falls back to name (returning the one live
instance of that type if exactly one exists) so singleton lookups like
registry.get("ICPRegistration") keep working unchanged.
"""
from __future__ import annotations
import importlib
import pkgutil
import logging
from typing import Optional

from core.plugin_base import (
    PluginBase, PluginContext, SceneUpdate,
)

log = logging.getLogger("multi3dviz.registry")


class PluginRegistry:
    def __init__(self, ctx: PluginContext):
        self.ctx = ctx
        # name -> plugin class (the catalog: one entry per plugin type)
        self._catalog: dict[str, type[PluginBase]] = {}
        # instance_id -> live enabled instance
        self._instances: dict[str, PluginBase] = {}
        self._discover()

    # --- discovery ---
    def _discover(self) -> None:
        """Import every submodule of backend.plugins.<category> and harvest
        PluginBase subclasses."""
        import plugins  # backend/plugins package
        for category in ("source", "display", "tool", "service"):
            pkg_name = f"plugins.{category}"
            try:
                pkg = importlib.import_module(pkg_name)
            except ModuleNotFoundError:
                continue
            for mod_info in pkgutil.iter_modules(pkg.__path__):
                mod_name = f"{pkg_name}.{mod_info.name}"
                try:
                    mod = importlib.import_module(mod_name)
                except Exception as e:
                    log.warning("plugin module %s failed to import: %s", mod_name, e)
                    continue
                for attr in dir(mod):
                    obj = getattr(mod, attr)
                    if (isinstance(obj, type)
                            and issubclass(obj, PluginBase)
                            and obj not in (PluginBase,)
                            and getattr(obj, "category", None) == category
                            and obj.__module__ == mod.__name__):
                        if obj.name in self._catalog:
                            log.warning("duplicate plugin name %r (from %s) — skipping",
                                        obj.name, mod_name)
                            continue
                        self._catalog[obj.name] = obj
                        log.info("discovered %s plugin: %s — %s",
                                 category, obj.name, obj.description)

    # --- catalog (for the frontend list) ---
    def catalog(self) -> list[dict]:
        """Static description of every available plugin TYPE, for the frontend.
        Includes missing_deps so the UI can render an 'Install' button for
        plugins whose optional heavy deps (torch) aren't in the slim build."""
        from core import optional_deps
        out = []
        for name, cls in sorted(self._catalog.items()):
            missing = optional_deps.get_missing(name)
            out.append({
                "name": name,
                "category": cls.category,
                "description": cls.description,
                "default_enabled": cls.default_enabled,
                "multiple": cls.multiple,
                "properties": cls.properties,
                "missing_deps": missing,
            })
        return out

    # --- instance-id minting ---
    def _next_instance_id(self, name: str) -> str:
        """Mint a fresh instance_id for a multi-instance plugin: Name#N where
        N is the smallest unused integer >= 1."""
        n = 1
        while f"{name}#{n}" in self._instances:
            n += 1
        return f"{name}#{n}"

    def enable_defaults(self) -> None:
        """Create the default instance set for every plugin (first-run path).
        - multi-instance with default_instances: one instance per entry, each
          with its property overrides applied.
        - any plugin with default_enabled=True and no default_instances: one
          bare instance.
        Called by the backend when there's no saved config to restore from.
        """
        for name, cls in self._catalog.items():
            if cls.default_instances:
                for overrides in cls.default_instances:
                    iid = self.enable(name)
                    inst = self._instances.get(iid) if iid else None
                    if inst is not None:
                        for k, v in overrides.items():
                            inst.set_property(k, v)
            elif cls.default_enabled:
                self.enable(name)

    def _instances_of_type(self, name: str) -> list[str]:
        """All live instance_ids of a given plugin type (name)."""
        return [iid for iid, inst in self._instances.items()
                if inst.name == name]

    # --- enable/disable lifecycle ---
    def is_enabled(self, key: str) -> bool:
        """True if `key` is a live instance_id OR (for singletons) the type
        name with one live instance."""
        if key in self._instances:
            return True
        return bool(self._instances_of_type(key))

    def enable(self, name: str, instance_id: str = None) -> Optional[str]:
        """Instantiate and activate a plugin. Returns the instance_id on
        success, None on failure.

        - single-instance (multiple=False): instance_id defaults to `name`;
          if already enabled, returns the existing id (idempotent).
        - multi-instance (multiple=True): a new instance is created each call;
          instance_id auto-mints as Name#N unless an explicit one is given
          (used by config restore).
        """
        cls = self._catalog.get(name)
        if cls is None:
            log.warning("enable: unknown plugin %r", name)
            return None
        if cls.multiple:
            iid = instance_id or self._next_instance_id(name)
            if iid in self._instances:
                return iid  # restore path: already exists
        else:
            iid = instance_id or name
            if iid in self._instances:
                return iid  # idempotent for singletons
        try:
            inst = cls(self.ctx)
            # Assign the instance_id AFTER construction so subclasses don't
            # each have to thread it through their __init__/super().__init__.
            # The base __init__ already defaulted it to self.name.
            inst.instance_id = iid
            inst.on_enable()
        except Exception as e:
            log.exception("enable %r failed: %s", iid, e)
            return None
        self._instances[iid] = inst
        log.info("enabled plugin %s", iid)
        return iid

    def disable(self, instance_id: str) -> bool:
        inst = self._instances.pop(instance_id, None)
        if inst is None:
            return False
        try:
            inst.on_disable()
        except Exception as e:
            log.exception("disable %r failed: %s", instance_id, e)
        # Clear saved config for this instance so a removed instance doesn't
        # resurrect on restart.
        store = getattr(self.ctx, "config_store", None)
        if store is not None:
            store.clear_plugin(instance_id)
        log.info("disabled plugin %s", instance_id)
        return True

    def get(self, key: str) -> Optional[PluginBase]:
        """Fetch a live instance. Backwards-compatible:
          1. exact instance_id match
          2. if key is a type name with exactly ONE live instance, return it
             (so registry.get("ICPRegistration") works for singletons)
        Returns None if not found, or if a type name has 0 or 2+ instances."""
        if key in self._instances:
            return self._instances[key]
        ids = self._instances_of_type(key)
        if len(ids) == 1:
            return self._instances[ids[0]]
        return None

    def set_property(self, instance_id: str, key: str, value) -> bool:
        inst = self._instances.get(instance_id)
        if inst is None:
            return False
        return inst.set_property(key, value)

    def property_state(self, instance_id: str) -> Optional[dict]:
        inst = self._instances.get(instance_id)
        return inst.property_state() if inst else None

    # --- tick: advance all live Display/Service plugins ---
    def tick(self, dt: float) -> list[SceneUpdate]:
        updates = []
        for iid, inst in list(self._instances.items()):
            if inst.category == "tool":
                continue
            try:
                upd = inst.update(dt)
            except Exception as e:
                log.exception("plugin %s update() failed: %s", iid, e)
                continue
            if upd is not None:
                updates.append(upd)
        return updates

    def enabled_list(self) -> list[dict]:
        """Live instances with their current property values + identity."""
        return [
            {"name": inst.name, "instance_id": iid,
             "category": inst.category,
             "properties": inst.property_state()}
            for iid, inst in self._instances.items()
        ]
