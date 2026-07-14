"""config_store.py — persistent JSON configuration for plugins + the fleet.

Stores user-tunable state so it survives restarts:
    {
      "plugins": { "<PluginName>": { "<key>": <value>, ... }, ... },
      "robots":   [ {robot_id, host, user, password, ...}, ... ],
      "app":      { "<key>": <value>, ... }
    }

Location: a multi3dviz_config.json next to the backend entry (dev) or in
%APPDATA%/Multi3DViz (packed). For now, dev path is fine — the packed path
resolution can be added when packaging.

Thread-safety: a single lock guards the file. Saves are debounced in-process
(flush every SAVE_DEBOUNCE_S) so a slider drag doesn't hammer disk.
"""
from __future__ import annotations
import os
import json
import time
import threading
import logging

log = logging.getLogger("multi3dviz.config")

SAVE_DEBOUNCE_S = 1.0   # flush at most once per second


class ConfigStore:
    def __init__(self, path: str):
        self.path = path
        self._lock = threading.Lock()
        self._dirty = False
        self._last_save = 0.0
        self._data = {"plugins": {}, "robots": [], "app": {}}
        self._load()

    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                self._data = json.load(f)
            # Ensure the expected top-level keys exist.
            self._data.setdefault("plugins", {})
            self._data.setdefault("robots", [])
            self._data.setdefault("app", {})
            log.info("config loaded from %s", self.path)
        except FileNotFoundError:
            log.info("no config file at %s — starting fresh", self.path)
        except (json.JSONDecodeError, OSError) as e:
            log.warning("config load failed (%s) — starting fresh", e)

    # --- plugin instance properties (keyed by instance_id) ---
    def get_plugin_props(self, instance_id: str) -> dict:
        """Return saved property values for a plugin instance, or {} if none."""
        with self._lock:
            return dict(self._data["plugins"].get(instance_id, {}))

    def set_plugin_prop(self, instance_id: str, key: str, value):
        with self._lock:
            self._data["plugins"].setdefault(instance_id, {})[key] = value
            self._dirty = True

    def clear_plugin(self, instance_id: str):
        """Drop a plugin instance's saved state (e.g. when it's removed)."""
        with self._lock:
            self._data["plugins"].pop(instance_id, None)
            self._dirty = True

    # --- robot fleet ---
    def get_robots(self) -> list[dict]:
        with self._lock:
            return list(self._data.get("robots", []))

    def set_robots(self, robots: list[dict]):
        with self._lock:
            self._data["robots"] = robots
            self._dirty = True

    # --- app-level ---
    def get_app(self, key: str, default=None):
        with self._lock:
            return self._data.get("app", {}).get(key, default)

    def set_app(self, key: str, value):
        with self._lock:
            self._data.setdefault("app", {})[key] = value
            self._dirty = True

    # --- persistence ---
    def maybe_save(self) -> bool:
        """Flush to disk if dirty AND the debounce window has elapsed.
        Returns True if a save happened. Called periodically by the backend."""
        with self._lock:
            if not self._dirty:
                return False
            if time.monotonic() - self._last_save < SAVE_DEBOUNCE_S:
                return False
            data_snapshot = json.dumps(self._data, indent=2, default=_json_default)
            self._dirty = False
            self._last_save = time.monotonic()
        # Write outside the lock so concurrent reads aren't blocked on disk IO.
        try:
            tmp = self.path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write(data_snapshot)
            os.replace(tmp, self.path)  # atomic on Windows >= py3.3
            return True
        except OSError as e:
            log.warning("config save failed: %s", e)
            return False

    def force_save(self):
        """Save immediately (e.g. on app exit), ignoring the debounce."""
        with self._lock:
            self._last_save = 0.0
            self._dirty = True
        self.maybe_save()


def _json_default(o):
    """Serialize numpy scalars/arrays that JSON can't handle natively."""
    import numpy as np
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"not JSON serializable: {type(o)}")
