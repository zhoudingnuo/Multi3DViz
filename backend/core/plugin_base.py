"""plugin_base.py — Base classes for the four plugin categories.

Plugin model (aligned with RViz2 Display architecture):
    DataSource  — produces normalized sensor frames (point clouds, odometry)
    Display     — consumes data, produces declarative SceneUpdates (3D/2D)
    Tool        — interactive (click to set goal, measure, etc.)
    Service     — background computation/control (ICP, explorer, SSH launcher)

Key contract: plugins produce DECLARATIVE SceneUpdates describing what to
add/update/remove in the Three.js scene. They never touch frontend code —
the frontend renders SceneUpdates uniformly. This is RViz's "Display produces
data, framework renders" model.

All plugins declare a `properties` dict describing user-tunable knobs; the
frontend auto-generates an property panel from it. Property schema:
    {'type': 'float'|'int'|'select'|'bool'|'string'|'robot_ref'|'path',
     'default': ..., 'min'?, 'max'?, 'step'?, 'options'? (for select),
     'label'? (display name), 'group'? (panel section)}
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Optional


class PluginBase:
    """Common base for all plugin categories.

    Subclasses set the class attributes `name`, `category`, `description`,
    `properties`. The registry instantiates plugins on enable and destroys
    them on disable."""
    name: str = "base"
    category: str = "base"            # 'source' | 'display' | 'tool' | 'service'
    description: str = ""
    # When False this plugin is available but not auto-enabled at startup.
    default_enabled: bool = False
    # Can the user enable MULTIPLE instances of this plugin? Sources/displays
    # set True (one per robot); services/tools stay False (globally unique).
    multiple: bool = False
    # For multi-instance plugins: a list of property-override dicts, one per
    # default instance to create at first run. E.g. a dual-robot source sets
    # default_instances = [{'robot':'unitree','robot_id':'robot_a'},
    #                      {'robot':'agibot','robot_id':'robot_b'}].
    # When empty + default_enabled=True, exactly one default instance is made.
    default_instances: list = []
    # Human-tunable knobs; frontend builds a panel from this.
    properties: dict = {}

    def __init__(self, ctx: "PluginContext", instance_id: str = None):
        self.ctx = ctx
        # instance_id identifies THIS instance (vs `name` = the plugin type).
        # For singletons it equals `name`; for multi-instance it's `name#N`.
        # Used as the key in the registry, config store, and WS messages.
        self.instance_id = instance_id or self.name
        self._prop_values = {}
        # Initialize property values from schema defaults.
        for key, schema in self.properties.items():
            self._prop_values[key] = schema.get("default")
        # Override with persisted values (user's last session) if a config
        # store is wired. Unknown keys are ignored so a schema change across
        # versions doesn't resurrect stale properties. Keyed by instance_id
        # so two instances of the same plugin keep separate saved state.
        store = getattr(ctx, "config_store", None)
        if store is not None:
            saved = store.get_plugin_props(self.instance_id)
            for key, val in saved.items():
                if key in self.properties:
                    self._prop_values[key] = val

    # --- property access ---
    def get(self, key: str, default: Any = None) -> Any:
        return self._prop_values.get(key, default)

    def set_property(self, key: str, value: Any) -> bool:
        """Set a property. Returns True if accepted. Subclasses override to
        react to changes (e.g. re-derive colors). Persists to the config store
        so the value survives restarts."""
        if key not in self.properties:
            return False
        self._prop_values[key] = value
        self.on_property_change(key, value)
        store = getattr(self.ctx, "config_store", None)
        if store is not None:
            store.set_plugin_prop(self.instance_id, key, value)
        return True

    def on_property_change(self, key: str, value: Any) -> None:
        """Override hook — called after a property changes."""
        pass

    def property_state(self) -> dict:
        """Current property values, for the frontend panel."""
        return dict(self._prop_values)

    # --- lifecycle ---
    def on_enable(self) -> None:
        """Called once when the user enables this plugin. Override to allocate
        resources (open files, connect SSH, subscribe to a source)."""
        pass

    def on_disable(self) -> None:
        """Called when the user disables this plugin. Override to release
        resources. Must be idempotent."""
        pass


class DataSourcePlugin(PluginBase):
    """Produces normalized sensor frames. Consumed by Display plugins.

    A frame is a dict shaped like:
        {'points': (N,3) float32, 'colors'?: (N,3) float32,
         'odom'?: {x,y,z,qx,qy,qz,qw}, 'robot_id': str, 'frame_idx': int}
    DataSources push frames into the PluginContext's data bus; Displays pull
    the latest frame for their configured source robot."""
    category = "source"


class DisplayPlugin(PluginBase):
    """Consumes data and produces SceneUpdates for the frontend to render.

    Override `update(dt)` to return a SceneUpdate (or None when nothing
    changed). Read sensor data from ctx.data via the configured source."""
    category = "display"

    def update(self, dt: float) -> Optional["SceneUpdate"]:
        """Called each tick. Return a SceneUpdate describing scene changes,
        or None if nothing to render this frame."""
        return None


class ToolPlugin(PluginBase):
    """Interactive: reacts to frontend input events (3D clicks, 2D clicks).

    Override on_scene_click / on_grid_click. The frontend forwards user input
    events with world coordinates; the tool decides what to do (e.g. issue a
    navigation goal via SSH)."""
    category = "tool"

    def on_scene_click(self, world_xyz, robot_id=None) -> Optional["SceneUpdate"]:
        return None

    def on_grid_click(self, world_xy, robot_id=None) -> Optional["SceneUpdate"]:
        return None


class ServicePlugin(PluginBase):
    """Background computation/control. Runs autonomously once enabled; not
    tied to the render loop. Override update(dt) for periodic work."""
    category = "service"

    def update(self, dt: float) -> Optional["SceneUpdate"]:
        return None


# ---------------------------------------------------------------------------
# SceneUpdate — declarative description of frontend scene changes
# ---------------------------------------------------------------------------
@dataclass
class SceneObject:
    """One renderable object in the frontend scene, keyed by `id`.
    The frontend owns the Three.js object with this id; add/update/remove
    operate on it. `kind` tells the frontend which Three.js class to use."""
    id: str                      # stable id, e.g. "robot_a_cloud"
    kind: str                    # 'points' | 'mesh' | 'box' | 'line' | 'label' | 'grid2d'
    # Payload — interpretation depends on kind. For 'points':
    #   {'positions': (N,3)float32, 'colors'?: (N,3)float32, 'point_size'?: float}
    # For 'box':  {'size': [x,y,z], 'color': [r,g,b], 'pose': 4x4 list}
    # For 'mesh': {'positions': (M,3), 'indices': (K,3)int, 'colors'?: (M,3)}
    # For 'line': {'positions': (L,3), 'color': [r,g,b], 'width'?: float}
    # For 'label': {'text': str, 'position': [x,y,z], 'color'?: [r,g,b]}
    payload: dict = field(default_factory=dict)
    # Optional frame metadata for grouping in the frontend (robot_id etc.)
    meta: dict = field(default_factory=dict)


@dataclass
class SceneUpdate:
    """A batch of scene changes a Display/Service wants applied. The
    SceneBridge serializes this into a WS message (binary frames for big
    arrays like point positions)."""
    add: list = field(default_factory=list)       # list[SceneObject]
    update: list = field(default_factory=list)    # list[SceneObject]
    remove: list = field(default_factory=list)    # list[str] object ids

    @staticmethod
    def empty():
        return SceneUpdate()


# ---------------------------------------------------------------------------
# PluginContext — what plugins can reach at runtime
# ---------------------------------------------------------------------------
class DataBus:
    """Latest sensor frames keyed by robot_id. DataSources write, Displays
    read. Lightweight — just a dict of {'robot_id': latest_frame}."""

    def __init__(self):
        self._frames: dict[str, dict] = {}

    def publish(self, robot_id: str, frame: dict) -> None:
        self._frames[robot_id] = frame

    def latest(self, robot_id: str) -> Optional[dict]:
        return self._frames.get(robot_id)

    def robots(self) -> list[str]:
        return list(self._frames.keys())


class PluginContext:
    """Passed to every plugin at construction. Plugins use it to read data,
    publish results, and access shared services (robot manager, config)."""

    def __init__(self):
        self.data = DataBus()
        self.robots = None          # RobotManager — set by backend/main.py
        self.config = {}            # app-level config (paths, perf mode)
        self.config_store = None    # ConfigStore — set by backend/main.py
        # Callback the SceneBridge registers; plugins may emit out-of-band
        # scene updates outside their update() tick (e.g. an ICP service
        # announcing a new merged cloud the instant it finishes).
        self.emit = None            # callable(SceneUpdate)
