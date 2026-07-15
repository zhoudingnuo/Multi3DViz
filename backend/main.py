"""main.py — Multi3DViz Python backend (Electron sidecar).

Runs an asyncio WebSocket server. Electron spawns this, reads the bound port
from stdout (line "READY ws://127.0.0.1:PORT"), and points the renderer at it.

Responsibilities:
  - bind a random free port (avoid clashes / easy multi-instance)
  - discover plugins, hold a PluginRegistry + PluginContext + SceneBridge
  - on each WS connection: handshake, then drive the tick loop (call
    Display/Service update(dt), serialize SceneUpdates, push frames)
  - dispatch JSON control messages from the frontend (enable_plugin etc.)

Thread/async note: heavy compute (ICP, voxel downsample) must stay off the
event loop. Plugins that need it run work in asyncio executors or background
threads and emit SceneUpdates via ctx.emit when ready. Phase 1's point-cloud
plugin does light per-frame slicing and colors on the loop.
"""
from __future__ import annotations
import os
import sys
import json
import asyncio
import logging
import time

# --- WHEA BSOD guard (carried over from ccenter) ---
# Cap OpenMP/MKL/OpenBLAS threads BEFORE importing open3d/numpy-heavy paths.
# ccenter historically hit WHEA errors when ICP pinned all cores; we keep the
# same defensive cap here even though Phase 1 doesn't run ICP yet.
_MAX_THREADS = max(2, (os.cpu_count() or 4) // 2)
os.environ.setdefault("OMP_NUM_THREADS", str(_MAX_THREADS))
os.environ.setdefault("MKL_NUM_THREADS", str(_MAX_THREADS))
os.environ.setdefault("OPENBLAS_NUM_THREADS", str(_MAX_THREADS))

# --- Optional-dep stubs (slim PyInstaller build) ---
# open3d's import chain pulls in plotly/dash/matplotlib at module top level.
# In the slim build these are excluded to keep the bundle <1G. Install a meta
# path finder that auto-stubs any of a known set of "viz-only" packages when
# they're not installed, so open3d's `import plotly`/`import dash` succeeds.
# The stubbed code paths (visualization/export) are never actually used by the
# backend's data pipeline. This must happen BEFORE any `import open3d`.
import importlib.abc as _ilabc
import types as _types
_STUB_PACKAGES = {
    "plotly", "dash", "matplotlib", "werkzeug", "flask",
    "retry", "tenacity", "pandas", "nbformat",
}
class _StubFinder(_ilabc.MetaPathFinder):
    """Return a stub module for any name under _STUB_PACKAGES (e.g.
    plotly.graph_objects, dash.dcc, matplotlib.pyplot). The stub is a package
    (has __path__) so submodule imports are tolerated too."""
    def find_spec(self, fullname, path, target=None):
        top = fullname.split(".")[0]
        if top in _STUB_PACKAGES and fullname not in sys.modules:
            import importlib.machinery as im
            m = self._make_stub(fullname)
            return im.ModuleSpec(fullname, self, is_package=True)
        return None
    def _make_stub(self, fullname):
        m = _types.ModuleType(fullname)
        m.__path__ = []
        m.__all__ = []
        sys.modules[fullname] = m
        return m
    # Legacy loader API — needed by PyInstaller's frozen bootloader which
    # runs a Python that still calls load_module after find_spec.
    def load_module(self, fullname):
        if fullname not in sys.modules:
            self._make_stub(fullname)
        return sys.modules[fullname]
    def create_module(self, spec):
        return self._make_stub(spec.name)
    def exec_module(self, module):
        pass
# Insert at the FRONT of meta_path so it wins over the "not found" fallback
# (but only for the stubbed set — real imports still work).
sys.meta_path.insert(0, _StubFinder())

# Make backend/ importable as the package root (so `from core... import`,
# `from plugins... import`, `from lib... import` work regardless of cwd).
# When PyInstaller-frozen, the bundled modules live under sys._MEIPASS (the
# _internal/ dir of the onedir bundle), and __file__ is the exe path — so we
# point at sys._MEIPASS in that case. Otherwise (dev) it's backend/'s parent.
if getattr(sys, "frozen", False) and hasattr(sys, "_MEIPASS"):
    sys.path.insert(0, sys._MEIPASS)              # frozen: _internal/
    sys.path.insert(0, os.path.join(sys._MEIPASS, "backend"))
else:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Make previously pip-installed optional deps (torch etc.) importable before
# any plugin discovery. optional_deps.setup_runtime_path() inserts
# userData/torch_runtime/ onto sys.path if it exists.
from core import optional_deps
optional_deps.setup_runtime_path()

import websockets
from websockets.exceptions import ConnectionClosed

from core.plugin_base import PluginContext, SceneUpdate
from core.plugin_registry import PluginRegistry
from core.scene_bridge import SceneBridge
from core.robot_manager import RobotManager, RobotConfig
from core.config_store import ConfigStore
from core.logger import install_excepthooks, mem_mb, cpu_pct
from core import ws_protocol as proto

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    stream=sys.stderr,  # keep stdout clean for the READY line
)
log = logging.getLogger("multi3dviz.main")

# Install crash hooks FIRST so any startup failure produces a crash log.
install_excepthooks()

HOST = "127.0.0.1"
TICK_HZ = 30                  # target plugin update rate

# Default robot fleet seeded on first run (empty config). Matches the ccenter
# deployment: Unitree Go2 (Robot A, password auth) + Agibot D1 (Robot B, key
# auth). The user can edit/remove these from the right-side control panel.
DEFAULT_ROBOTS = [
    {"robot_id": "robot_a", "host": "10.60.77.187", "port": 22,
     "user": "unitree", "password": "123",
     "label": "Unitree Go2",
     "data_path": "", "launch_cmd": "/home/unitree/sda2/restart_all.sh"},
    {"robot_id": "robot_b", "host": "10.60.77.154", "port": 22,
     "user": "orin-001", "password": None,
     "label": "Agibot D1",
     "data_path": "", "launch_cmd": "/home/orin-001/sda2/restart_all.sh"},
]


class Backend:
    """Holds all shared state for one running backend instance."""

    def __init__(self):
        self.ctx = PluginContext()
        self.bridge = SceneBridge()
        # Persistent config: plugin props + robot fleet + app settings.
        # Lives next to the backend entry (dev). Packaging can redirect to
        # %APPDATA% via an env override.
        cfg_path = os.environ.get(
            "M3V_CONFIG_PATH",
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "multi3dviz_config.json"))
        self.config = ConfigStore(cfg_path)
        self.ctx.config_store = self.config
        # registry built once at startup; plugins discovered synchronously.
        # NOTE: the ConfigStore must be on ctx BEFORE the registry builds, so
        # plugin __init__ can load persisted property values.
        self.registry = PluginRegistry(self.ctx)
        # Robot manager (dynamic multi-robot SSH). on_status fires from
        # heartbeat threads — we marshal it onto the asyncio loop via
        # call_soon_threadsafe so we can push robot_status to the client.
        self.robots = RobotManager(on_status=self._on_robot_status)
        self.ctx.robots = self.robots
        self._loop = asyncio.get_event_loop()
        # the single live frontend connection (Phase 1 = one client)
        self.client = None
        # wire ctx.emit so services can push out-of-band updates
        self.ctx.emit = self._emit_update
        # queue of outgoing frames (str|bytes) to flush from the writer task
        self._outbox: asyncio.Queue = None
        # pending scene updates accumulated between ticks
        self._pending: list[SceneUpdate] = []
        # playback-state broadcast accumulator (every ~0.25s)
        self._pb_t = 0.0
        # registration-status broadcast accumulator (every ~0.5s)
        self._reg_t = 0.0
        # process-stats broadcast accumulator (every ~1s)
        self._stat_t = 0.0
        # battery query accumulator (every ~30s — SSH query is slow)
        self._batt_t = 0.0

    # --- connection lifecycle ---
    async def serve(self, websocket):
        """Per-connection handler. One client at a time in Phase 1."""
        if self.client is not None:
            # Reject extra clients — Phase 1 is single-client.
            await websocket.close(code=1013, reason="client already connected")
            return
        self.client = websocket
        self._outbox = asyncio.Queue()
        log.info("frontend connected from %s", websocket.remote_address)
        # Start the writer task that drains the outbox, and the tick loop.
        writer = asyncio.create_task(self._writer_loop(websocket))
        ticker = asyncio.create_task(self._tick_loop())
        try:
            await websocket.send(proto.make_msg("ready"))
            # Push the initial catalog + state so the UI can render panels.
            await websocket.send(proto.make_msg("catalog",
                                                plugins=self.registry.catalog()))
            await websocket.send(proto.make_msg("state",
                                                enabled=self.registry.enabled_list()))
            # Restore the enabled-plugin instances from config (so the user's
            # last session's panel choices survive restart). The saved value
            # is a list of {name, instance_id}. On first run (no saved state)
            # fall back to each plugin's default_enabled.
            saved = self.config.get_app("enabled_plugins")
            if saved and isinstance(saved, list):
                # Restore each saved instance by its instance_id.
                for entry in saved:
                    self.registry.enable(entry["name"],
                                         instance_id=entry.get("instance_id"))
            else:
                # First run: create the default instance set (e.g. dual-robot
                # LocalReplay + PointCloud) declared by each plugin class.
                self.registry.enable_defaults()
            await websocket.send(proto.make_msg("state",
                                                enabled=self.registry.enabled_list()))
            # Restore the saved robot fleet (best-effort — hosts may be offline).
            saved_robots = self.config.get_robots()
            if not saved_robots:
                # First run: seed the default dual-robot fleet (Unitree A +
                # Agibot B) so the right panel isn't empty. These match the
                # ccenter deployment (unitree@10.60.77.187 pw:123, agibot
                # key-auth). The user can edit/remove them from the UI.
                saved_robots = DEFAULT_ROBOTS
            for rcfg in saved_robots:
                self.robots.add(RobotConfig(**{k: v for k, v in rcfg.items()
                                              if k in RobotConfig.__dataclass_fields__}))
            if self.robots.all():
                await websocket.send(proto.make_msg("robot_status",
                                                    robots=self.robots.list_state()))
            # Persist the resolved enabled set so first-run defaults get saved
            # (subsequent restarts restore exactly this set).
            self._persist_enabled()
            # Wire the ICP service's progress forwarder so its on_progress
            # callbacks reach the client as registration_progress events.
            icp = self.registry.get("ICPRegistration")
            if icp is not None:
                icp.set_progress_forwarder(self._forward_registration)
            # Expose the ICP instance to other plugins (explorer reads its
            # transform). ctx.icp_ref is set once and re-read each tick.
            self.ctx.icp_ref = icp
            # Main receive loop.
            async for raw in websocket:
                if isinstance(raw, bytes):
                    continue  # backend doesn't accept binary from frontend yet
                await self._handle_text(websocket, raw)
        except ConnectionClosed:
            pass
        except Exception:
            log.exception("connection handler crashed")
        finally:
            ticker.cancel()
            writer.cancel()
            self.client = None
            self._outbox = None
            log.info("frontend disconnected")

    # --- message dispatch ---
    async def _handle_text(self, ws, text: str):
        try:
            msg = proto.parse(text)
        except ValueError:
            await ws.send(proto.make_error(-1, "bad json"))
            return
        mtype = msg.get("type")
        rid = msg.get("id")
        if mtype == "hello":
            await ws.send(proto.make_response(rid, ok=True,
                                              server="multi3dviz", version="0.1"))
        elif mtype == "list_plugins":
            await ws.send(proto.make_response(rid,
                                              plugins=self.registry.catalog()))
        elif mtype == "enable_plugin":
            # Toggle a plugin. For singletons: enable if off. For multi-instance
            # types: same as add_instance (creates a new instance). Returns the
            # instance_id so the frontend can track the new row.
            name = msg.get("name")
            iid = self.registry.enable(name) if name else None
            await ws.send(proto.make_response(rid, ok=bool(iid), instance_id=iid))
            await self._broadcast_state()
            self._persist_enabled()
        elif mtype == "add_instance":
            # Explicitly create a new instance of a multi-instance plugin.
            name = msg.get("name")
            iid = self.registry.enable(name) if name else None
            await ws.send(proto.make_response(rid, ok=bool(iid), instance_id=iid))
            await self._broadcast_state()
            self._persist_enabled()
        elif mtype == "disable_plugin":
            # Disable/remove by instance_id (or by name for singletons).
            key = msg.get("instance_id") or msg.get("name")
            ok = self.registry.disable(key) if key else False
            await ws.send(proto.make_response(rid, ok=ok))
            await self._broadcast_state()
            self._persist_enabled()
        elif mtype == "set_property":
            # Set a property on a specific instance (by instance_id) or a
            # singleton (by name). When name is used and multiple instances
            # exist, apply to ALL matching instances (e.g. stream_mode on
            # every LocalReplay).
            iid = msg.get("instance_id")
            name = msg.get("name")
            pkey, val = msg.get("key"), msg.get("value")
            if iid:
                ok = self.registry.set_property(iid, pkey, val)
            elif name:
                # Apply to all instances matching this plugin name.
                ok = False
                for e in self.registry.enabled_list():
                    if e["name"] == name:
                        if self.registry.set_property(e["instance_id"], pkey, val):
                            ok = True
            else:
                ok = False
            await ws.send(proto.make_response(rid, ok=ok))
        elif mtype == "get_state":
            await ws.send(proto.make_response(rid,
                                              enabled=self.registry.enabled_list()))
        elif mtype == "playback":
            # Playback control: route to the active LocalReplay (or any source
            # exposing control()/playback_state()). action: play|pause|toggle|
            # seek|rate. value: frame int (seek) or float (rate).
            action = msg.get("action")
            value = msg.get("value")
            handled = self._route_playback(action, value)
            await ws.send(proto.make_response(rid, ok=handled))
        elif mtype == "robot_add":
            cfg = self._robot_config_from_msg(msg)
            ok = self.robots.add(cfg) if cfg else False
            await ws.send(proto.make_response(rid, ok=ok))
            if ok:
                self._persist_fleet()
        elif mtype == "robot_remove":
            rid2 = msg.get("robot_id")
            ok = self.robots.remove(rid2) if rid2 else False
            await ws.send(proto.make_response(rid, ok=ok))
            if ok:
                self._persist_fleet()
        elif mtype == "robot_list":
            await ws.send(proto.make_response(rid, robots=self.robots.list_state()))
        elif mtype == "robot_command":
            # SSH command on a robot: action launch|stop|estop|toggle_pose|vel|run.
            # Run in executor — SSH calls can take seconds (DDS init, timeout)
            # and would block the asyncio loop if called inline.
            inst = self.registry.get("SSHLauncher")
            if inst is None:
                self.registry.enable("SSHLauncher")
                inst = self.registry.get("SSHLauncher")
            loop = asyncio.get_event_loop()
            if inst is not None:
                result = await loop.run_in_executor(
                    None, inst.command, msg.get("robot_id"),
                    msg.get("action"), msg.get("value"))
            else:
                result = {"ok": False, "error": "SSHLauncher not available"}
            await ws.send(proto.make_response(rid, **result))
        elif mtype == "robot_vel":
            # Keyboard takeover velocity: {robot_id, vx, vy, yaw}. Fire-and-forget
            # (no response — frontend sends ~10Hz, acking each would flood).
            # Routes to SSHLauncher vel action which SSH-sends to the robot.
            inst = self.registry.get("SSHLauncher")
            if inst is None:
                self.registry.enable("SSHLauncher")
                inst = self.registry.get("SSHLauncher")
            if inst is not None:
                inst.command(msg.get("robot_id"), "vel",
                             {"vx": float(msg.get("vx", 0)),
                              "vy": float(msg.get("vy", 0)),
                              "yaw": float(msg.get("yaw", 0))})
        elif mtype == "register":
            # Force (re-)run of ICP registration between source_a/source_b.
            inst = self.registry.get("ICPRegistration")
            if inst is None:
                self.registry.enable("ICPRegistration")
                inst = self.registry.get("ICPRegistration")
                if inst is not None:
                    inst.set_progress_forwarder(self._forward_registration)
            if inst is not None:
                inst.force_reregister()
            await ws.send(proto.make_response(rid, ok=inst is not None))
        elif mtype == "set_target":
            # Manual navigation target: user clicked a world point for a robot.
            # robot_id: 'robot_a'|'robot_b'; world: [x, y] merged-frame meters.
            ex = self.registry.get("DualAgentExplorer")
            rid2 = msg.get("robot_id", "robot_a")
            agent = 0 if rid2.endswith("a") or rid2.endswith("A") else 1
            world = msg.get("world", [0, 0])
            ok = ex.set_manual_target(agent, (float(world[0]), float(world[1]))) \
                if ex is not None else False
            await ws.send(proto.make_response(rid, ok=ok))
        elif mtype == "export_trajectory":
            # Render a trajectory PNG (grid + trails + targets) and return the
            # path so the user can find the file. Reuses ccenter's
            # trajectory_plot.save_trajectory_figure.
            path = self._export_trajectory()
            await ws.send(proto.make_response(rid, ok=bool(path), path=path))
        elif mtype == "install_dependency":
            # In-app optional dependency install (e.g. torch for Semantics).
            # Acks immediately, runs pip in a daemon thread, streams progress
            # via install_progress events (same marshal pattern as ICP register).
            pkg = msg.get("name", "torch")
            if optional_deps.is_installed(pkg):
                await ws.send(proto.make_response(rid, ok=True, already_installed=True))
            else:
                optional_deps.install(pkg,
                                      on_progress=self._forward_install,
                                      on_done=lambda ok, m: self._on_install_done(pkg, ok, m))
                await ws.send(proto.make_response(rid, ok=True, installing=True))
        elif mtype == "semantics_trigger":
            # Manual semantic inference trigger (button in gridmap section).
            inst = self.registry.get("Semantics")
            if inst is None:
                self.registry.enable("Semantics")
                inst = self.registry.get("Semantics")
            if inst is not None:
                inst.force_predict()
            await ws.send(proto.make_response(rid, ok=inst is not None))
        elif mtype == "confirm_targets":
            # Manual confirm frontier targets (Enter key in non-auto explore mode).
            ex = self.registry.get("DualAgentExplorer")
            if ex is not None:
                ok = ex.confirm_targets()
            else:
                ok = False
            await ws.send(proto.make_response(rid, ok=ok))
        else:
            await ws.send(proto.make_error(rid, f"unknown type {mtype!r}"))

    @staticmethod
    def _robot_config_from_msg(msg) -> RobotConfig:
        """Build a RobotConfig from a robot_add message payload."""
        rid = msg.get("robot_id")
        if not rid:
            return None
        pw = msg.get("password")
        return RobotConfig(
            robot_id=rid,
            host=msg.get("host", ""),
            port=int(msg.get("port", 22)),
            user=msg.get("user", ""),
            password=pw if pw else None,   # empty → key auth
            data_path=msg.get("data_path", ""),
            launch_cmd=msg.get("launch_cmd", ""),
            label=msg.get("label", "") or rid,
        )

    def _on_robot_status(self, robot_id, state, error):
        """Called from heartbeat threads (NOT the asyncio loop). Marshal onto
        the loop so we can push robot_status to the client safely."""
        self._loop.call_soon_threadsafe(self._push_robot_status, robot_id, state, error)

    def _push_robot_status(self, robot_id, state, error):
        """On the loop: snapshot the full robot list + push to the client."""
        if self.client is None or self._outbox is None:
            return
        self._outbox.put_nowait(proto.make_msg(
            "robot_status",
            robots=self.robots.list_state(),
            changed={"robot_id": robot_id, "state": state, "error": error},
        ))

    def _forward_registration(self, payload):
        """ICP progress callback (called from the ICP worker thread). Marshal
        onto the loop + push as a registration_progress event. Carries per-trial
        fitness/rmse/score and phase=start|try|done so the UI can render a
        live progress readout."""
        self._loop.call_soon_threadsafe(self._push_registration, payload)

    def _push_registration(self, payload):
        if self.client is None or self._outbox is None:
            return
        self._outbox.put_nowait(proto.make_msg("registration_progress", **payload))

    # --- optional dependency install (in-app "Install torch" button) ---
    def _forward_install(self, payload):
        """pip install progress callback (from the install daemon thread).
        Marshal onto the loop + push as install_progress — same pattern as
        _forward_registration."""
        self._loop.call_soon_threadsafe(self._push_install, payload)

    def _push_install(self, payload):
        if self.client is None or self._outbox is None:
            return
        self._outbox.put_nowait(proto.make_msg("install_progress", **payload))

    def _on_install_done(self, pkg, ok, message):
        """Called from the install thread when pip finishes. If torch just got
        installed, re-enable the Semantics plugin so it picks up the new dep."""
        log.info("install %s %s: %s", pkg, "ok" if ok else "FAILED", message)
        if ok and pkg == "torch":
            # Re-import sem_infer (its _try_import_deps cached None; a fresh
            # module reload picks up the now-importable torch).
            import importlib
            try:
                import lib.sem_infer as _si
                importlib.reload(_si)
            except Exception:
                log.exception("reload sem_infer failed")
            # Push a plugin_status event so the UI updates the Install button.
            self._loop.call_soon_threadsafe(self._push_plugin_status)
        # Clear the install state after a short delay so the UI sees "done".
        def _clear():
            import time as _t; _t.sleep(3)
            optional_deps.clear_state()
        import threading as _th
        _th.Thread(target=_clear, daemon=True).start()

    def _push_plugin_status(self):
        if self.client is None or self._outbox is None:
            return
        # Re-compute missing deps for all plugins and broadcast.
        statuses = []
        for name in optional_deps.PLUGIN_DEPS:
            statuses.append({
                "name": name,
                "missing_deps": optional_deps.get_missing(name),
                "available": len(optional_deps.get_missing(name)) == 0,
            })
        self._outbox.put_nowait(proto.make_msg("plugin_status", plugins=statuses))

    def _route_playback(self, action, value) -> bool:
        """Send a playback command to the first enabled source plugin that
        implements control(). Returns True if a source accepted it."""
        for name, inst in self.registry._instances.items():
            if inst.category != "source":
                continue
            if hasattr(inst, "control"):
                try:
                    inst.control(action, value)
                    return True
                except Exception as e:
                    log.warning("playback control on %s failed: %s", name, e)
        return False

    async def _broadcast_state(self):
        if self.client is None:
            return
        await self.client.send(proto.make_msg("state",
                                              enabled=self.registry.enabled_list()))

    def _persist_enabled(self):
        """Save the live instance list (name + instance_id) so it survives
        restart. Multi-instance plugins restore as separate instances."""
        instances = [{"name": inst.name, "instance_id": iid}
                     for iid, inst in self.registry._instances.items()]
        self.config.set_app("enabled_plugins", instances)

    def _persist_fleet(self):
        """Snapshot the current robot configs (id/host/user/...) to config.
        Connection state isn't saved — only identity."""
        import dataclasses
        robots = [dataclasses.asdict(c.cfg) for c in self.robots.all().values()]
        self.config.set_robots(robots)

    def _info_state(self) -> dict:
        """Aggregate the live app state for the status/info panel — mirrors
        ccenter's info_state dict (frame, points, reg metrics, exploration
        progress). Reads from active plugins + the data bus."""
        info = {
            "frame": 0, "max_frame": 0,
            "pts_a": 0, "pts_b": 0,
            "reg_status": "idle", "reg_fitness": 0.0, "reg_rmse": 0.0,
            "n_frontiers": 0, "explored_pct": 0.0,
            "robots_online": 0, "robots_total": 0,
        }
        # Per-robot frame + points from the data bus.
        fa = self.ctx.data.latest("robot_a")
        fb = self.ctx.data.latest("robot_b")
        if fa:
            info["frame"] = fa.get("frame_idx", 0)
            info["max_frame"] = fa.get("max_frame", 0)
            info["pts_a"] = len(fa.get("positions", []))
        if fb:
            info["pts_b"] = len(fb.get("positions", []))
        # Registration state from ICPRegistration.
        icp = self.registry.get("ICPRegistration")
        if icp is not None:
            snap = icp.state_snapshot()
            info["reg_status"] = snap["state"]
            info["reg_fitness"] = snap["fitness"]
            info["reg_rmse"] = snap["rmse"]
        # Exploration state from DualAgentExplorer.
        ex = self.registry.get("DualAgentExplorer")
        if ex is not None and getattr(ex, "_explorer", None) is not None:
            e = ex._explorer
            info["n_frontiers"] = len(e.frontier_clusters)
            if e.explored is not None and e.explored.size > 0:
                info["explored_pct"] = round(
                    100.0 * float(e.explored.sum()) / e.explored.size, 1)
        # Robot connection tally.
        robots = self.robots.list_state()
        info["robots_total"] = len(robots)
        info["robots_online"] = sum(1 for r in robots if r["state"] == "online")
        return info

    def _export_trajectory(self) -> str | None:
        """Render a trajectory PNG from the explorer's current state. Returns
        the saved path or None if the explorer has no grid yet."""
        ex = self.registry.get("DualAgentExplorer")
        if ex is None or getattr(ex, "_gmap", None) is None or \
                getattr(ex, "_explorer", None) is None:
            return None
        import os as _os
        from lib.trajectory_plot import save_trajectory_figure, default_save_path
        out_dir = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                                "..", "output")
        _os.makedirs(out_dir, exist_ok=True)
        path = default_save_path(out_dir)
        # Build the robots arg expected by save_trajectory_figure: per-robot
        # trail + color. Targets from the explorer.
        robots = [
            {"name": "Robot A", "trail": ex._traj_a, "color": "#E74C3C"},
            {"name": "Robot B", "trail": ex._traj_b, "color": "#3498DB"},
        ]
        targets = []
        e = ex._explorer
        for i in (0, 1):
            if e.targets[i] is not None:
                gy, gx = e.targets[i]
                targets.append(e.grid_to_world(gy, gx))
            else:
                targets.append(None)
        coverage = e.explored if e.explored is not None else None
        try:
            return save_trajectory_figure(ex._gmap, robots, path,
                                          coverage_mask=coverage, targets=targets)
        except Exception as e2:
            log.warning("trajectory export failed: %s", e2)
            return None

    # --- tick loop: drive plugins, serialize updates, enqueue frames ---
    async def _tick_loop(self):
        """Call every enabled Display/Service update(dt), collect their
        SceneUpdates, merge, serialize and enqueue frames to the outbox."""
        last = time.monotonic()
        period = 1.0 / TICK_HZ
        while True:
            try:
                await asyncio.sleep(period)
            except asyncio.CancelledError:
                return
            now = time.monotonic()
            dt = now - last
            last = now
            # Run plugin updates in a thread so a slow numpy op can't stall
            # the WS loop. Phase 1 ops are fast, but this is the right shape.
            updates = await asyncio.get_event_loop().run_in_executor(
                None, self.registry.tick, dt)
            for upd in updates:
                self._pending.append(upd)
            # Merge + serialize + enqueue.
            if self._pending and self._outbox is not None:
                merged = self._merge_updates(self._pending)
                self._pending.clear()
                for frame in self.bridge.serialize(merged):
                    await self._outbox.put(frame)
            # Periodically broadcast playback state so the UI's seek bar + play
            # button stay in sync (covers looping, programmatic seeks, etc.).
            self._pb_t += dt
            if self._pb_t >= 0.25 and self._outbox is not None:
                self._pb_t = 0.0
                states = [inst.playback_state()
                          for inst in self.registry._instances.values()
                          if inst.category == "source" and hasattr(inst, "playback_state")]
                if states and self.client is not None:
                    await self._outbox.put(proto.make_msg("playback_state",
                                                          sources=states))
            # Periodically broadcast registration status (state/fitness/rmse)
            # so the UI panel stays current even between progress events.
            self._reg_t += dt
            if self._reg_t >= 0.5 and self._outbox is not None:
                self._reg_t = 0.0
                icp = self.registry.get("ICPRegistration")
                if icp is not None and self.client is not None:
                    await self._outbox.put(proto.make_msg("registration_status",
                                                          **icp.state_snapshot()))
            # Keep ctx.icp_ref current so the explorer service can read the
            # transform even if ICP was enabled after startup.
            self.ctx.icp_ref = self.registry.get("ICPRegistration")
            # Likewise expose the explorer instance so the semantics service
            # can read its merged grid.
            self.ctx.explorer_ref = self.registry.get("DualAgentExplorer")
            # Flush persisted config (plugin props / fleet / enabled set) if
            # dirty. Debounced inside ConfigStore to ~1 write/sec.
            self.config.maybe_save()
            # Periodic process stats (mem/CPU) for the status bar — 1Hz.
            self._stat_t += dt
            if self._stat_t >= 1.0 and self._outbox is not None and self.client is not None:
                self._stat_t = 0.0
                await self._outbox.put(proto.make_msg(
                    "process_stats",
                    mem_mb=round(mem_mb(), 1), cpu_pct=round(cpu_pct(), 1)))
                # Rich info state (frames/points/reg/exploration) aggregated from
                # the active plugins — mirrors ccenter's ui_info panel.
                await self._outbox.put(proto.make_msg("info_state",
                                                      **self._info_state()))
            # Periodic battery query — every 30s, SSH to each online robot.
            # Runs in executor so the slow SSH call doesn't stall the tick loop.
            self._batt_t += dt
            if self._batt_t >= 30.0 and self.robots is not None:
                self._batt_t = 0.0
                loop = asyncio.get_event_loop()
                loop.run_in_executor(None, self._query_batteries)

    def _query_batteries(self):
        """Query battery % for all online robots via SSH. Runs in executor."""
        inst = self.registry.get("SSHLauncher")
        if inst is None:
            return
        for rid, conn in self.robots.all().items():
            if conn.state != "online":
                continue
            try:
                result = inst.command(rid, "battery")
                pct = result.get("pct", -1)
                conn.battery_pct = pct
            except Exception:
                pass
        # Push updated robot_status with battery levels.
        if self._outbox is not None and self.client is not None:
            self._loop.call_soon_threadsafe(lambda: self._outbox.put_nowait(
                proto.make_msg("robot_status", robots=self.robots.list_state())))

    @staticmethod
    def _merge_updates(updates: list[SceneUpdate]) -> SceneUpdate:
        """Concatenate add/update/remove across multiple SceneUpdates into one.
        Remove takes precedence: if an id is in any remove list, it's stripped
        from adds/upds so a later plugin's update() can't resurrect a cloud
        that ICP just removed (the classic "two clouds still showing" bug)."""
        merged = SceneUpdate()
        adds, upds = {}, {}
        removes = set()
        for u in updates:
            for o in u.add:
                adds[o.id] = o
            for o in u.update:
                upds[o.id] = o
            removes.update(u.remove)
        # Remove wins — drop any add/update for ids that are being removed.
        if removes:
            for rid in removes:
                adds.pop(rid, None)
                upds.pop(rid, None)
        merged.add = list(adds.values())
        merged.update = list(upds.values())
        merged.remove = list(removes)
        return merged

    async def _writer_loop(self, ws):
        """Drain the outbox and send frames in order. Keeps send() calls on
        one task to avoid interleaving binary header/body across coroutines."""
        while True:
            try:
                frame = await self._outbox.get()
            except asyncio.CancelledError:
                return
            try:
                if isinstance(frame, str):
                    await ws.send(frame)
                else:
                    await ws.send(frame)
            except ConnectionClosed:
                return
            self._outbox.task_done()

    # --- out-of-band emission (services announcing results mid-tick) ---
    def _emit_update(self, update: SceneUpdate):
        """Called from plugin threads (not the loop). Schedule serialization."""
        if update is None:
            return
        self._pending.append(update)


async def main():
    global _backend_ref
    backend = Backend()
    _backend_ref = backend
    # Bind 0 = let the OS pick a free port. We read it back for the READY line.
    server = await websockets.serve(backend.serve, HOST, 0,
                                    max_size=256 * 1024 * 1024,  # 256MB for big clouds
                                    compression=None)            # binary stays raw
    # Extract the actually-bound port from the server's sockets.
    sock = server.sockets[0]
    port = sock.getsockname()[1]
    # READY line — Electron parses this. Print to stdout, flush immediately.
    print(f"READY ws://{HOST}:{port}", flush=True)
    log.info("listening on ws://%s:%d", HOST, port)
    log.info("discovered %d plugins", len(backend.registry.catalog()))
    await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    finally:
        # Stop all robot heartbeats + close SSH clients on exit so we don't
        # leave zombie connections to the robots.
        try:
            _backend_ref.robots.shutdown()
        except Exception:
            pass
        # Force-save config so the user's last session (plugin props, fleet,
        # enabled set) survives the next restart.
        try:
            _backend_ref.config.force_save()
        except Exception:
            pass


# Module-level handle so the finally block above can reach the backend for
# robot cleanup. Set in main().
_backend_ref = None
