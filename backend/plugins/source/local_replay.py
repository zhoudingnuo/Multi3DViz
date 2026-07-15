"""local_replay.py — DataSource plugin: replay a recorded robot run from disk.

Reuses ccenter's player.load_frames / load_odometry and data_utils.load_gravity.
A run directory looks like:
    <data_root>/<robot>/<run>/
        cloud_registered/*.npy   (N,3) point frames
        Odometry/*.json          {x,y,z,qx,qy,qz,qw} per frame
        gravity_calibration.json roll/pitch correction
On enable it loads everything, then on each update() advances the playback
cursor by `playback_rate` and publishes the LATEST cumulative point cloud into
the data bus (keyed by this source's robot_id). Downstream Displays read it.

This mirrors ccenter's incremental accumulation: instead of re-sending every
frame's points, we publish the accumulated cloud so a Display can compute the
delta (only new points since last push) — keeping WS traffic proportional to
new data, not total history.
"""
from __future__ import annotations
import os
import sys
import time
import glob
import threading
import logging
import numpy as np

# Ensure backend root is importable when this plugin is loaded by the registry.
_THIS = os.path.dirname(os.path.abspath(__file__))
_BACKEND = os.path.abspath(os.path.join(_THIS, "..", ".."))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)

from core.plugin_base import DataSourcePlugin
from lib import player, data_utils
from lib.data_utils import cap_accum

log = logging.getLogger("multi3dviz.source.local_replay")

DEFAULT_DATA_ROOT = r"C:\Users\Z790\ccenter"  # ccenter's data lives here


class LocalReplaySource(DataSourcePlugin):
    name = "LocalReplay"
    category = "source"
    description = "Replay a recorded robot run (cloud_registered + Odometry) from disk."
    default_enabled = True
    multiple = True              # one instance per robot
    # Default to BOTH robots so a fresh install shows the dual-robot scene
    # (unitree + agibot) without the user having to add a second instance.
    default_instances = [
        {"robot": "unitree", "robot_id": "robot_a"},
        {"robot": "agibot", "robot_id": "robot_b"},
    ]

    properties = {
        "data_root": {
            "type": "path",
            "default": DEFAULT_DATA_ROOT,
            "label": "Data root (contains <robot>/<run> folders)",
            "group": "Source",
        },
        "robot": {
            "type": "select",
            "options": ["unitree", "agibot"],
            "default": "unitree",
            "label": "Robot",
            "group": "Source",
        },
        "robot_id": {
            "type": "string",
            "default": "robot_a",
            "label": "Robot ID (data bus key)",
            "group": "Source",
        },
        "playback_rate": {
            "type": "float",
            "default": 1.0,
            "min": 0.0, "max": 100.0, "step": 0.5,
            "label": "Playback speed (x)",
            "group": "Playback",
        },
        "instant_load": {
            "type": "bool",
            "default": True,
            "label": "Instant load (skip to last frame, no playback)",
            "group": "Mode",
        },
        "voxel_size": {
            "type": "float",
            "default": 0.1,
            "min": 0.01, "max": 1.0, "step": 0.01,
            "label": "Voxel downsample (m)",
            "group": "Quality",
        },
        "stream_mode": {
            "type": "bool",
            "default": False,
            "label": "Live stream (poll new frames as they're written)",
            "group": "Mode",
        },
    }

    def __init__(self, ctx):
        super().__init__(ctx)
        self._frames = None        # list of (N,3) gravity-corrected frames
        self._odom = None
        self._vis_pts = None       # per-frame voxel-downsampled (for display)
        self._vis_cum = None       # cumulative offsets into vis_pts
        self._vis_colors = None    # (M,3) colors over the whole run
        self._n = 0
        self._cursor = 0.0         # float playback position (frames)
        self._last_pushed = 0      # last frame index pushed to the bus
        self._loaded_root = None
        self._retry_t = 0.0        # accumulates dt while waiting for data
        self._loading = False      # True while the background load thread runs
        self._playing = True       # playback running vs paused
        # Streaming-mode state (poll_new_frames_nonblocking accumulates here).
        self._stream_known = 0     # frames already consumed from disk
        self._stream_known_odom = 0
        self._stream_R = None      # gravity rotation (loaded once)
        # When the user seeks, we must force a re-push even if the new frame
        # index is <= _last_pushed (normal tick path skips in that case).
        self._seek_to = None       # int frame to jump to, or None
        # Publish cache: avoid re-downsampling the same accumulated cloud
        # every tick. Invalidated when f changes or voxel_size changes.
        self._cached_pub_frame = -1
        self._cached_pub_pts = None
        self._cached_pub_cols = None
        self._cached_voxel = -1.0
        self._stream_dedup_t = 0.0  # throttle: re-downsample every 0.5s in stream mode

    # --- lifecycle ---
    def on_enable(self):
        # Derive a unique robot_id from the instance_id if the user hasn't set
        # one explicitly, so two LocalReplay instances don't both publish to
        # robot_a. LocalReplay#1→robot_a, #2→robot_b, #3→robot_c, ...
        if self.get("robot_id", "robot_a") == "robot_a" and self.instance_id != "LocalReplay#1" \
                and self.instance_id.startswith("LocalReplay#"):
            n = int(self.instance_id.split("#")[1])
            # robot_b, robot_c, ... (skip 'a' for non-first instances)
            self._prop_values["robot_id"] = "robot_" + chr(ord('a') + n - 1)
        # Don't load here — both modes resolve lazily in update():
        #   batch → _update_batch triggers _reload on first tick (background thread)
        #   stream → _update_stream polls incrementally
        # Doing it here would race with property setup (data_root not set yet).

    def on_disable(self):
        self._frames = None
        self._odom = None
        self._vis_pts = None

    def on_property_change(self, key, value):
        # Invalidate the publish cache on voxel_size change so the next
        # publish recomputes at the new resolution.
        if key == "voxel_size":
            self._cached_pub_frame = -1
            self._cached_pub_pts = None
        # Mode switch (stream/batch or instant_load): reset state so the
        # source reloads from scratch in the new mode. Without this, switching
        # to stream mode after batch already loaded shows stale history.
        if key in ("stream_mode", "instant_load"):
            self._loaded_root = None
            self._frames = None
            self._vis_pts = None
            self._vis_cum = None
            self._vis_colors = None
            self._odom = None
            self._n = 0
            self._cursor = 0.0
            self._last_pushed = 0
            self._cached_pub_frame = -1
            self._cached_pub_pts = None
            # Cancel any in-progress batch load — the background thread checks
            # this flag and bails out if the mode switched mid-load.
            self._loading = False
            log.info("mode switch: %s=%s → full state reset, batch load cancelled", key, value)
        # Re-load only when data-source identity changes.
        if key in ("data_root", "robot"):
            if self.get("stream_mode", False):
                # Stream mode: reset stream state so update() re-resolves the
                # run dir from the new data_root next tick (no batch load).
                self._loaded_root = None
                self._stream_known = 0
                self._stream_known_odom = 0
                self._frames = None
                self._n = 0
            else:
                self._reload()

    # --- loading ---
    def _run_dir(self) -> str:
        root = self.get("data_root")
        robot = self.get("robot")
        return os.path.join(root, robot, "data")

    def _reload(self):
        if self._loading:
            return  # a load is already in flight
        data_root = self._run_dir()
        log.info("LocalReplay loading from %s", data_root)
        latest = player._latest_run_dir(data_root)
        if latest is None:
            log.warning("no run directory found under %s — idle until data appears",
                        data_root)
            return
        # Heavy load + voxel downsample runs on a background thread so the WS
        # event loop stays responsive. A large run (thousands of frames) can
        # take 20-30s to downsample; doing it on the tick loop would freeze
        # the whole app. update() stays idle until _loading clears.
        self._loading = True
        voxel = float(self.get("voxel_size", 0.1))

        def _worker():
            try:
                frames = player.load_frames(latest)
                odom = player.load_odometry(latest)
                if not frames:
                    log.warning("no cloud_registered frames in %s", latest)
                    return
                _, R = data_utils.load_gravity(os.path.dirname(latest))
                frames = [(R @ f.T).T for f in frames]
                # GLOBAL voxel downsample over ALL frames at once (cross-frame
                # overlap collapses to one point per cell). With instant_load
                # (default), we store this as a single precomputed block and
                # _publish just reuses it every tick — no per-tick recompute.
                if self.get("instant_load", True):
                    vis_all_raw = np.vstack(frames) if frames else np.empty((0, 3))
                    vis_ds = data_utils.voxel_downsample(vis_all_raw, voxel)
                    vis_pts = [vis_ds]
                    vis_cum = [0, len(vis_ds)]
                    vis_colors = data_utils.height_color_blue_red(vis_ds).astype(np.float32)
                else:
                    # Per-frame raw (downsample happens at publish time with cache).
                    vis_pts = frames
                    vis_cum = np.cumsum([0] + [len(p) for p in vis_pts]).tolist()
                    vis_colors = np.zeros((vis_cum[-1], 3), dtype=np.float32)
                self._frames = frames
                self._odom = odom
                self._vis_pts = vis_pts
                self._vis_cum = vis_cum
                self._vis_colors = vis_colors
                self._n = len(frames)
                self._cursor = 0.0
                self._last_pushed = 0
                self._loaded_root = latest
                log.info("LocalReplay ready: %d frames, %d vis pts",
                         self._n, vis_cum[-1])
            except Exception as e:
                log.exception("LocalReplay load failed: %s", e)
            finally:
                self._loading = False

        threading.Thread(target=_worker, daemon=True, name="localreplay-load").start()
        self._loaded_root = latest

    # --- playback control (driven by WS 'playback' messages via backend) ---
    def control(self, action, value=None):
        """Apply a playback command.
            action='play'   -> resume
            action='pause'  -> halt
            action='toggle' -> flip
            action='seek'   -> jump to frame int(value)
            action='rate'   -> set playback_rate property"""
        if action == "play":
            self._playing = True
        elif action == "pause":
            self._playing = False
        elif action == "toggle":
            self._playing = not self._playing
        elif action == "seek":
            if self._frames is not None and self._n > 0:
                target = max(0, min(int(value), self._n))
                self._cursor = float(target)
                self._seek_to = target  # force re-push even if <= last_pushed
        elif action == "rate":
            self.set_property("playback_rate", float(value))

    def playback_state(self):
        """Snapshot for the UI: {playing, frame, max_frame, rate}."""
        return {
            "robot_id": self.get("robot_id", "robot_a"),
            "playing": self._playing,
            "frame": int(self._cursor) if self._n else 0,
            "max_frame": int(self._n or 0),
            "rate": float(self.get("playback_rate", 1.0)),
        }

    # --- per-tick update ---
    def update(self, dt: float):
        """Two modes:
        - batch (default): load everything once (background thread), then
          advance a playback cursor publishing the accumulated cloud.
        - stream (stream_mode=True): poll for NEW frames each tick via
          poll_new_frames_nonblocking, appending them incrementally. Supports
          a robot currently recording — frames appear as they're written."""
        if self.get("stream_mode", False):
            # Stream mode ignores the batch-loading flag (it may be stale from
            # a property change that fired before stream_mode was set).
            self._loading = False
            return self._update_stream(dt)
        if self._loading:
            return None
        return self._update_batch(dt)

    def _update_batch(self, dt: float):
        """Batch mode: with instant_load (default), publish the pre-downsampled
        full cloud once then stay idle. Without instant_load, advance a
        playback cursor frame by frame."""
        if self._frames is None or self._n == 0:
            # Retry load every ~1s until a run directory appears.
            self._retry_t += dt
            if self._retry_t >= 1.0:
                self._retry_t = 0.0
                self._reload()
            return None
        rid = self.get("robot_id", "robot_a")
        # SEEK takes priority — publish exactly the requested frame regardless
        # of play/pause state, so scrubbing works while paused.
        if self._seek_to is not None:
            f = min(self._seek_to, self._n)
            self._seek_to = None
            self._publish(rid, f)
            self._last_pushed = f
            return None
        # instant_load: publish the full pre-downsampled cloud once, then idle.
        # No per-tick cursor advance — the cloud is already complete. The frame
        # counter shows total/max so the UI isn't misleading.
        if self.get("instant_load", True):
            if self._last_pushed < self._n:
                self._publish(rid, self._n)
                self._last_pushed = self._n
            return None
        # Per-frame playback mode (instant_load off): advance cursor.
        if not self._playing:
            return None
        rate = float(self.get("playback_rate", 1.0))
        # ~30 fps base: advance `rate` frames per tick at TICK_HZ=30.
        self._cursor += rate
        f = int(self._cursor)
        if f <= self._last_pushed:
            return None  # no new frame since last push
        f = min(f, self._n)
        self._publish(rid, f)
        self._last_pushed = f
        # Loop playback.
        if f >= self._n and rate > 0:
            self._cursor = 0.0
            self._last_pushed = 0
        return None

    def _update_stream(self, dt: float):
        """Stream mode: incrementally poll NEW frames from the latest run dir
        each tick (non-blocking) and append them. Supports a robot actively
        recording — frames appear as the recorder flushes them to disk.

        FRESHNESS CHECK: if the latest .npy frame is older than 5 minutes,
        the robot is NOT actively recording — fall back to batch/instant_load
        so the user sees the full historical cloud instead of waiting for
        data that isn't coming."""
        # Resolve the run dir once (re-resolve every ~2s in case a new run
        # directory appears when the robot starts a fresh session).
        self._retry_t += dt
        if self._loaded_root is None or self._retry_t >= 2.0:
            self._retry_t = 0.0
            latest = player._latest_run_dir(self._run_dir())
            if latest is None:
                return None
            if latest != self._loaded_root:
                # New/different run dir — reset stream state.
                self._loaded_root = latest
                self._stream_known = 0
                self._stream_known_odom = 0
                self._frames = []
                self._vis_pts = []
                self._vis_cum = [0]
                self._odom = []
                self._n = 0
                self._stream_dedup_t = 0.0
                self._cached_pub_frame = -1
                self._cached_pub_pts = None
                # Gravity correction loaded once per run.
                try:
                    _, self._stream_R = data_utils.load_gravity(
                        os.path.dirname(latest))
                except Exception:
                    self._stream_R = np.eye(3)
        if self._loaded_root is None:
            return None
        # FRESHNESS CHECK: if the latest .npy in this run dir is older than 5
        # FRESHNESS CHECK: if the latest .npy is > 5 min old, the robot is
        # NOT actively recording. Stay in stream mode but don't load stale
        # history — just wait for new data. Do NOT auto-switch to batch (that
        # would override the user's explicit choice and load history).
        cloud_dir = os.path.join(self._loaded_root, "cloud_registered")
        npys = sorted(glob.glob(os.path.join(cloud_dir, "*.npy")))
        if npys:
            age = time.time() - os.path.getmtime(npys[-1])
            if age > 300:  # 5 minutes — stale
                log.info("stream: latest frame %.0fs old — staying in stream, waiting for new data", age)
                return None  # don't load, just wait
        # Ensure stream accumulators exist (may be None if a batch _reload
        # raced and reset them).
        if self._frames is None:
            self._frames = []
            self._vis_pts = []
            self._vis_cum = [0]
            self._odom = []
            self._stream_known = 0
            self._stream_known_odom = 0
        # Poll for new frames (non-blocking — returns [] if none ready).
        voxel = float(self.get("voxel_size", 0.1))
        new_frames, self._stream_known = player.poll_new_frames_nonblocking(
            self._loaded_root, self._stream_known)
        new_odom, self._stream_known_odom = player.poll_new_odometry(
            self._loaded_root, self._stream_known_odom)
        if not new_frames:
            return None  # nothing new this tick
        R = self._stream_R if self._stream_R is not None else np.eye(3)
        for f in new_frames:
            gf = (R @ f.T).T
            self._frames.append(gf)
            self._vis_pts.append(gf)  # raw gravity-corrected (global dedup at _publish)
            self._vis_cum.append(self._vis_cum[-1] + len(gf))
        self._odom.extend(new_odom)
        self._n = len(self._frames)
        rid = self.get("robot_id", "robot_a")
        self._cursor = float(self._n)
        # STREAM PERFORMANCE: throttle the expensive global re-downsample to
        # every ~0.5s instead of every tick. The cloud keeps accumulating raw
        # in _vis_pts, but _publish only re-downsamples when the throttle
        # window elapses. Between re-downsamples, it publishes the last
        # cached result — the cloud visibly grows in bursts every 0.5s rather
        # than stalling the tick loop on every 10Hz frame arrival.
        self._stream_dedup_t += dt
        if self._stream_dedup_t >= 0.5:
            self._stream_dedup_t = 0.0
            self._publish(rid, self._n)
        elif self._cached_pub_pts is not None:
            # Between re-downsamples: publish the cached cloud with updated odom.
            self.ctx.data.publish(rid, {
                "robot_id": rid,
                "frame_idx": self._n,
                "max_frame": self._n,
                "positions": self._cached_pub_pts,
                "colors": self._cached_pub_cols,
                "odom": self._odom[-1] if self._odom else None,
                "all_odom": self._odom,
            })
        self._last_pushed = self._n
        return None

    def _publish(self, rid, f):
        """Publish accumulated points/colors up to frame f into the data bus.

        PERFORMANCE: In batch mode with instant_load (default), we precompute
        the GLOBALLY downsampled cloud ONCE at load time and reuse it on every
        publish — no per-tick re-downsample. The playback cursor just slices
        into the precomputed array. In stream mode or when a new frame arrives,
        we recompute incrementally but cache by frame index so repeated ticks
        with the same frame don't re-downsample."""
        # Fast path: instant_load mode — the full cloud is already downsampled
        # at load time in _reload(). Just publish it with the current frame idx.
        if self.get("instant_load", True) and self._vis_pts and len(self._vis_pts) == 1:
            # _vis_pts[0] is the pre-downsampled full cloud.
            pts = self._vis_pts[0]
            cols = self._vis_colors if self._vis_colors is not None and len(self._vis_colors) == len(pts) \
                else data_utils.height_color_blue_red(pts).astype(np.float32)
            self.ctx.data.publish(rid, {
                "robot_id": rid,
                "frame_idx": f if f > 0 else self._n,
                "max_frame": self._n,
                "positions": pts,
                "colors": cols,
                "odom": self._odom[min(f - 1, len(self._odom) - 1)]
                        if self._odom else None,
                "all_odom": self._odom,
            })
            return

        # Slow path: per-frame downsample with cache (stream mode / instant_load off).
        if f == self._cached_pub_frame and self._cached_pub_pts is not None:
            pts = self._cached_pub_pts
            cols = self._cached_pub_cols
        else:
            pts = self._pts_up_to(f)
            if len(pts) > 0:
                voxel = float(self.get("voxel_size", 0.1))
                pts = data_utils.voxel_downsample(pts, voxel)
                cols = data_utils.height_color_blue_red(pts).astype(np.float32)
            else:
                cols = np.empty((0, 3), dtype=np.float32)
            pts, cols = cap_accum(pts, cols)
            self._cached_pub_frame = f
            self._cached_pub_pts = pts
            self._cached_pub_cols = cols
        self.ctx.data.publish(rid, {
            "robot_id": rid,
            "frame_idx": f,
            "max_frame": self._n,
            "positions": pts,
            "colors": cols,
            "odom": self._odom[min(f - 1, len(self._odom) - 1)]
                    if self._odom else None,
            "all_odom": self._odom,
        })

    def _pts_up_to(self, f: int) -> np.ndarray:
        return np.vstack(self._vis_pts[:f]) if f > 0 else np.empty((0, 3))

    def _colors_up_to(self, f: int) -> np.ndarray:
        return self._vis_colors[:self._vis_cum[f]] if f > 0 else np.empty((0, 3))
