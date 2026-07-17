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
    # Default to BOTH robots so a fresh install shows the dual-robot scene.
    # Per-instance mode so one dog can be online while the other plays back a
    # recorded run:
    #   - robot_a (unitree): LIVE stream — polls cloud_registered/latest.npy
    #     as the robot writes new frames.
    #   - robot_b (agibot): BATCH replay — loads a specific recorded run from
    #     disk (not auto-latest). Set run_dir to pin which run to replay.
    default_instances = [
        {"robot": "unitree", "robot_id": "robot_a", "stream_mode": True},
        {"robot": "agibot", "robot_id": "robot_b",
         "stream_mode": False, "instant_load": True,
         "run_dir": "run_20260701_175044"},
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
        "run_dir": {
            "type": "path",
            "default": "",
            "label": "Specific run dir (empty = auto latest)",
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
            "default": False,
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
            "default": True,
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
        # Incremental voxel accumulator for stream mode: holds the globally
        # downsampled cloud, grown by merging each new (downsampled) frame.
        # Bounded by periodic re-downsample — never vstacks ALL raw history.
        self._stream_accum_pts = np.empty((0, 3), dtype=np.float32)
        self._stream_accum_cols = np.empty((0, 3), dtype=np.float32)
        self._stream_last_mtime = 0.0  # mtime of last processed latest.npy
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
            # Publish an EMPTY frame so the previous mode's cloud is cleared
            # from the frontend immediately. Without this, the last batch cloud
            # stays on screen until the new mode produces data — in stream mode
            # against a stale recording that NEVER happens (freshness gate), so
            # the old batch cloud would linger forever, looking like stream fell
            # back to batch. frame_idx=0 differs from any prior published idx,
            # so PointCloud's dedup check lets the empty frame through and emits
            # a remove op (point_cloud.py handles len(pos)==0 → remove).
            rid = self.get("robot_id", "robot_a")
            self.ctx.data.publish(rid, {
                "robot_id": rid,
                "frame_idx": 0,
                "max_frame": 0,
                "positions": np.empty((0, 3), dtype=np.float32),
                "colors": np.empty((0, 3), dtype=np.float32),
                "odom": None,
                "all_odom": [],
            })
            log.info("mode switch: %s=%s → full state reset, batch load cancelled, cloud cleared",
                     key, value)
        # Re-load only when data-source identity changes.
        if key in ("data_root", "robot", "run_dir"):
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
    def _parent_data_dir(self) -> str:
        """The directory containing run_* subdirs: <data_root>/<robot>/data."""
        root = self.get("data_root")
        robot = self.get("robot")
        return os.path.join(root, robot, "data")

    def _resolve_run(self):
        """Return the concrete run directory to load/watch, or None.

        - If `run_dir` is set, it's used verbatim (absolute) or resolved
          against <data_root>/<robot>/data (relative). The path MUST exist and
          be a directory — a specific run is a hard requirement, not auto-pick.
        - Otherwise auto-pick the latest run subdir under the parent data dir.
        """
        spec = (self.get("run_dir") or "").strip()
        if spec:
            path = spec if os.path.isabs(spec) \
                else os.path.join(self._parent_data_dir(), spec)
            if os.path.isdir(path):
                return path
            log.warning("run_dir '%s' does not exist — idle until created", path)
            return None
        return player._latest_run_dir(self._parent_data_dir())

    def _run_dir(self) -> str:
        """Resolve the actual run directory to load from.

        - If the user set a specific `run_dir` property, use it verbatim
          (relative paths resolve against <data_root>/<robot>/data for
          convenience — typing just "run_20260701_175044" works).
        - Otherwise auto-pick the latest run subdir (legacy behavior)."""
        spec = (self.get("run_dir") or "").strip()
        if spec:
            if os.path.isabs(spec):
                return spec
            return os.path.join(self._parent_data_dir(), spec)
        return self._parent_data_dir()  # caller resolves latest via _latest_run_dir

    def _reload(self):
        if self._loading:
            return  # a load is already in flight
        latest = self._resolve_run()
        if latest is None:
            log.warning("no run directory found for %s — idle until data appears",
                        self.get("robot"))
            return
        log.info("LocalReplay loading from %s", latest)
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
                _, R = data_utils.load_gravity(latest)
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
        """Stream mode — SINGLE-FILE polling.

        The robot-side recorder (go2_record.py) operates in overwrite mode:
          - cloud_registered/latest.npy  ← always the newest frame (overwritten)
          - Odometry/odom_stream.jsonl   ← always the latest odom line (truncated)

        This function checks latest.npy's mtime each tick (one os.stat call).
        When it changes, we load the new frame, downsample it, and merge into
        the display accumulator. Old frames are NOT kept on disk — no listdir,
        no sort, no file-count tracking. O(N_new) per tick, constant regardless
        of how long the robot has been recording.

        The accumulator is periodically re-downsampled (voxel grid) so duplicate
        points from overlapping scans merge — bounded memory, no visual loss."""
        # ---- resolve run dir (throttled every ~2s) ----
        self._retry_t += dt
        if self._loaded_root is None or self._retry_t >= 2.0:
            self._retry_t = 0.0
            latest = self._resolve_run()
            if latest is None:
                return None
            if latest != self._loaded_root:
                self._loaded_root = latest
                self._stream_accum_pts = np.empty((0, 3), dtype=np.float32)
                self._stream_accum_cols = np.empty((0, 3), dtype=np.float32)
                self._odom = []
                self._n = 0
                self._stream_dedup_t = 0.0
                # ONLINE-MODE SEMANTICS: initialize last_mtime to the CURRENT
                # mtime of latest.npy so we DON'T load the stale file already
                # on disk. Only frames written AFTER we start watching get
                # loaded. Without this, _stream_last_mtime=0.0 makes every old
                # file look "new" → history leaks into the live stream.
                _init_npy = os.path.join(latest, "cloud_registered", "latest.npy")
                try:
                    self._stream_last_mtime = os.path.getmtime(_init_npy)
                    self._stream_last_size = os.path.getsize(_init_npy)
                except OSError:
                    self._stream_last_mtime = 0.0
                    self._stream_last_size = 0
                self._cached_pub_pts = None
                self._cached_pub_cols = None
                # Gravity correction loaded once per run.
                try:
                    _, self._stream_R = data_utils.load_gravity(latest)
                except Exception:
                    self._stream_R = np.eye(3)
                log.info("stream: watching %s (skipping existing file)", latest)
        if self._loaded_root is None:
            return None
        # Ensure accumulators exist (may be None if a batch _reload raced).
        if self._stream_accum_pts is None:
            self._stream_accum_pts = np.empty((0, 3), dtype=np.float32)
            self._stream_accum_cols = np.empty((0, 3), dtype=np.float32)
            self._odom = []
            self._n = 0
            self._stream_last_mtime = 0.0
        rid = self.get("robot_id", "robot_a")
        # ---- check latest.npy mtime (ONE os.stat — no listdir/glob) ----
        latest_npy = os.path.join(self._loaded_root, "cloud_registered", "latest.npy")
        try:
            mtime = os.path.getmtime(latest_npy)
            msize = os.path.getsize(latest_npy)
        except OSError:
            # File doesn't exist yet — pipeline may not have produced data.
            new_odom = self._read_latest_odom()
            if new_odom:
                self._odom = new_odom
            self._publish_stream_cached(rid)
            return None
        if not hasattr(self, '_stream_last_mtime'):
            self._stream_last_mtime = 0.0
            self._stream_last_size = 0
        # Detect new frame: mtime changed OR size changed (SCP may overwrite
        # within the same mtime second on Windows NTFS — 1s resolution).
        is_new = (mtime != self._stream_last_mtime) or (msize != self._stream_last_size)
        # Diagnostic: log every 2s what we see
        self._stream_diag_t = getattr(self, '_stream_diag_t', 0.0) + dt
        if self._stream_diag_t >= 2.0:
            self._stream_diag_t = 0.0
            log.info("stream %s: mtime=%.1f last=%.1f size=%d last_size=%d n=%d is_new=%s",
                     rid, mtime, self._stream_last_mtime, msize, self._stream_last_size,
                     self._n, is_new)
        if not is_new:
            # Same file as last tick — just refresh odom.
            new_odom = self._read_latest_odom()
            if new_odom:
                self._odom = new_odom
            self._publish_stream_cached(rid)
            return None
        # ---- NEW frame detected: load, downsample, merge into accumulator ----
        import time as _stime
        _t0 = _stime.monotonic()
        try:
            arr = player._try_load(latest_npy)
        except Exception:
            arr = None
        if arr is None:
            # File may be partially written (SCP in progress) — retry next tick.
            # Update last_mtime so we don't keep hammering the same half-file.
            self._stream_last_mtime = mtime
            self._stream_last_size = msize
            self._publish_stream_cached(rid)
            return None
        self._stream_last_mtime = mtime
        self._stream_last_size = msize
        voxel = float(self.get("voxel_size", 0.1))
        R = self._stream_R if self._stream_R is not None else np.eye(3)
        gf = (R @ arr.T).T  # gravity correction
        # Downsample this single frame, then merge into accumulator.
        if len(gf) > 1000:
            gf_ds = data_utils.voxel_downsample(gf, voxel)
        else:
            gf_ds = gf.astype(np.float32)
        self._stream_accum_pts = np.vstack([self._stream_accum_pts, gf_ds])
        self._n += 1
        self._cursor = float(self._n)
        # Read fresh odom.
        new_odom = self._read_latest_odom()
        if new_odom:
            self._odom = new_odom
        # Re-downsample accumulator when it grows large (bounds memory +
        # removes duplicate points from overlapping scans). Throttled ~0.5s.
        self._stream_dedup_t += dt
        if self._stream_dedup_t >= 0.5:
            self._stream_dedup_t = 0.0
            pts = self._stream_accum_pts
            # Aggressively downsample to keep the published cloud small —
            # large clouds (>100K pts = >2MB WS frame) slow the frontend
            # renderer and clog the tick loop. 50K is plenty for display.
            if len(pts) > 50000:
                pts = data_utils.voxel_downsample(pts, voxel)
                self._stream_accum_pts = pts
            cols = data_utils.height_color_blue_red(pts).astype(np.float32)
            self._stream_accum_pts, self._stream_accum_cols = cap_accum(pts, cols)
        else:
            pts = self._stream_accum_pts
            cols = data_utils.height_color_blue_red(pts).astype(np.float32)
            self._stream_accum_cols = cols
        _ms = (_stime.monotonic() - _t0) * 1000
        if _ms > 30:
            log.warning("stream frame %d: %d pts, took %.0fms", self._n, len(pts), _ms)
        self.ctx.data.publish(rid, {
            "robot_id": rid,
            "frame_idx": self._n,
            "max_frame": self._n,
            "positions": self._stream_accum_pts,
            "colors": self._stream_accum_cols,
            "odom": self._odom[-1] if self._odom else None,
            "all_odom": self._odom,
        })
        self._last_pushed = self._n
        return None

    def _publish_stream_cached(self, rid):
        """Publish the cached accumulator cloud with fresh odom — used when
        no new frame arrived this tick but we still want to update the pose
        marker and keep the cloud visible."""
        if self._stream_accum_pts is None or len(self._stream_accum_pts) == 0:
            return
        self.ctx.data.publish(rid, {
            "robot_id": rid,
            "frame_idx": self._n,
            "max_frame": self._n,
            "positions": self._stream_accum_pts,
            "colors": self._stream_accum_cols,
            "odom": self._odom[-1] if self._odom else None,
            "all_odom": self._odom,
        })

    def _read_latest_odom(self):
        """Read odom_stream.jsonl. The SCP pusher rewrites this file to contain
        only the latest line, so we just read the whole (tiny) file."""
        if self._loaded_root is None:
            return None
        odom_path = os.path.join(self._loaded_root, "Odometry", "odom_stream.jsonl")
        if not os.path.exists(odom_path):
            return None
        try:
            import json as _json
            out = []
            with open(odom_path, "r", encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if line:
                        out.append(_json.loads(line))
            return out
        except Exception:
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
