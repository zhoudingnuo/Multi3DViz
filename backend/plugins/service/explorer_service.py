"""explorer_service.py — Service plugin: dual-agent frontier-based exploration.

Runs after ICP registration succeeds. Owns:
  - a merged GridMap (rebuilt periodically from the merged cloud)
  - a DualAgentExplorer (reused verbatim from ccenter) for frontier detection
    + target assignment
  - per-robot trajectories (list of (wx,wy) in merged-frame meters)
  - target dispatch to robots via RobotManager SSH (writes a target file the
    robot polls, same protocol as ccenter's remote_flag.update_robot_target)

Each tick (after ICP):
  1. Read both robots' current world position from odometry (robot A's odom is
     already in merged frame; B's is transformed by T_b_to_a).
  2. mark_explored around each robot; append to trajectories.
  3. assign_targets() every N ticks → get fresh frontier targets.
  4. Publish the coverage grid + frontiers + trajectories + targets as scene
     objects (grid2d overlay + lines + boxes) so the UI can render them.
  5. Dispatch targets to robots (best-effort SSH write).

Visualization objects (all keyed so the frontend updates them in place):
  - 'explorer_coverage'  : grid2d, cells = explored mask (rendered as green tint)
  - 'explorer_frontiers' : grid2d, cells = frontier mask (rendered as yellow)
  - 'traj_a' / 'traj_b'  : line objects (orange / magenta)
  - 'target_a' / 'target_b': box markers (orange / magenta)
"""
from __future__ import annotations
import os
import sys
import time
import threading
import logging
import numpy as np

_THIS = os.path.dirname(os.path.abspath(__file__))
_BACKEND = os.path.abspath(os.path.join(_THIS, "..", ".."))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)

from core.plugin_base import ServicePlugin, SceneUpdate, SceneObject
from lib.gridmap import GridMap
from lib.explorer import DualAgentExplorer
from lib.data_utils import transform_points, quat_to_mat

log = logging.getLogger("multi3dviz.service.explorer")

ASSIGN_INTERVAL = 1.0      # re-run frontier assignment every N seconds
GRID_REBUILD_INTERVAL = 0.5
TARGET_REACHED_M = 0.3
DISPATCH_COOLDOWN = 2.0    # min seconds between SSH dispatches per robot


class ExplorerService(ServicePlugin):
    name = "DualAgentExplorer"
    category = "service"
    description = "Frontier-based dual-agent exploration + target dispatch (post-ICP)."
    default_enabled = True

    properties = {
        "source_a": {"type": "robot_ref", "default": "robot_a",
                     "label": "Robot A", "group": "Robots"},
        "source_b": {"type": "robot_ref", "default": "robot_b",
                     "label": "Robot B", "group": "Robots"},
        "dispatch_targets": {"type": "bool", "default": True,
                             "label": "SSH-dispatch targets to robots", "group": "Behavior"},
        "auto_explore": {"type": "bool", "default": True,
                         "label": "Auto explore (toggle: on=auto dispatch, off=manual confirm)",
                         "group": "Behavior"},
        "target_path_a": {"type": "string",
                          "default": "/home/unitree/sda2/online/ccenter_target_a.txt",
                          "label": "Robot A target file (remote)", "group": "Dispatch"},
        "target_path_b": {"type": "string",
                          "default": "/home/orin-001/ccenter_target_b.txt",
                          "label": "Robot B target file (remote)", "group": "Dispatch"},
    }

    def __init__(self, ctx):
        super().__init__(ctx)
        self._explorer = None
        self._gmap = None
        self._batch_marked = False  # batch mark_explored over all odom (once)
        self._T_b_to_a = None
        self._traj_a = []   # list of (wx, wy) merged-frame
        self._traj_b = []
        self._assign_t = 0.0
        self._grid_t = 0.0
        self._last_dispatch = [0.0, 0.0]
        self._last_target = [None, None]

    def on_enable(self):
        log.info("DualAgentExplorer ready (waits for ICP)")

    def set_manual_target(self, agent_idx, world_xy):
        """User clicked a point to send a robot there manually. Overrides the
        explorer's auto-assigned target for that agent until the next
        assign_targets() run (which re-evaluates frontiers). Returns True if
        set. agent_idx: 0 (A) or 1 (B). world_xy: (wx, wy) merged frame."""
        if self._explorer is None or self._gmap is None:
            return False
        # Snap the clicked world point into the nearest free grid cell.
        i, j = self._explorer.world_to_grid(world_xy[0], world_xy[1])
        gy, gx = j, i
        H, W = self._gmap.grid.shape
        if not (0 <= gy < H and 0 <= gx < W):
            return False
        self._explorer.targets[agent_idx] = (gy, gx)
        # Force a dispatch on the next tick by clearing the cooldown latch.
        self._last_target[agent_idx] = None
        return True

    # --- main tick ---
    def update(self, dt: float):
        # Gate: only run once ICP has produced a transform. We peek at the
        # ICPRegistration plugin's state (loose coupling — both are services).
        icp = self.ctx  # ctx.robots etc; we reach the registry via a hook
        # The backend sets ctx.icp_ref so we can read T_b_to_a without import cycles.
        T = self._get_icp_transform()
        if T is None:
            return None
        self._T_b_to_a = T

        fa = self.ctx.data.latest(self.get("source_a", "robot_a"))
        fb = self.ctx.data.latest(self.get("source_b", "robot_b"))
        if fa is None or fb is None:
            return None

        # Periodically rebuild the merged gridmap from the merged cloud.
        self._grid_t += dt
        if self._grid_t >= GRID_REBUILD_INTERVAL:
            self._grid_t = 0.0
            self._rebuild_grid(fa, fb)

        if self._explorer is None or self._gmap is None:
            return None

        # On first entry after grid+explorer creation, do a BATCH mark_explored
        # over ALL recorded odom positions — not just the latest. ccenter does
        # this naturally because its playback cursor advances frame by frame
        # and mark_explored runs each tick. But Multi3DViz's instant_load mode
        # publishes odom[-1] every tick (the final pose), so without this batch
        # pass only a single disk at the endpoint would be marked explored.
        if not self._batch_marked:
            self._batch_mark_explored(fa, fb, T)
            self._batch_marked = True

        # Current robot positions in merged frame: (wx, wy, yaw).
        pos_a = self._robot_pos(fa, np.eye(4))
        pos_b = self._robot_pos(fb, T)
        if pos_a is not None:
            wx, wy, yaw = pos_a
            self._explorer.mark_explored((wx, wy), yaw=yaw)
            self._append_traj(self._traj_a, (wx, wy))
        if pos_b is not None:
            wx, wy, yaw = pos_b
            self._explorer.mark_explored((wx, wy), yaw=yaw)
            self._append_traj(self._traj_b, (wx, wy))

        # Re-run frontier assignment on a coarse cadence.
        self._assign_t += dt
        if self._assign_t >= ASSIGN_INTERVAL:
            self._assign_t = 0.0
            if pos_a is not None and pos_b is not None:
                # assign_targets takes world positions (x,y); yaw is only used
                # by mark_explored for the sensor model.
                self._explorer.assign_targets((pos_a[0], pos_a[1]),
                                              (pos_b[0], pos_b[1]))
                self._maybe_dispatch((pos_a[0], pos_a[1]), (pos_b[0], pos_b[1]))

        return self._publish_scene()

    # --- helpers ---
    def _get_icp_transform(self):
        """Read the ICP service's T_b_to_a if it has succeeded. The backend
        wires ctx.icp_ref to the ICPRegistration instance."""
        icp = getattr(self.ctx, "icp_ref", None)
        if icp is not None and getattr(icp, "_T_b_to_a", None) is not None:
            return icp._T_b_to_a
        return None

    def _rebuild_grid(self, fa, fb):
        pts_a = np.asarray(fa.get("positions", []), dtype=np.float64)
        pts_b = np.asarray(fb.get("positions", []), dtype=np.float64)
        if len(pts_b):
            pts_b = transform_points(pts_b, self._T_b_to_a)
        merged = np.vstack([pts_a, pts_b]) if len(pts_a) and len(pts_b) \
            else (pts_a if len(pts_a) else pts_b)
        if len(merged) == 0:
            return
        # GridMap is now FIXED-size (allocates 60m×60m once, origin locked at
        # camera_init). So we create the grid + explorer ONCE and keep feeding
        # the same objects — update() is idempotent on a fixed grid and the
        # explored mask persists naturally (no copy needed).
        if self._gmap is None:
            self._gmap = GridMap()
            self._explorer = DualAgentExplorer(self._gmap)
        self._gmap.update(merged)

    def _batch_mark_explored(self, fa, fb, T):
        """Walk ALL odom positions for both robots and mark_explored at each —
        reproducing what ccenter does naturally as its playback cursor sweeps
        frame by frame. Without this, instant_load mode publishes only odom[-1]
        and mark_explored sees just the final pose → a single explored disk
        instead of the full swept trail. This runs ONCE after grid+explorer
        creation, then the per-tick mark_explored handles live updates."""
        log.info("batch mark_explored: sweeping all odom positions...")
        for frame, T_to_merged, label in [(fa, np.eye(4), "A"), (fb, T, "B")]:
            all_odom = frame.get("all_odom") if frame else None
            if not all_odom:
                continue
            n = 0
            for odom in all_odom:
                pos = self._robot_pos({"odom": odom}, T_to_merged)
                if pos is not None:
                    wx, wy, yaw = pos
                    self._explorer.mark_explored((wx, wy), yaw=yaw)
                    n += 1
            log.info("  robot %s: marked %d odom positions", label, n)

    @staticmethod
    def _robot_pos(frame, T_to_merged):
        """World position + heading of a robot, transformed into the merged
        frame. Returns (wx, wy, yaw) or None. yaw is the heading in radians
        (0 = +x) in merged-frame world coords — needed by the direction-aware
        mark_explored (LiDAR sees farther forward than behind)."""
        odom = frame.get("odom")
        if not odom:
            return None
        try:
            Tw = quat_to_mat(odom["qx"], odom["qy"], odom["qz"], odom["qw"])
            Tw[0, 3] = odom["x"]; Tw[1, 3] = odom["y"]; Tw[2, 3] = odom["z"]
            # Transform the full pose (rotation + translation) into merged frame.
            Tm = T_to_merged @ Tw
            origin = Tm[:3, 3]
            # Heading = atan2 of the transformed forward axis (column 0 = +x dir).
            yaw = float(np.arctan2(Tm[1, 0], Tm[0, 0]))
            return (float(origin[0]), float(origin[1]), yaw)
        except Exception:
            return None

    @staticmethod
    def _append_traj(traj, pos):
        if not traj or (abs(traj[-1][0]-pos[0]) + abs(traj[-1][1]-pos[1])) > 0.05:
            traj.append(pos)
            if len(traj) > 5000:
                del traj[:1000]   # cap memory

    def confirm_targets(self):
        """Manual confirm: dispatch the current targets to robots immediately,
        bypassing the auto_explore toggle. Called when the user presses Enter
        in manual (non-auto) mode. Returns True if any target was dispatched."""
        ex = self._explorer
        if ex is None or self._gmap is None:
            return False
        fa = self.ctx.data.latest(self.get("source_a", "robot_a"))
        fb = self.ctx.data.latest(self.get("source_b", "robot_b"))
        if fa is None or fb is None:
            return False
        pos_a = self._robot_pos(fa, np.eye(4))
        pos_b = self._robot_pos(fb, self._T_b_to_a or np.eye(4))
        if pos_a is None or pos_b is None:
            return False
        # Force dispatch by clearing cooldown latches.
        self._last_dispatch = [0.0, 0.0]
        self._dispatch_now(pos_a, pos_b)
        return True

    def _dispatch_now(self, pos_a, pos_b):
        """Internal: dispatch targets regardless of auto_explore state."""
        saved = self.get("auto_explore", True)
        self._prop_values["auto_explore"] = True  # temporarily override
        try:
            self._maybe_dispatch(pos_a, pos_b)
        finally:
            self._prop_values["auto_explore"] = saved

    def _maybe_dispatch(self, pos_a, pos_b):
        """SSH-write target files to both robots (best-effort, rate-limited).
        Only dispatches when auto_explore is ON. When OFF, targets are computed
        + displayed but NOT dispatched — the user confirms via Enter key
        (confirm_targets action) to send them."""
        if not self.get("dispatch_targets", True):
            return
        if not self.get("auto_explore", True):
            return  # manual mode — wait for confirm_targets
        if not self.ctx.robots:
            return
        now = time.monotonic()
        for i, (pos, rid, tpath) in enumerate([
            (pos_a, self.get("source_a", "robot_a"), self.get("target_path_a")),
            (pos_b, self.get("source_b", "robot_b"), self.get("target_path_b")),
        ]):
            tgt = self._explorer.targets[i]
            if tgt is None:
                continue
            if tgt == self._last_target[i] and now - self._last_dispatch[i] < DISPATCH_COOLDOWN:
                continue
            conn = self.ctx.robots.get(rid)
            if conn is None:
                continue
            gy, gx = tgt
            wx, wy = self._explorer.grid_to_world(gy, gx)
            # Same field schema as ccenter's update_robot_target.
            content = (f"mode: explore\nframe: {0}\ntimestamp: {time.time()}\n"
                       f"global_x: {wx}\nglobal_y: {wy}\nlocal_x: {wx}\nlocal_y: {wy}\n")
            ok = conn.write_file(tpath, content)
            if ok:
                self._last_target[i] = tgt
                self._last_dispatch[i] = now

    # --- scene publish ---
    def _publish_scene(self) -> SceneUpdate:
        upd = SceneUpdate()
        ex = self._explorer
        gmap = self._gmap
        if ex is None or gmap is None:
            return upd
        # Coverage + frontier as int8 grids the frontend tints.
        # Encode: 0=free-unexplored, 1=explored, 2=frontier, 100=obstacle.
        cov = np.zeros_like(gmap.grid, dtype=np.int8)
        cov[gmap.grid == 100] = 100
        cov[ex.explored] = 1
        if ex.frontier_cells is not None and ex.frontier_cells.shape == cov.shape:
            # Guard: only mark frontiers if we have meaningful explored area
            # AND the frontier count is reasonable (< 30% of free cells).
            # Without this guard, early explorer runs with nearly-empty explored
            # masks produce frontier_cells covering the entire grid → all yellow.
            n_frontier = int(ex.frontier_cells.sum())
            n_free = int((gmap.grid == 0).sum())
            if n_free > 0 and n_frontier < n_free * 0.3:
                cov[ex.frontier_cells] = 2
        upd.update.append(SceneObject(
            id="explorer_overlay",
            kind="grid2d",
            payload={"cells": cov, "origin": list(gmap.origin), "resolution": gmap.res},
            meta={"type": "explorer", "robots": [self.get("source_a"), self.get("source_b")]},
        ))
        # Trajectories as lines.
        for traj, oid, color in [
            (self._traj_a, "traj_a", [1.0, 0.4, 0.0]),
            (self._traj_b, "traj_b", [1.0, 0.0, 0.8]),
        ]:
            if len(traj) >= 2:
                pts = np.array([[x, y, 0.0] for x, y in traj], dtype=np.float32)
                upd.update.append(SceneObject(
                    id=oid, kind="line",
                    payload={"positions": pts, "color": color, "width": 2.0},
                    meta={}))
        # Target markers as boxes.
        for i, (oid, color) in enumerate([("target_a", [1.0, 0.4, 0.0]),
                                          ("target_b", [1.0, 0.0, 0.8])]):
            tgt = ex.targets[i]
            if tgt is not None:
                gy, gx = tgt
                wx, wy = ex.grid_to_world(gy, gx)
                pose = [[1,0,0,wx],[0,1,0,wy],[0,0,1,0.05],[0,0,0,1]]
                upd.update.append(SceneObject(
                    id=oid, kind="box",
                    payload={"size": [0.3, 0.3, 0.3], "color": color, "pose": pose},
                    meta={}))
            else:
                upd.remove.append(oid)
        return upd
