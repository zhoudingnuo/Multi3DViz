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
        self._single_rid = None  # set in update(): which robot has data in single mode

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
        # Explorer grid/frontier logic is DISABLED (causes tick freeze).
        # Only publish robot pose arrows so the user can see where robots are.
        import math as _m
        sa = self.get("source_a", "robot_a")
        sb = self.get("source_b", "robot_b")
        fa = self.ctx.data.latest(sa)
        fb = self.ctx.data.latest(sb)
        upd = SceneUpdate()
        for rid, frame, color in [(sa, fa, [1.0, 0.6, 0.0]),
                                   (sb, fb, [0.8, 0.0, 1.0])]:
            if frame is None:
                continue
            odom = frame.get("odom")
            if not odom:
                continue
            x = float(odom.get("x", 0)); y = float(odom.get("y", 0))
            qx = float(odom.get("qx", 0)); qy = float(odom.get("qy", 0))
            qz = float(odom.get("qz", 0)); qw = float(odom.get("qw", 1))
            yaw = _m.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            pose = np.eye(4).tolist()
            pose[0][3] = x; pose[1][3] = y; pose[2][3] = 0.0
            c, s = _m.cos(yaw), _m.sin(yaw)
            pose[0][0] = c; pose[0][1] = -s
            pose[1][0] = s; pose[1][1] = c
            upd.update.append(SceneObject(
                id=f"{rid}_pose", kind="arrow",
                payload={"color": color, "pose": pose, "length": 0.5},
                meta={"type": "robot_pose"},
            ))
        return upd if upd.update else None

    def _bg_compute(self, dt: float):
        """Background thread: run the full explorer update cycle. Stores the
        SceneUpdate result in _bg_result for update() to pick up next tick.
        Any crash is caught — explorer must NEVER bring down the backend."""
        try:
            self._bg_result = self._update_inner(dt)
        except Exception:
            log.exception("explorer bg compute crashed (tick loop protected)")
            self._bg_result = None
        finally:
            self._bg_running = False

    def _update_inner(self, dt: float):
        # ICP transform is optional — single-robot mode doesn't need it.
        # We read it if available (dual-robot, post-ICP); otherwise T=None and
        # we operate on a single source alone (single-robot exploration).
        T = self._get_icp_transform()
        self._T_b_to_a = T

        sa = self.get("source_a", "robot_a")
        sb = self.get("source_b", "robot_b")
        fa = self.ctx.data.latest(sa)
        fb = self.ctx.data.latest(sb) if T is not None else None
        # Single-robot fallback: if source_a has no data (e.g. only robot_b is
        # online and recording), use source_b as the single source. This lets
        # exploration work with whichever robot actually has data, without the
        # user having to swap source_a/source_b in the config.
        self._single_rid = None  # which robot the single-source mode is using
        if fa is None and fb is None:
            # Neither source has data yet — try any robot in the data bus.
            for rid in self.ctx.data.robots():
                fa = self.ctx.data.latest(rid)
                if fa is not None:
                    self._single_rid = rid
                    break
            if fa is None:
                return None
        elif fa is None:
            # source_b has data but source_a doesn't — swap: use B as the
            # single source (identity transform).
            fa = fb
            fb = None
            T = None
            self._single_rid = sb
            self._T_b_to_a = None

        # Periodically rebuild the gridmap. Single-robot: just A's cloud.
        # Dual-robot (post-ICP): merged A + T@B.
        self._grid_t += dt
        if self._grid_t >= GRID_REBUILD_INTERVAL:
            self._grid_t = 0.0
            self._rebuild_grid(fa, fb)

        if self._explorer is None or self._gmap is None:
            return None

        # On first entry after grid+explorer creation, do a BATCH mark_explored
        # over ALL recorded odom positions.
        if not self._batch_marked:
            self._batch_mark_explored(fa, fb, T)
            self._batch_marked = True

        # Current robot positions in merged frame: (wx, wy, yaw).
        pos_a = self._robot_pos(fa, np.eye(4))
        pos_b = self._robot_pos(fb, T) if (fb is not None and T is not None) else None
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
            if pos_a is not None:
                # Single-robot: assign_targets with A's position for agent 0,
                # and B's position if available (agent 1). The explorer handles
                # a missing second agent gracefully (pos_b None → agent 1 idle).
                pb = (pos_b[0], pos_b[1]) if pos_b is not None else None
                self._explorer.assign_targets((pos_a[0], pos_a[1]), pb)
                if pos_b is not None:
                    self._maybe_dispatch((pos_a[0], pos_a[1]), (pos_b[0], pos_b[1]))
                else:
                    # Single-robot dispatch: only agent 0.
                    self._maybe_dispatch_single((pos_a[0], pos_a[1]))

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
        pts_b = np.asarray(fb.get("positions", []), dtype=np.float64) if fb is not None else np.empty((0,3))
        if len(pts_b) and self._T_b_to_a is not None:
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
        if fa is None:
            return False
        pos_a = self._robot_pos(fa, np.eye(4))
        if pos_a is None:
            return False
        # source_b is optional (single-robot mode).
        fb = self.ctx.data.latest(self.get("source_b", "robot_b")) \
            if self._T_b_to_a is not None else None
        pos_b = self._robot_pos(fb, self._T_b_to_a) if (fb and self._T_b_to_a is not None) else None
        # Force dispatch by clearing cooldown latches.
        self._last_dispatch = [0.0, 0.0]
        if pos_b is not None:
            self._dispatch_now(pos_a, pos_b)
        else:
            # Single-robot: force-dispatch agent 0 only.
            self._dispatch_now_single(pos_a)
        return True

    def _dispatch_now(self, pos_a, pos_b):
        """Internal: dispatch targets regardless of auto_explore state."""
        saved = self.get("auto_explore", True)
        self._prop_values["auto_explore"] = True  # temporarily override
        try:
            self._maybe_dispatch(pos_a, pos_b)
        finally:
            self._prop_values["auto_explore"] = saved

    def _dispatch_now_single(self, pos_a):
        """Force-dispatch agent 0 only (single-robot mode)."""
        saved = self.get("auto_explore", True)
        self._prop_values["auto_explore"] = True
        try:
            self._maybe_dispatch_single(pos_a)
        finally:
            self._prop_values["auto_explore"] = saved

    def _maybe_dispatch_single(self, pos_a):
        """SSH-write target file to the single active robot (single-robot mode).
        Uses self._single_rid (set in update) to pick the right robot + target
        path — may be robot_a or robot_b depending on which has data."""
        if not self.get("dispatch_targets", True):
            return
        if not self.get("auto_explore", True):
            return  # manual mode — wait for confirm_targets (Enter)
        if not self.ctx.robots:
            return
        tgt = self._explorer.targets[0]
        if tgt is None:
            return
        now = time.monotonic()
        if tgt == self._last_target[0] and now - self._last_dispatch[0] < DISPATCH_COOLDOWN:
            return
        # Pick the robot + target path for whichever robot is the single source.
        rid = self._single_rid or self.get("source_a", "robot_a")
        if rid == self.get("source_b", "robot_b"):
            tpath = self.get("target_path_b")
        else:
            tpath = self.get("target_path_a")
        conn = self.ctx.robots.get(rid)
        if conn is None:
            return
        gy, gx = tgt
        wx, wy = self._explorer.grid_to_world(gy, gx)
        content = (f"mode: explore\nframe: {0}\ntimestamp: {time.time()}\n"
                   f"global_x: {wx}\nglobal_y: {wy}\nlocal_x: {wx}\nlocal_y: {wy}\n")
        if conn.write_file(tpath, content):
            self._last_target[0] = tgt
            self._last_dispatch[0] = now
            self._last_dispatch[0] = now

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
        # CROP to the explored bounding box — the full grid is 1200x1200 (1.4MB)
        # but most of it is empty unexplored space. Sending the whole thing every
        # explorer cycle (2s) floods the WS with 2.8MB per update → bridge.serialize
        # blocks the event loop → tick loop freezes during takeover.
        # We crop to the bounding box of (explored | obstacles | frontiers) with
        # a small margin, and adjust origin accordingly. Typical crop: 200x200.
        active = ex.explored if ex.explored is not None else np.zeros_like(gmap.grid, dtype=bool)
        active = active | (gmap.grid == 100)
        if ex.frontier_cells is not None:
            active = active | ex.frontier_cells
        ys, xs = np.where(active)
        if len(ys) == 0:
            return upd  # nothing to show yet
        margin = 20
        y0, y1 = max(0, ys.min() - margin), min(gmap.grid.shape[0], ys.max() + margin + 1)
        x0, x1 = max(0, xs.min() - margin), min(gmap.grid.shape[1], xs.max() + margin + 1)
        cells = gmap.grid[y0:y1, x0:x1].copy()
        crop_origin = [gmap.origin[0] + x0 * gmap.res,
                       gmap.origin[1] + y0 * gmap.res]
        # Merged base occupancy grid (cropped).
        upd.update.append(SceneObject(
            id="merged_grid2d",
            kind="grid2d",
            payload={"cells": cells,
                     "origin": crop_origin, "resolution": gmap.res},
            meta={"type": "merged"},
        ))
        # Coverage + frontier (cropped).
        # Encode: 0=free-unexplored, 1=explored, 2=frontier, 100=obstacle.
        cov = np.zeros_like(cells, dtype=np.int8)
        cov[cells == 100] = 100
        explored_crop = ex.explored[y0:y1, x0:x1] if ex.explored is not None else None
        if explored_crop is not None:
            cov[explored_crop] = 1
        if ex.frontier_cells is not None and ex.frontier_cells.shape == gmap.grid.shape:
            frontier_crop = ex.frontier_cells[y0:y1, x0:x1]
            n_frontier = int(frontier_crop.sum())
            n_free = int((cells == 0).sum())
            if n_free > 0 and n_frontier < n_free * 0.3:
                cov[frontier_crop] = 2
        upd.update.append(SceneObject(
            id="explorer_overlay",
            kind="grid2d",
            payload={"cells": cov, "origin": crop_origin, "resolution": gmap.res},
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
