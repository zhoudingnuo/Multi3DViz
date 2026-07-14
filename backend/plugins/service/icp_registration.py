"""icp_registration.py — Service plugin: align two robots' point clouds via ICP
and publish a merged cloud.

Reuses lib/registration.py:icp_align verbatim. Convention (same as ccenter):
robot A is the merged-map origin (target); robot B is aligned to A (source).
The resulting 4x4 transform T_b_to_a maps B's points into A's frame.

Flow:
  1. Watch the data bus for two robots (source_a, source_b). When both have
     accumulated >= min_frames, run ICP once in a background thread.
  2. on_progress callbacks are forwarded to the UI as 'registration_progress'
     events (fitness/rmse/score per trial, init/done phases).
  3. On success, publish:
       - a 'registration_result' event with T, fitness, rmse
       - a merged cloud (A's accumulated points + T @ B's) as a 'merged_cloud'
         point object, and keep updating it as new frames arrive.

The merge is recomputed each tick after ICP succeeds (cheap: vstack A + T@B),
so the merged view stays live as both robots keep scanning. Heavy ICP only runs
once (or again if the user forces re-registration).
"""
from __future__ import annotations
import os
import sys
import threading
import logging
import numpy as np

_THIS = os.path.dirname(os.path.abspath(__file__))
_BACKEND = os.path.abspath(os.path.join(_THIS, "..", ".."))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)

from core.plugin_base import ServicePlugin, SceneUpdate, SceneObject
from lib.registration import icp_align
from lib.data_utils import transform_points

log = logging.getLogger("multi3dviz.service.icp")


class ICPRegistrationService(ServicePlugin):
    name = "ICPRegistration"
    category = "service"
    description = "Align two robots via ICP and publish a merged point cloud."
    default_enabled = True

    properties = {
        "source_a": {
            "type": "robot_ref", "default": "robot_a",
            "label": "Robot A (target / map origin)", "group": "Robots",
        },
        "source_b": {
            "type": "robot_ref", "default": "robot_b",
            "label": "Robot B (source / aligned to A)", "group": "Robots",
        },
        "min_frames": {
            "type": "int", "default": 30, "min": 5, "max": 500, "step": 5,
            "label": "Min frames before registration", "group": "Trigger",
        },
        "voxel_size": {
            "type": "float", "default": 0.5, "min": 0.1, "max": 2.0, "step": 0.1,
            "label": "ICP voxel size (m)", "group": "Quality",
        },
    }

    def __init__(self, ctx):
        super().__init__(ctx)
        self._T_b_to_a = None            # 4x4 once ICP succeeds
        self._reg_state = "idle"         # idle|running|done|failed
        self._fitness = 0.0
        self._rmse = 0.0
        self._running = False            # guards against concurrent ICP runs
        self._last_merged_frame_a = -1
        self._last_merged_frame_b = -1
        self._progress_cb = None         # set by backend to forward to UI
        self._pending_clear = None       # SceneUpdate to emit on next tick (force_reregister)

    def on_enable(self):
        log.info("ICPRegistration ready")

    # --- backend wires this so progress reaches the WS client ---
    def set_progress_forwarder(self, cb):
        self._progress_cb = cb

    def force_reregister(self):
        """User-requested re-registration. Resets state so the next tick re-runs ICP.
        Also removes the merged cloud + position markers so the pre-registration
        individual clouds are visible again during the re-run."""
        self._T_b_to_a = None
        self._reg_state = "idle"
        self._last_merged_frame_a = -1
        self._last_merged_frame_b = -1
        # Clear the merged cloud + markers so individual clouds show again.
        # The SceneUpdate is stashed and returned on the next update() tick.
        self._pending_clear = SceneUpdate()
        self._pending_clear.remove.extend(["merged_cloud", "robot_positions"])

    # --- per-tick ---
    def update(self, dt: float):
        # If a force_reregister is pending, emit the clear first so the old
        # merged cloud disappears and individual clouds show again.
        if self._pending_clear is not None:
            clear = self._pending_clear
            self._pending_clear = None
            return clear
        rid_a = self.get("source_a", "robot_a")
        rid_b = self.get("source_b", "robot_b")
        fa = self.ctx.data.latest(rid_a)
        fb = self.ctx.data.latest(rid_b)
        # If either robot has no data, nothing to do.
        if fa is None or fb is None:
            return None

        # --- trigger ICP when both have enough data and we haven't run yet ---
        if self._T_b_to_a is None and not self._running:
            min_f = int(self.get("min_frames", 30))
            if fa.get("frame_idx", 0) >= min_f and fb.get("frame_idx", 0) >= min_f:
                self._start_registration(fa, fb)
            return None

        # --- after ICP: keep the merged cloud live ---
        if self._T_b_to_a is not None:
            return self._update_merged(fa, fb)
        return None

    # --- ICP execution (background thread) ---
    def _start_registration(self, fa, fb):
        self._running = True
        self._reg_state = "running"
        pts_a = np.asarray(fa["positions"], dtype=np.float64)
        pts_b = np.asarray(fb["positions"], dtype=np.float64)
        voxel = float(self.get("voxel_size", 0.5))
        log.info("ICP starting: A=%d pts, B=%d pts, voxel=%.2f",
                 len(pts_a), len(pts_b), voxel)
        self._forward({"phase": "start", "src_pts": len(pts_b), "tgt_pts": len(pts_a)})

        def _worker():
            try:
                # icp_align(source=B, target=A) — convention: A is the origin.
                _, (fitness, rmse), is_valid, T = icp_align(
                    pts_b, pts_a, voxel_size=voxel, on_progress=self._forward)
                if is_valid:
                    self._T_b_to_a = T
                    self._fitness = float(fitness)
                    self._rmse = float(rmse)
                    self._reg_state = "done"
                    log.info("ICP done: fitness=%.3f rmse=%.4f", fitness, rmse)
                else:
                    self._reg_state = "failed"
                    log.warning("ICP failed validation (fitness=%.3f rmse=%.4f)",
                                fitness, rmse)
                self._forward({"phase": "done", "ok": is_valid,
                               "fitness": float(fitness), "rmse": float(rmse)})
            except Exception as e:
                self._reg_state = "failed"
                log.exception("ICP crashed: %s", e)
                self._forward({"phase": "done", "ok": False, "error": str(e)})
            finally:
                self._running = False

        threading.Thread(target=_worker, daemon=True, name="icp-worker").start()

    def _forward(self, payload):
        """Push an ICP progress event to the UI (if a forwarder is wired)."""
        if self._progress_cb:
            try:
                self._progress_cb({**payload, "state": self._reg_state})
            except Exception:
                pass

    # --- merged cloud publish (after ICP) ---
    def _update_merged(self, fa, fb) -> SceneUpdate:
        f_a = fa.get("frame_idx", 0)
        f_b = fb.get("frame_idx", 0)
        # Only re-emit when either robot advanced.
        if f_a == self._last_merged_frame_a and f_b == self._last_merged_frame_b:
            return None
        self._last_merged_frame_a = f_a
        self._last_merged_frame_b = f_b
        pts_a = np.asarray(fa["positions"], dtype=np.float32)
        pts_b = np.asarray(fb["positions"], dtype=np.float32)
        if len(pts_a) == 0 and len(pts_b) == 0:
            return None
        # Transform B into A's frame, then concatenate.
        pts_b_aligned = transform_points(pts_b, self._T_b_to_a).astype(np.float32) \
            if len(pts_b) else pts_b
        merged = np.vstack([pts_a, pts_b_aligned]) if len(pts_a) and len(pts_b) \
            else (pts_a if len(pts_a) else pts_b_aligned)
        # Color: A=blue-red height ramp, B=green-cyan height ramp, so the two
        # robots are distinguishable in the merged view. Use lib helpers.
        from lib.data_utils import height_color_blue_red, height_color_cyan_yellow
        col_a = height_color_blue_red(pts_a).astype(np.float32) if len(pts_a) \
            else np.empty((0, 3), np.float32)
        col_b = height_color_cyan_yellow(pts_b_aligned).astype(np.float32) if len(pts_b_aligned) \
            else np.empty((0, 3), np.float32)
        colors = np.vstack([col_a, col_b]) if len(col_a) and len(col_b) \
            else (col_a if len(col_a) else col_b)
        obj = SceneObject(
            id="merged_cloud",
            kind="points",
            payload={"positions": merged, "colors": colors, "point_size": 0.04},
            meta={"frame_a": f_a, "frame_b": f_b, "reg_state": self._reg_state,
                  "fitness": self._fitness, "rmse": self._rmse},
        )
        upd = SceneUpdate()
        upd.update.append(obj)

        # --- HIDE the individual pre-registration clouds ---
        # The PointCloud plugin keeps publishing robot_a_cloud / robot_b_cloud
        # (it doesn't know ICP succeeded). Remove them so only the merged cloud
        # is visible after registration.
        rid_a = self.get("source_a", "robot_a")
        rid_b = self.get("source_b", "robot_b")
        upd.remove.append(f"{rid_a}_cloud")
        upd.remove.append(f"{rid_b}_cloud")

        # --- PUBLISH ROBOT POSITIONS (live odom markers) ---
        # Draw each robot as a small bright dot at its current odom position
        # so the user can see where the robots actually are. Robot A is at its
        # own odom origin frame; Robot B is transformed into A's frame via T.
        pos_markers = []
        marker_colors = []
        marker_size = 0.3  # larger than cloud points so it stands out

        # Robot A position (from its own odom, already in merged frame since
        # A IS the origin).
        odom_a = fa.get("odom")
        if odom_a:
            pos_markers.append([odom_a.get("x", 0), odom_a.get("y", 0), odom_a.get("z", 0)])
            marker_colors.append([1.0, 0.8, 0.0])  # gold for A
        # Robot B position (transform B's odom into A's frame via T_b_to_a).
        odom_b = fb.get("odom")
        if odom_b and self._T_b_to_a is not None:
            bp = np.array([odom_b.get("x", 0), odom_b.get("y", 0), odom_b.get("z", 0), 1.0])
            bp_a = (self._T_b_to_a @ bp)[:3]
            pos_markers.append(bp_a.tolist())
            marker_colors.append([0.0, 1.0, 0.6])  # mint for B
        if pos_markers:
            marker_obj = SceneObject(
                id="robot_positions",
                kind="points",
                payload={"positions": np.array(pos_markers, dtype=np.float32),
                         "colors": np.array(marker_colors, dtype=np.float32),
                         "point_size": marker_size},
                meta={"type": "robot_markers"},
            )
            upd.update.append(marker_obj)

        return upd

    def state_snapshot(self):
        """For the backend to include in periodic status broadcasts."""
        return {
            "state": self._reg_state,
            "fitness": self._fitness,
            "rmse": self._rmse,
            "has_transform": self._T_b_to_a is not None,
            "source_a": self.get("source_a", "robot_a"),
            "source_b": self.get("source_b", "robot_b"),
        }
