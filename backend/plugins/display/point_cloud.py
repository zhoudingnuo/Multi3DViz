"""point_cloud.py — Display plugin: render a robot's accumulated point cloud.

Reads the latest frame from the data bus (published by a DataSource) and emits
a SceneUpdate that UPDATES a single 'points' object keyed by robot_id. Because
the source publishes the FULL accumulated cloud each time, and the frontend
replaces (not appends) the object's geometry on 'update', we send the whole
accumulated array — but voxel-downsampled already at the source, so it stays
bounded (~10^5 pts, not 10^6).

Color mode options (mirrors ccenter's height ramps):
    'height'   — blue(low)→red(high), via lib.data_utils.height_color_blue_red
    'solid'    — single robot color
    'robot'    — per-robot distinct color

This is the canonical Phase 1 plugin: proves DataSource→Display→SceneBridge→
WS→Three.js end-to-end. No incremental-delta optimization yet — the source
already keeps the cloud bounded via voxel downsample.
"""
from __future__ import annotations
import os
import sys
import logging
import numpy as np

_THIS = os.path.dirname(os.path.abspath(__file__))
_BACKEND = os.path.abspath(os.path.join(_THIS, "..", ".."))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)

from core.plugin_base import DisplayPlugin, SceneUpdate, SceneObject
from lib import data_utils

log = logging.getLogger("multi3dviz.display.point_cloud")

# Per-robot display colors when color_mode == 'robot'.
ROBOT_COLORS = {
    "robot_a": [1.0, 0.4, 0.2],   # orange (unitree accent in ccenter)
    "robot_b": [0.2, 0.85, 0.4],  # green (agibot accent)
}


class PointCloudDisplay(DisplayPlugin):
    name = "PointCloud"
    category = "display"
    description = "Render a robot's accumulated point cloud (height/solid/robot color)."
    default_enabled = True
    multiple = True              # one instance per robot cloud
    # Default: one display per robot so both clouds render on first run.
    # Both use 'height' color mode (blue→red height ramp) so the user can
    # see structure immediately. robot_b can be switched to 'robot' (solid
    # green) from the UI if visual distinction is preferred.
    default_instances = [
        {"robot_id": "robot_a", "color_mode": "height"},
        {"robot_id": "robot_b", "color_mode": "height"},
    ]

    properties = {
        "robot_id": {
            "type": "robot_ref",
            "default": "robot_a",
            "label": "Robot",
            "group": "Source",
        },
        "color_mode": {
            "type": "select",
            "options": ["height", "solid", "robot"],
            "default": "height",
            "label": "Color mode",
            "group": "Appearance",
        },
        "solid_color": {
            "type": "string",
            "default": "1.0,1.0,1.0",
            "label": "Solid color (r,g,b 0-1)",
            "group": "Appearance",
        },
        "point_size": {
            "type": "float",
            "default": 0.04,
            "min": 0.001, "max": 0.5, "step": 0.005,
            "label": "Point size (m)",
            "group": "Appearance",
        },
    }

    def __init__(self, ctx):
        super().__init__(ctx)
        self._last_frame_idx = -1   # skip re-push when source frame unchanged

    def update(self, dt: float):
        rid = self.get("robot_id", "robot_a")
        frame = self.ctx.data.latest(rid)
        if frame is None:
            return None
        # Only emit when the source advanced — avoids resending identical data.
        if frame.get("frame_idx", -1) == self._last_frame_idx:
            return None
        self._last_frame_idx = frame.get("frame_idx", -1)

        pos = frame.get("positions")
        if pos is None or len(pos) == 0:
            # Empty so far — send a remove so the frontend clears stale geo.
            return SceneUpdate(remove=[f"{rid}_cloud"])
        pos = np.ascontiguousarray(pos, dtype=np.float32)
        colors = self._compute_colors(pos, rid, frame)
        obj = SceneObject(
            id=f"{rid}_cloud",
            kind="points",
            payload={
                "positions": pos,
                "colors": colors,
                "point_size": float(self.get("point_size", 0.04)),
            },
            meta={"robot_id": rid, "frame_idx": int(frame.get("frame_idx", 0)),
                  "max_frame": int(frame.get("max_frame", 0))},
        )
        # 'add' first time, 'update' thereafter — the frontend treats update
        # on a non-existent id as add, so 'update' alone is sufficient and
        # simpler. We still use add on the very first emission for clarity.
        op = "add" if self._last_frame_idx <= 1 else "update"
        upd = SceneUpdate()
        (upd.add if op == "add" else upd.update).append(obj)
        return upd

    def _compute_colors(self, pos: np.ndarray, rid: str, frame) -> np.ndarray:
        mode = self.get("color_mode", "height")
        if mode == "height":
            # Source may already supply colors from its own height ramp — use
            # those if present and matching length (saves a recompute).
            src_colors = frame.get("colors")
            if src_colors is not None and len(src_colors) == len(pos):
                return np.ascontiguousarray(src_colors, dtype=np.float32)
            return data_utils.height_color_blue_red(pos).astype(np.float32)
        if mode == "robot":
            c = ROBOT_COLORS.get(rid, [1, 1, 1])
            return np.tile(np.array(c, dtype=np.float32), (len(pos), 1))
        # solid
        try:
            c = [float(x) for x in str(self.get("solid_color", "1,1,1"))
                 .split(",")] + [1, 1, 1]
            c = c[:3]
        except Exception:
            c = [1, 1, 1]
        return np.tile(np.array(c, dtype=np.float32), (len(pos), 1))
