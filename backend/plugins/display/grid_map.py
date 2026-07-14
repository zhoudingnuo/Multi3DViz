"""grid_map.py — Display plugin: top-down 2D occupancy grid.

Consumes the same data-bus frames as PointCloud (a robot's accumulated cloud),
runs them through lib.gridmap.GridMap (reused verbatim from ccenter) to build a
2-state occupancy grid (0 free / 100 obstacle), and emits a grid2d SceneUpdate.
The frontend renders it on a <canvas> in its own panel.

Throttle: rebuilding the grid from scratch each tick is wasteful; ccenter
rebuilt it from accum every VIS_INTERVAL frames. We mirror that — only re-emit
when the source frame advanced past the last built frame by >= GRID_REFRESH
frames, and reuse a persistent GridMap (GridMap.update() is incremental).
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
from lib.gridmap import GridMap

log = logging.getLogger("multi3dviz.display.grid_map")

# Re-emit the grid every N source frames (matches ccenter GRID_REFRESH_INTERVAL).
GRID_REFRESH_FRAMES = 20


class GridMapDisplay(DisplayPlugin):
    name = "GridMap"
    category = "display"
    description = "Top-down 2D occupancy grid built from a robot's point cloud."
    default_enabled = True
    multiple = True              # one instance per robot grid

    properties = {
        "robot_id": {
            "type": "robot_ref", "default": "robot_a",
            "label": "Robot", "group": "Source",
        },
        "obs_height_min": {
            "type": "float", "default": 0.1, "min": 0.0, "max": 2.0, "step": 0.05,
            "label": "Obstacle min height (m)", "group": "Thresholds",
        },
        "max_height": {
            "type": "float", "default": 1.5, "min": 0.5, "max": 5.0, "step": 0.1,
            "label": "Obstacle max height (m)", "group": "Thresholds",
        },
    }

    def __init__(self, ctx):
        super().__init__(ctx)
        self._gmap = None
        self._last_built_frame = -1
        self._emitted = False

    def on_property_change(self, key, value):
        # Threshold changes invalidate the grid — rebuild from scratch next tick.
        if key in ("obs_height_min", "max_height"):
            self._gmap = None
            self._last_built_frame = -1

    def update(self, dt: float):
        rid = self.get("robot_id", "robot_a")
        frame = self.ctx.data.latest(rid)
        if frame is None:
            return None
        fidx = frame.get("frame_idx", -1)
        if fidx <= self._last_built_frame:
            return None
        # Only rebuild on a coarse cadence — building the grid from the full
        # accumulated cloud each tick is expensive and the 2D view changes
        # slowly. Reuse a persistent GridMap and feed it incrementally.
        if fidx - max(self._last_built_frame, 0) < GRID_REFRESH_FRAMES and self._emitted:
            return None

        # Apply current thresholds to the GridMap (mirrors ccenter's
        # gridmap.OBS_HEIGHT_MIN / MAX_HEIGHT module globals).
        import lib.gridmap as _gm
        _gm.OBS_HEIGHT_MIN = float(self.get("obs_height_min", 0.1))
        _gm.MAX_HEIGHT = float(self.get("max_height", 1.5))

        pos = frame.get("positions")
        if pos is None or len(pos) == 0:
            return None
        pos = np.asarray(pos, dtype=np.float64)
        # Rebuild from the full accumulated cloud each refresh. GridMap.update
        # is incremental, but since the source gives us the *whole* accumulated
        # set each time, a fresh GridMap is simpler and correct.
        gmap = GridMap()
        gmap.update(pos)
        self._gmap = gmap
        self._last_built_frame = fidx
        self._emitted = True

        obj = SceneObject(
            id=f"{rid}_grid2d",
            kind="grid2d",
            payload={
                "cells": gmap.grid,             # HxW int8
                "origin": list(gmap.origin),    # [x, y] world meters
                "resolution": gmap.res,
            },
            meta={"robot_id": rid, "frame_idx": int(fidx)},
        )
        upd = SceneUpdate()
        upd.update.append(obj)   # frontend treats update-on-missing as add
        return upd
