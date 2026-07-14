"""semantics.py — Service plugin: UNet semantic segmentation + room detection
on the merged occupancy grid.

Reuses ccenter's sem_infer.SemPredictor (UNet, classes 1-4) and
room_detect.detect_rooms (scipy connected components). Both run on the merged
grid the ExplorerService builds; we reach it via ctx.explorer_ref (loose
coupling, same pattern as ICP).

Output: a 'sem_overlay' grid2d where each cell carries a combined code so the
frontend can tint by semantic class OR by room id:
    0   free, no label
    100 obstacle
    1..4 semantic class (1=wall,2=room,3=corridor,4=furniture)
    10+ room id (10..) when room detection is the active mode

Both modes publish to the same overlay id; a property `mode` selects which.
Throttled: inference is expensive, so we run at most every PREDICT_INTERVAL
seconds (default 5s) and only when the grid changed.
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
from lib.sem_infer import SemPredictor
from lib.room_detect import detect_rooms

log = logging.getLogger("multi3dviz.service.semantics")

PREDICT_INTERVAL = 5.0      # seconds between UNet inference runs


class SemanticsService(ServicePlugin):
    name = "Semantics"
    category = "service"
    description = "UNet semantic segmentation + room detection on the merged grid."
    default_enabled = False   # opt-in (needs the trained model + is CPU-heavy)

    properties = {
        "mode": {"type": "select", "options": ["semantic", "rooms"],
                 "default": "semantic", "label": "Overlay mode", "group": "Mode"},
        "predict_interval": {"type": "float", "default": PREDICT_INTERVAL,
                             "min": 1.0, "max": 60.0, "step": 1.0,
                             "label": "Inference interval (s)", "group": "Timing"},
        "min_room_cells": {"type": "int", "default": 200, "min": 20, "max": 5000, "step": 20,
                           "label": "Min room size (cells)", "group": "Rooms"},
    }

    def __init__(self, ctx):
        super().__init__(ctx)
        self._predictor = None
        self._predicting = False
        self._sem = None          # last semantic label grid
        self._sem_origin = None
        self._rooms = None        # last room labels grid
        self._rooms_list = None
        self._t = 0.0
        self._last_grid_shape = None

    def on_enable(self):
        # Load the UNet lazily on enable (fails soft if no model — service stays
        # up but mode=rooms still works without it).
        try:
            self._predictor = SemPredictor()
            if self._predictor.available:
                log.info("SemPredictor loaded (semantic mode available)")
            else:
                log.warning("SemPredictor unavailable — semantic mode disabled, "
                            "rooms mode still works")
        except Exception as e:
            log.warning("SemPredictor init failed: %s — rooms mode only", e)
            self._predictor = None
        self._force = False  # manual trigger flag (set by WS semantics_trigger)

    def force_predict(self):
        """Manual trigger: next update() tick will run inference immediately
        regardless of the interval throttle."""
        self._force = True

    # --- main tick ---
    def update(self, dt: float):
        ex = getattr(self.ctx, "explorer_ref", None)
        if ex is None or getattr(ex, "_gmap", None) is None:
            return None
        gmap = ex._gmap
        self._t += dt
        shape_changed = gmap.grid.shape != self._last_grid_shape
        if shape_changed:
            self._sem = None
            self._rooms = None
            self._last_grid_shape = gmap.grid.shape
        # Run if: interval elapsed, grid shape changed, or manual force.
        should_run = (self._t >= float(self.get("predict_interval", PREDICT_INTERVAL))
                      or shape_changed or self._force)
        if not should_run:
            return None
        self._t = 0.0
        self._force = False

        mode = self.get("mode", "semantic")
        if mode == "semantic":
            self._run_semantic(gmap)
        else:
            self._run_rooms(gmap)
        return self._publish(gmap)

    # --- inference (background thread; UNet is slow) ---
    def _run_semantic(self, gmap):
        if self._predictor is None or not self._predictor.available:
            return
        if self._predicting:
            return
        grid = gmap.grid.copy()
        origin = gmap.origin.copy()
        self._predicting = True

        def _worker():
            try:
                sem = self._predictor.predict(grid)
                if sem is not None:
                    self._sem = sem
                    self._sem_origin = origin
            except Exception as e:
                log.warning("sem predict failed: %s", e)
            finally:
                self._predicting = False

        threading.Thread(target=_worker, daemon=True, name="sem-worker").start()

    def _run_rooms(self, gmap):
        try:
            rooms, labels = detect_rooms(gmap.grid, gmap.origin, gmap.res,
                                         min_cells=int(self.get("min_room_cells", 200)))
            self._rooms = labels
            self._rooms_list = rooms
        except Exception as e:
            log.warning("room detect failed: %s", e)

    # --- publish overlay ---
    def _publish(self, gmap):
        # Build an overlay grid same shape as gmap, encoding the active mode.
        ov = np.zeros_like(gmap.grid, dtype=np.int8)
        ov[gmap.grid == 100] = 100
        mode = self.get("mode", "semantic")
        if mode == "semantic" and self._sem is not None \
                and self._sem.shape == gmap.grid.shape:
            # sem carries 1..4 on labeled cells; overlay those.
            labeled = self._sem > 0
            ov[labeled] = self._sem[labeled]
        elif mode == "rooms" and self._rooms is not None \
                and self._rooms.shape == gmap.grid.shape:
            # encode room id as 10 + id (so frontend tints distinctly)
            for r in (self._rooms_list or []):
                rid = r["id"]
                ov[self._rooms == rid] = 10 + rid
        else:
            return SceneUpdate()  # nothing to show yet
        obj = SceneObject(
            id="sem_overlay",
            kind="grid2d",
            payload={"cells": ov, "origin": list(gmap.origin), "resolution": gmap.res},
            meta={"type": "semantics", "mode": mode,
                  "n_rooms": len(self._rooms_list or [])},
        )
        upd = SceneUpdate()
        upd.update.append(obj)
        return upd
