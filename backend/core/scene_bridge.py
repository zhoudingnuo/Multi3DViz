"""scene_bridge.py — serialize SceneUpdate into WebSocket frames.

A SceneUpdate is a declarative batch of add/update/remove ops on SceneObjects.
Most objects are small (boxes, lines, labels) and ride along as pure JSON.
Point clouds are large (10^5–10^6 floats) so they're split out into a binary
frame: one JSON "scene_binary" header describing the layout, immediately
followed by one binary WS frame holding the concatenated float32 arrays.

Binary layout (all little-endian float32, tightly packed, in layout order):
    For each layout entry with kind=='points':
        positions: n_points * 3 float32   (x,y,z per point)
        colors:    n_points * 3 float32   (only if has_colors)
    Mesh payloads carry positions+indices+colors similarly (see _layout_for).

The frontend reads the header, then consumes the next binary message by
slicing according to the layouts. This keeps point data off the JSON parser
(>10x smaller + faster than base64-in-JSON).
"""
from __future__ import annotations
import json
import logging
import numpy as np

from core.plugin_base import SceneUpdate, SceneObject

log = logging.getLogger("multi3dviz.scene")

# Max points per binary frame. A single giant cloud is fine, but capping keeps
# memory predictable; larger clouds are split (Phase 1 uses a single frame).
MAX_POINTS_PER_FRAME = 2_000_000


def _to_f32(arr) -> np.ndarray:
    return np.ascontiguousarray(arr, dtype=np.float32)


def _to_u32(arr) -> np.ndarray:
    return np.ascontiguousarray(arr, dtype=np.uint32)


def _identity_pose():
    return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


class SceneBridge:
    """Turns SceneUpdates into WS frame(s). Call serialize(update) to get a
    list of frames to send: each item is either a str (JSON text) or bytes
    (binary). Always returns JSON-frames first, then their matching binary."""

    def serialize(self, update: SceneUpdate) -> list:
        if update is None:
            return []
        json_ops = []          # small ops in one JSON frame
        binary_payloads = []   # (layout_meta, bytes)
        layouts = []

        # --- adds/updates ---
        for obj in list(update.add) + list(update.update):
            action = "add" if obj in update.add else "update"
            if obj.kind == "points":
                meta, blob = self._serialize_points(obj)
                if blob is not None:
                    meta["op"] = action
                    layouts.append(meta)
                    binary_payloads.append(blob)
            elif obj.kind == "mesh":
                meta, blob = self._serialize_mesh(obj)
                if blob is not None:
                    meta["op"] = action
                    layouts.append(meta)
                    binary_payloads.append(blob)
            elif obj.kind == "grid2d":
                # Occupancy grid — int8 cells ride the binary path so it scales
                # to large grids without base64-in-JSON bloat.
                meta, blob = self._serialize_grid2d(obj)
                if blob is not None:
                    meta["op"] = action
                    layouts.append(meta)
                    binary_payloads.append(blob)
            else:
                # box / line / label — small, JSON only.
                json_ops.append(self._small_op(obj, action))

        # --- removes ---
        for oid in update.remove:
            json_ops.append({"op": "remove", "id": oid})

        frames = []
        # 1) JSON ops frame (small objects + removes). May be empty if all
        #    ops were points/mesh — still emit empty so frontend flushes.
        if json_ops:
            frames.append(json.dumps({"type": "scene", "ops": json_ops}))

        # 2) Binary header + binary frame (if any large arrays).
        if layouts:
            header = json.dumps({"type": "scene_binary", "layouts": layouts})
            frames.append(header)
            frames.append(b"".join(binary_payloads))
        return frames

    # --- per-kind serializers ---
    def _serialize_points(self, obj: SceneObject):
        p = obj.payload
        pos = p.get("positions")
        if pos is None or len(pos) == 0:
            return None, None
        pos = _to_f32(pos)
        n = pos.shape[0]
        if n > MAX_POINTS_PER_FRAME:
            log.warning("points obj %s has %d pts (>cap %d) — truncating",
                        obj.id, n, MAX_POINTS_PER_FRAME)
            pos = pos[:MAX_POINTS_PER_FRAME]
            n = MAX_POINTS_PER_FRAME
        colors = p.get("colors")
        has_colors = colors is not None and len(colors) == n
        blobs = [pos.tobytes()]
        if has_colors:
            blobs.append(_to_f32(colors).tobytes())
        meta = {"id": obj.id, "kind": "points", "n_points": int(n),
                "has_colors": bool(has_colors),
                "point_size": float(p.get("point_size", 0.05)),
                "meta": obj.meta}
        return meta, b"".join(blobs)

    def _serialize_mesh(self, obj: SceneObject):
        p = obj.payload
        pos = p.get("positions")
        idx = p.get("indices")
        if pos is None or idx is None or len(pos) == 0:
            return None, None
        pos = _to_f32(pos)
        idx = _to_u32(idx)
        n_v, n_t = pos.shape[0], idx.shape[0]
        colors = p.get("colors")
        has_colors = colors is not None and len(colors) == n_v
        blobs = [pos.tobytes(), idx.tobytes()]
        if has_colors:
            blobs.append(_to_f32(colors).tobytes())
        meta = {"id": obj.id, "kind": "mesh", "n_vertices": int(n_v),
                "n_triangles": int(n_t), "has_colors": bool(has_colors),
                "meta": obj.meta}
        return meta, b"".join(blobs)

    def _serialize_grid2d(self, obj: SceneObject):
        """Occupancy grid as int8 cells + origin/resolution in meta.
        payload: {'cells': HxW int8 (0=free,100=obstacle,-1=unknown),
                  'origin': [x,y], 'resolution': float}
        Binary: cells as raw int8 (H*W bytes). The frontend reads origin/res
        and H/W from the layout meta and maps cells → colors."""
        p = obj.payload
        cells = p.get("cells")
        if cells is None:
            return None, None
        cells = np.ascontiguousarray(cells, dtype=np.int8)
        if cells.ndim != 2:
            return None, None
        h, w = cells.shape
        origin = p.get("origin", [0.0, 0.0])
        res = float(p.get("resolution", 0.05))
        meta = {"id": obj.id, "kind": "grid2d", "width": int(w), "height": int(h),
                "origin": [float(origin[0]), float(origin[1])],
                "resolution": res, "meta": obj.meta}
        return meta, cells.tobytes()

    def _small_op(self, obj: SceneObject, action: str) -> dict:
        p = obj.payload
        if obj.kind == "box":
            return {"op": action, "id": obj.id, "kind": "box",
                    "size": list(p.get("size", [1, 1, 1])),
                    "color": list(p.get("color", [1, 1, 1])),
                    "pose": p.get("pose", _identity_pose()),
                    "meta": obj.meta}
        if obj.kind == "line":
            return {"op": action, "id": obj.id, "kind": "line",
                    "positions": [list(pt) for pt in p.get("positions", [])],
                    "color": list(p.get("color", [1, 1, 1])),
                    "width": float(p.get("width", 1.0)),
                    "meta": obj.meta}
        if obj.kind == "label":
            return {"op": action, "id": obj.id, "kind": "label",
                    "text": p.get("text", ""),
                    "position": list(p.get("position", [0, 0, 0])),
                    "color": list(p.get("color", [1, 1, 1])),
                    "meta": obj.meta}
        return {"op": action, "id": obj.id, "kind": obj.kind,
                "payload": p, "meta": obj.meta}
