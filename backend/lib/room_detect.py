"""
room_detect.py — geometric room detection on a grid map.

Uses 4-connectivity connected-components on free cells (grid==0) to find
contiguous free regions. Each component above `min_cells` becomes a "room"
with a centroid in world coords. This is a heuristic — a long corridor is
also a connected component — but it gives a useful proxy for "areas of
interest" without relying on the trained UNet (which collapses on real
point-cloud grids).
"""
import numpy as np
from scipy.ndimage import label


def detect_rooms(grid, origin, res, min_cells=200):
    """Find connected free regions and return them sorted by size desc.

    Args:
        grid: HxW int8, 0=free, 100=obstacle, -1=unknown
        origin: (x, y) world position of grid[0,0]
        res: meters per cell
        min_cells: ignore components smaller than this (default 200 = 0.5 m²)

    Returns:
        (rooms, labels) where:
          rooms: list of dicts {id, size, cx, cy, area_m2, bbox}
          labels: HxW int32 — 0 means no room, 1..N is the room id (post-sort)
    """
    free = (grid == 0)
    H, W = grid.shape
    labels = np.zeros((H, W), dtype=np.int32)
    if not free.any():
        return [], labels
    # 4-connectivity structuring element
    struct = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=np.uint8)
    lab, n = label(free, structure=struct)
    if n == 0:
        return [], labels

    rooms = []
    for i in range(1, n + 1):
        ys, xs = np.where(lab == i)
        if len(xs) < min_cells:
            continue
        cx = float(origin[0] + (xs.mean() + 0.5) * res)
        cy = float(origin[1] + (ys.mean() + 0.5) * res)
        rooms.append({
            'id': 0,  # assigned after sorting
            'raw_label': i,
            'size': int(len(xs)),
            'area_m2': float(len(xs) * res * res),
            'cx': cx,
            'cy': cy,
            'bbox': (int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())),
        })

    # Sort by size descending; assign IDs in that order
    rooms.sort(key=lambda r: r['size'], reverse=True)
    for idx, r in enumerate(rooms, start=1):
        r['id'] = idx
        labels[lab == r['raw_label']] = idx
    return rooms, labels
