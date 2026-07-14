import numpy as np
import os
import cv2

RESOLUTION = 0.05
OBS_HEIGHT_MIN = 0.1
MAX_HEIGHT = 1.5
# Half-extent of the fixed grid in meters. The grid is allocated ONCE at first
# update and NEVER resized or shifted. Origin is fixed at (0,0) — the robot's
# camera_init frame origin (FAST-LIO start pose) — so grid coordinates and
# camera_init world coordinates share the SAME origin. This is critical for
# target dispatch: the explorer computes targets in grid/world coords, and the
# robots interpret them in camera_init coords; if origins differ, every target
# is off by the offset. At 0.05m/px, 60m span = 1200x1200 = 1.4MB int8.
MAP_HALF_EXTENT_M = 30.0

class GridMap:
    def __init__(self, resolution=RESOLUTION):
        self.res = resolution
        self.grid = np.zeros((1, 1), dtype=np.int8)  # 0 free, 100 obstacle
        self.origin = np.array([0.0, 0.0])
        self.initialized = False

    def _world_to_grid(self, xy):
        return ((xy - self.origin) / self.res).astype(np.int32)

    def _snap(self, val):
        return np.floor(val / self.res) * self.res

    def _allocate_fixed(self, center_xy):
        """Allocate the full grid ONCE. Origin is fixed at (-HALF_EXTENT,
        -HALF_EXTENT) so that world (0,0) — the camera_init origin — maps to
        the CENTER of the grid. This ensures grid coordinates and camera_init
        world coordinates share the same origin, so targets computed by the
        explorer can be dispatched to robots without any offset correction."""
        e = MAP_HALF_EXTENT_M
        self.origin = np.array([-e, -e])
        size = int(round(2 * e / self.res))
        self.grid = np.zeros((size, size), dtype=np.int8)  # default free
        self.initialized = True

    def update(self, points):
        """Incrementally update grid with new points.

        Cells are 2-state: 0 free / 100 obstacle.
        A point below OBS_HEIGHT_MIN is "ground" — marks its cell free (0).
        A point in [OBS_HEIGHT_MIN, MAX_HEIGHT] marks the cell obstacle (100).
        Overhead points (z > MAX_HEIGHT) are ignored.
        Obstacle wins over free if both appear in the same cell.
        Cells with no data remain free (0) by default."""
        if len(points) == 0:
            return
        xy = points[:, :2]
        z = points[:, 2]

        if not self.initialized:
            self._allocate_fixed(xy.mean(axis=0))

        gx = self._world_to_grid(xy)
        h, w = self.grid.shape
        valid = (gx[:, 0] >= 0) & (gx[:, 0] < w) & (gx[:, 1] >= 0) & (gx[:, 1] < h)
        gx_v, z_v = gx[valid], z[valid]

        # Obstacles are permanent: once a cell is marked 100 it stays 100 even
        # if a later frame only sees floor points there (noise, occlusion, or a
        # different scan angle). So floor only clears cells that are NOT already
        # obstacles. This prevents the inflated obstacle visualization from
        # flickering (growing then shrinking) as frames alternate.
        floor = z_v < OBS_HEIGHT_MIN
        floor_cells = self.grid[gx_v[floor, 1], gx_v[floor, 0]]
        # Only set free where it's not already an obstacle.
        clear_mask = floor_cells != 100
        floor_y = gx_v[floor, 1][clear_mask]
        floor_x = gx_v[floor, 0][clear_mask]
        self.grid[floor_y, floor_x] = 0
        obs = (z_v >= OBS_HEIGHT_MIN) & (z_v <= MAX_HEIGHT)
        self.grid[gx_v[obs, 1], gx_v[obs, 0]] = 100

    def to_mesh(self):
        """Convert grid to TriangleMesh with colored cells."""
        import open3d as o3d
        obs_y, obs_x = np.where(self.grid == 100)
        n = len(obs_y)
        if n == 0:
            m = o3d.geometry.TriangleMesh()
            m.vertices = o3d.utility.Vector3dVector(np.zeros((1, 3)))
            m.triangles = o3d.utility.Vector3iVector(np.zeros((1, 3), dtype=np.int32))
            m.vertex_colors = o3d.utility.Vector3dVector(np.zeros((1, 3)))
            return m

        wx = self.origin[0] + obs_x * self.res
        wy = self.origin[1] + obs_y * self.res
        r = self.res
        verts = np.zeros((n * 4, 3), dtype=np.float64)
        verts[0::4, 0] = wx;        verts[0::4, 1] = wy
        verts[1::4, 0] = wx + r;    verts[1::4, 1] = wy
        verts[2::4, 0] = wx + r;    verts[2::4, 1] = wy + r
        verts[3::4, 0] = wx;        verts[3::4, 1] = wy + r
        verts[:, 2] = 0.001

        tris = np.zeros((n * 2, 3), dtype=np.int32)
        idx = np.arange(n) * 4
        tris[0::2, 0] = idx; tris[0::2, 1] = idx + 1; tris[0::2, 2] = idx + 2
        tris[1::2, 0] = idx; tris[1::2, 1] = idx + 2; tris[1::2, 2] = idx + 3

        colors = np.full((n * 4, 3), [0.8, 0.1, 0.1], dtype=np.float64)

        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(verts)
        mesh.triangles = o3d.utility.Vector3iVector(tris)
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
        return mesh

    def save(self, path):
        """Overwrite single file each call. path: e.g. 'gridmap.png'
        PNG: 255 (white) = free, 0 (black) = obstacle.
        Also writes .npy (raw int8 grid) and .json (origin/resolution)."""
        img = np.full(self.grid.shape, 255, dtype=np.uint8)  # default free
        img[self.grid == 100] = 0
        cv2.imwrite(path, img)
        # Raw grid array for downstream training/annotation
        npy_path = path.rsplit('.', 1)[0] + '.npy'
        np.save(npy_path, self.grid)
        meta_path = path.rsplit('.', 1)[0] + '.json'
        import json
        with open(meta_path, 'w') as f:
            json.dump({'origin': self.origin.tolist(), 'resolution': self.res,
                       'height': int(self.grid.shape[0]), 'width': int(self.grid.shape[1])}, f)
