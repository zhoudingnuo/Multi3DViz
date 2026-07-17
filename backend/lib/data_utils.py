"""data_utils.py - Pure helpers for point cloud data and transforms.

No global state. Safe to import from anywhere.
"""
import json
import os
import numpy as np
import open3d as o3d

# Tunable constants
VOXEL_VIS = 0.1            # downsample voxel size for display
MAX_REG_PTS = 100000       # cap registration input size
VIS_INTERVAL = 20          # update visuals every N frames
GRID_REFRESH_INTERVAL = 100  # repaint the 2D GDI grid panel every N frames
# Hard cap on display point clouds (per vis window). As the robots move they
# re-scan the same space frame after frame, so accum_pts grows unbounded with
# heavy spatial overlap (one run hit 9.3M points in a single window, which
# exhausted GPU memory and killed the WGL context: "Failed to make context
# current: invalid handle"). When accum exceeds this, we re-voxel-downsample
# the WHOLE cloud — same VOXEL_VIS grid, so overlapping re-scans collapse back
# into their original cells. No visual fidelity lost; memory bounded.
MAX_ACCUM_PTS = 1_500_000  # ~1.5M pts/window ceiling


def load_gravity(run_dir):
    """Return (run_dir, R) where R is the 3x3 gravity-correction rotation.

    Accepts a run directory. If it has gravity_calibration.json, use it.
    Otherwise search SIBLING run directories (same parent) for the newest one
    that has a gravity file — the IMU calibration is roughly constant across
    runs on the same robot, so a recent calibration is better than none."""
    grav_file = os.path.join(run_dir, "gravity_calibration.json")
    if os.path.exists(grav_file):
        # Direct hit — use it.
        pass
    else:
        # Search sibling runs in the parent data directory for a gravity file.
        parent = os.path.dirname(run_dir)
        found = None
        try:
            siblings = [os.path.join(parent, d) for d in os.listdir(parent)
                        if os.path.isdir(os.path.join(parent, d))]
            # Prefer siblings that have gravity_calibration.json, pick newest.
            with_grav = [d for d in siblings
                         if os.path.exists(os.path.join(d, "gravity_calibration.json"))]
            if with_grav:
                found = max(with_grav)
        except OSError:
            pass
        if found:
            grav_file = os.path.join(found, "gravity_calibration.json")
        else:
            return run_dir, np.eye(3)
    if os.path.exists(grav_file):
        grav = json.load(open(grav_file))
        roll, pitch = np.radians(grav["roll_deg"]), np.radians(grav["pitch_deg"])
        cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
        R_roll = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        R_pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R = R_roll @ R_pitch
    else:
        R = np.eye(3)
    return run_dir, R


def create_grid(spacing=10, extent=500):
    """Ground-plane reference grid as an Open3D LineSet."""
    lines = []
    for i in range(-extent // spacing, extent // spacing + 1):
        v = i * spacing
        lines.append([[v, -extent, 0], [v, extent, 0]])
        lines.append([[-extent, v, 0], [extent, v, 0]])
    pts = np.array(lines).reshape(-1, 3)
    idx = [[i, i + 1] for i in range(0, len(pts), 2)]
    grid = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(pts),
        lines=o3d.utility.Vector2iVector(idx))
    grid.colors = o3d.utility.Vector3dVector([[0.5, 0.5, 0.5]] * len(idx))
    return grid


def height_color_blue_red(pts):
    """Blue (low) → red (high) height ramp for Unitree clouds."""
    z = pts[:, 2]
    t = (z - z.min()) / (z.max() - z.min() + 1e-8)
    c = np.zeros_like(pts)
    c[:, 0] = np.clip(1 - t, 0, 1)
    c[:, 2] = np.clip(t, 0, 1)
    c[:, 1] = np.clip(1 - np.abs(t - 0.5) * 2, 0, 1)
    return c


def height_color_cyan_yellow(pts):
    """Cyan (low) → yellow (high) height ramp for Agibot clouds."""
    z = pts[:, 2]
    t = (z - z.min()) / (z.max() - z.min() + 1e-8)
    c = np.zeros_like(pts)
    c[:, 0] = np.clip(t, 0, 1)
    c[:, 1] = np.clip(1 - np.abs(t - 0.5) * 2, 0, 1)
    c[:, 2] = np.clip(1 - t, 0, 1)
    return c


def quat_to_mat(qx, qy, qz, qw):
    """Quaternion → 4x4 homogeneous transform."""
    Rm = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]])
    T = np.eye(4)
    T[:3, :3] = Rm
    return T


def transform_mesh(mesh, T):
    """Apply 4x4 transform to a TriangleMesh, returning a new mesh that
    shares triangles/normals/colors but has translated vertices."""
    v = np.asarray(mesh.vertices)
    h = np.hstack([v, np.ones((len(v), 1))])
    vt = (T @ h.T).T[:, :3]
    m = o3d.geometry.TriangleMesh()
    m.vertices = o3d.utility.Vector3dVector(vt)
    m.triangles = mesh.triangles
    m.vertex_normals = mesh.vertex_normals
    m.vertex_colors = mesh.vertex_colors
    return m


def transform_points(pts, T):
    """Apply 4x4 transform to an (N,3) array of points."""
    h = np.hstack([pts, np.ones((len(pts), 1))])
    return (T @ h.T).T[:, :3]


def voxel_downsample(pts, voxel_size):
    """Voxel-grid downsample an (N,3) numpy array. Returns numpy array."""
    if len(pts) < 1000:
        return pts
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    down = pcd.voxel_down_sample(voxel_size)
    return np.asarray(down.points)


def cap_accum(pts, colors, max_pts=MAX_ACCUM_PTS, voxel=VOXEL_VIS):
    """Bound a display cloud by re-downsampling when it exceeds max_pts.

    As the robot re-scans the same region frame after frame, accum_pts fills
    with spatially-overlapping points and grows without limit — eventually
    exhausting GPU memory and killing the WGL context. This collapses the
    whole cloud back onto the same VOXEL grid the per-frame downsample used,
    so duplicate re-scans merge into their original cells: bounded memory,
    no visual loss. Points + colors are returned together (colors resampled
    by nearest original index). No-op if under the cap."""
    if len(pts) <= max_pts:
        return pts, colors
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    down = pcd.voxel_down_sample(voxel)
    return np.asarray(down.points), np.asarray(down.colors)
