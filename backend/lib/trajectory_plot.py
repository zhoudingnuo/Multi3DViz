"""
trajectory_plot.py - Save exploration trajectory image combining gridmap + robot paths.

Visual style adapted from
CovSwarmRL/scripts/evaluate/eval_pathplanning_algorithms.py:plot_trajectory:

  - Gray-scaled grid background (free lighter, obstacle mid-gray, unknown darkest)
  - Green coverage overlay on explored free cells
  - Per-robot trajectory line in distinct color
  - Start (○) and end (■) markers with black edge
  - Title with frame/point/coverage stats

Called once at app exit and optionally when the user presses 'S'.

OPTIONAL DEP: matplotlib is imported lazily inside save_trajectory_figure so
this module loads even in a slim build that excludes it. If matplotlib is
missing, save_trajectory_figure returns False (export skipped).
"""
import os
import time
import numpy as np


# Per-robot display colors (hex). Match the ccenter_app viewport labels so the
# trajectory image is visually consistent with what the user saw during the run.
ROBOT_COLORS = {
    "Unitree": "#E74C3C",   # red — same family as the BGR-orange viewport dot
    "Agibot":  "#3498DB",   # blue
}


def save_trajectory_figure(gmap, robots, save_path, title_extra=None,
                            coverage_mask=None, targets=None):
    """Render and save a trajectory plot.

    Args:
        gmap: GridMap with .grid (HxW int8 -1/0/100), .origin (x,y), .res (m).
        robots: list of dicts:
            {'name': str, 'color': hex, 'trail': [(wx, wy), ...]}
            trail points are already in the gmap world frame (meters), e.g. the
            traj_a/traj_b lists recorded during exploration. No per-point
            transform is applied — trail must be in merged-grid coordinates.
        save_path: output PNG path. Parent dir is created if missing.
        title_extra: optional dict of extra title metrics (e.g. {'rmse': 0.054}).
        coverage_mask: optional HxW bool array marking cells the robots have
            actually explored (e.g. explorer.explored). If None, falls back to
            (grid == 0) — i.e. all free cells — which overstates coverage.
        targets: optional list of (wx, wy, color_hex, label) tuples marking
            each agent's current frontier target. Drawn as a crosshair above
            the trajectory lines.
    """
    if gmap is None or not getattr(gmap, "initialized", False):
        return None
    # Lazy import — matplotlib is excluded from the slim PyInstaller build.
    try:
        import matplotlib
        matplotlib.use("Agg")  # non-interactive backend; safe on any thread
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch
    except ImportError:
        print("[traj_plot] matplotlib not available (slim build); export skipped")
        return None
    grid = gmap.grid
    H, W = grid.shape
    res = gmap.res
    ox, oy = float(gmap.origin[0]), float(gmap.origin[1])

    fig, ax = plt.subplots(figsize=(12, 10))

    # ── 1) Background: tri-state gray. Darker = less known/less traversable.
    bg = np.full((H, W), 0.12, dtype=np.float32)   # unknown → very dark
    bg[grid == 0]   = 0.92                          # free → light
    bg[grid == 100] = 0.38                          # obstacle → mid-dark
    ax.imshow(bg, cmap="gray", origin="upper",
              extent=[0, W, H, 0], aspect="equal", interpolation="nearest")

    # ── 2) Coverage overlay: green tint where robot has actually explored.
    #       Falls back to "all free cells" only if caller passes no mask.
    #       Built as an explicit RGBA array (not a cmap+alpha imshow) because
    #       the Greens cmap at alpha=0.28 over a 0.92-gray free background is
    #       nearly invisible — a solid translucent green reads far better.
    mask_passed = coverage_mask is not None
    if coverage_mask is None:
        coverage_mask = (grid == 0)
    cov_rgba = np.zeros((H, W, 4), dtype=np.float32)
    cov_rgba[coverage_mask] = [0.40, 0.80, 0.45, 0.45]   # #66CC33-ish, 45% alpha
    ax.imshow(cov_rgba, origin="upper",
              extent=[0, W, H, 0], aspect="equal", interpolation="nearest")

    # ── 3) Per-robot trajectory. trail is [(wx, wy), ...] in world meters,
    #       already in the merged-grid frame — convert to grid cell coords.
    legend_handles = []
    for r in robots:
        name = r.get("name", "?")
        color = r.get("color", "#888888")
        trail = r.get("trail") or []
        if len(trail) < 2:
            continue
        # World (m) → grid indices for plotting against extent=[0,W,H,0]
        xs = [(p[0] - ox) / res for p in trail]
        ys = [(p[1] - oy) / res for p in trail]
        # Drop NaN/inf from bad samples
        xy = [(x, y) for x, y in zip(xs, ys) if np.isfinite(x) and np.isfinite(y)]
        if len(xy) < 2:
            continue
        xs = [p[0] for p in xy]
        ys = [p[1] for p in xy]
        ax.plot(xs, ys, color=color, linewidth=1.4, alpha=0.85, zorder=4)
        # Start marker — circle with black edge
        ax.scatter([xs[0]], [ys[0]], color=color, marker="o", s=110,
                   edgecolors="black", linewidths=1.5, zorder=6)
        # End marker — square with black edge
        ax.scatter([xs[-1]], [ys[-1]], color=color, marker="s", s=110,
                   edgecolors="black", linewidths=1.5, zorder=6)
        legend_handles.append(Patch(facecolor=color, edgecolor="black",
                                     label=f"{name} ({len(xs)} pts)"))

    # ── 3b) Current frontier targets — crosshair per agent.
    if targets:
        for wx, wy, tcolor, tlabel in targets:
            gx = (wx - ox) / res
            gy = (wy - oy) / res
            if not (np.isfinite(gx) and np.isfinite(gy)):
                continue
            ax.scatter([gx], [gy], color=tcolor, marker="x", s=160,
                       linewidths=2.4, zorder=7)
            ax.annotate(tlabel, (gx, gy), color=tcolor, fontsize=8,
                        xytext=(6, 6), textcoords="offset points", zorder=7)
            legend_handles.append(plt.Line2D(
                [0], [0], marker="x", color="none", markerfacecolor=tcolor,
                markeredgecolor=tcolor, markersize=10,
                label=f"Target {tlabel}"))

    # ── 4) Title with stats
    n_free = int((grid == 0).sum())
    n_obs  = int((grid == 100).sum())
    n_unk  = int((grid == -1).sum())
    total  = n_free + n_obs + n_unk
    # Coverage: if a real explored mask was passed, use explored/free (true
    # coverage of explorable space). Otherwise fall back to free/(free+unk).
    if mask_passed and n_free > 0:
        cov_pct = (float(coverage_mask.sum()) / n_free) * 100
    else:
        cov_pct = (n_free / max(n_free + n_unk, 1)) * 100

    title_lines = [
        f"CCenter · Exploration Trajectory",
        (f"Grid {W}×{H} @ {res*100:.0f}cm  ·  "
         f"Coverage {cov_pct:.1f}%  ·  Free {n_free:,}  Obs {n_obs:,}  Unk {n_unk:,}"),
    ]
    if title_extra:
        bits = [f"{k}={v}" for k, v in title_extra.items()]
        title_lines.append("  ·  ".join(bits))
    ax.set_title("\n".join(title_lines), fontsize=11, family="monospace")

    # ── 5) Legend (custom so we get color squares for each robot)
    if legend_handles:
        cov_patch = Patch(facecolor="#66CC33", edgecolor="none", alpha=0.45,
                          label="Explored")
        start_dot = plt.Line2D([0], [0], marker="o", color="none",
                                markerfacecolor="gray", markeredgecolor="black",
                                markersize=10, label="Start")
        end_dot = plt.Line2D([0], [0], marker="s", color="none",
                              markerfacecolor="gray", markeredgecolor="black",
                              markersize=10, label="End")
        ax.legend(handles=legend_handles + [cov_patch, start_dot, end_dot],
                  loc="upper right", fontsize=9, framealpha=0.85)

    ax.axis("off")
    plt.tight_layout()

    parent = os.path.dirname(save_path) or "."
    os.makedirs(parent, exist_ok=True)
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return save_path


def default_save_path(data_dir):
    """Build a timestamped save path under output/ next to the data dir."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(os.path.dirname(data_dir), "..", "output")
    out_dir = os.path.normpath(out_dir)
    return os.path.join(out_dir, f"trajectory_{ts}.png")
