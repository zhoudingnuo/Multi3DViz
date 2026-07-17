"""explorer.py — dual-agent frontier-based exploration for ccenter.

After ICP registration succeeds, both robots share a single explored map
aligned to the merged GridMap. This module detects frontier clusters
(boundaries between explored-free and unexplored-free) and assigns a
distinct target cluster to each robot.

Coordinates:
    World (wx, wy) — meters, merged-grid origin frame.
    Grid  (i, j)   — cells, i=x column, j=y row. world<->grid via gmap.origin/res.
    Array (j, i)   — numpy indexing, [row=y, col=x]. Clusters return (cy, cx).
"""
import numpy as np
from scipy.ndimage import (label as _nd_label, binary_dilation as _nd_dilate,
                           binary_erosion as _nd_erode)


class DualAgentExplorer:
    SENSOR_RADIUS_CELLS = 60   # 3 m at 0.05 m/px (set from config at startup)
    MIN_CLUSTER_SIZE = 12      # filter tiny frontier slivers
    # Direction-aware sensor model: the LiDAR sees farther forward than behind.
    # FRONT: ±105° from heading (210° total) at full SENSOR_RADIUS_CELLS.
    # REAR:  the remaining 150° cone at half radius.
    FRONT_HALF_ANGLE = np.radians(105)   # 210° / 2
    REAR_RADIUS_FRAC = 0.5               # rear sees 50% of forward range
    INFLATE_CELLS = 4          # inflate obstacles this many cells (~robot half-
                               # width, 0.2m) before labeling free components so
                               # diagonal slivers / narrow gaps are sealed off
                               # and only genuinely-traversable free space counts
                               # as one connected component.

    def __init__(self, gmap):
        self.gmap = gmap
        self.explored = np.zeros(gmap.grid.shape, dtype=bool)
        # Per-agent state
        self.targets = [None, None]      # (gy, gx) per agent, or None
        self.frontier_cells = None       # boolean HxW
        self.frontier_clusters = []      # list of (cy, cx, size, comp_id)
        # Region memory: 5m cell keys each agent has visited. Used to discourage
        # re-sweeping already-explored regions when fresh frontiers remain.
        self.visited = [set(), set()]
        # Connected-component cache of free cells (8-conn over inflated grid).
        # comp_id of an agent's cell == comp_id of a target cell  <=>  reachable
        # without crossing an obstacle. Recomputed each assign_targets call.
        self._conn_label = None
        self._conn_n = 0
        # Cached inflated-obstacle mask for _safe_target. Stored as
        # (inflate_value, mask) so changing INFLATE_CELLS via config slider
        # automatically invalidates the stale cache.
        self._inflated_cache = None
        self._inflated_cache_n = -1

    # --- grid bookkeeping ---
    def _expand_to_grid(self):
        """Ensure self.explored matches the gmap grid shape.

        Since GridMap now allocates a fixed-size grid once (origin never moves),
        this only fires once — when the gmap is first initialized. No offset
        correction needed because the grid never shifts."""
        H, W = self.gmap.grid.shape
        if self.explored.shape != (H, W):
            self.explored = np.zeros((H, W), dtype=bool)

    # --- connectivity / reachability ---
    def _compute_components(self):
        """Label 8-connected components of EXPLORED free space.

        Components are computed on (explored & free & ~obstacle-inflated), NOT
        on all free cells. This matters: a target must be reachable within the
        region the robot has already mapped and confirmed walkable. Labeling the
        entire free grid (including unexplored areas) would let the planner
        assign a target across unexplored space that may hide obstacles.

        Obstacles are inflated by INFLATE_CELLS (~robot half-width) first so
        diagonal slivers and narrow gaps too tight to traverse are sealed off."""
        self._expand_to_grid()
        g = self.gmap.grid
        obst = (g == 100)
        if self.INFLATE_CELLS > 0:
            k = self.INFLATE_CELLS
            st = np.ones((3, 3), dtype=bool)
            for _ in range(k):
                obst = _nd_dilate(obst, st)
        # Only EXPLORED free cells form components — the robot can only navigate
        # where it has actually observed.
        explored_free = self.explored & (g == 0) & ~obst
        self._conn_label, self._conn_n = _nd_label(
            explored_free, np.ones((3, 3), dtype=bool))

    def _comp_at(self, gy, gx):
        """Connected-component id at grid cell (gy,gx); 0 means not free.

        If the cell itself isn't free (obstacle, inflated margin, or out of
        bounds), fall back to the nearest free cell's component so an agent
        standing slightly inside an inflated band still gets a valid id."""
        H, W = self.gmap.grid.shape
        if self._conn_label is None:
            return 0
        if 0 <= gy < H and 0 <= gx < W:
            cid = int(self._conn_label[gy, gx])
            if cid != 0:
                return cid
        # Nearest-free fallback: search an expanding window for a free cell.
        r = max(1, self.INFLATE_CELLS)
        for span in range(r, max(H, W), r):
            y0, y1 = max(0, gy - span), min(H, gy + span + 1)
            x0, x1 = max(0, gx - span), min(W, gx + span + 1)
            sub = self._conn_label[y0:y1, x0:x1]
            nz = sub[sub != 0]
            if nz.size:
                return int(nz[0])
        return 0

    def _reachable(self, comp_id, gy, gx):
        """Is grid cell (gy,gx) in the same component as comp_id?"""
        if comp_id == 0 or self._conn_label is None:
            return False
        H, W = self.gmap.grid.shape
        if not (0 <= gy < H and 0 <= gx < W):
            return False
        return int(self._conn_label[gy, gx]) == comp_id

    def world_to_grid(self, wx, wy):
        i = int((wx - self.gmap.origin[0]) / self.gmap.res)
        j = int((wy - self.gmap.origin[1]) / self.gmap.res)
        return i, j

    def grid_to_world(self, gy, gx):
        wx = self.gmap.origin[0] + (gx + 0.5) * self.gmap.res
        wy = self.gmap.origin[1] + (gy + 0.5) * self.gmap.res
        return wx, wy

    def reached(self, agent_idx, world_pos, threshold_m=0.3):
        """Has this agent arrived at its assigned target?

        agent_idx: 0 (A) or 1 (B). world_pos: (wx, wy) in meters, merged frame.
        threshold_m: arrival distance in meters. Returns False if the agent has
        no target (None) — arrival is only meaningful when pursuing a goal."""
        target = self.targets[agent_idx]
        if target is None:
            return False
        gy, gx = target
        twx, twy = self.grid_to_world(gy, gx)
        d = float(np.hypot(world_pos[0] - twx, world_pos[1] - twy))
        return d < threshold_m

    # --- observation updates ---
    def mark_explored(self, world_xy, yaw=0.0):
        """Mark explored region around (wx, wy) as a direction-aware shape.

        The LiDAR sees farther forward than behind:
          FRONT: ±105° from heading (210° cone) at full SENSOR_RADIUS_CELLS.
          REAR:  the remaining 150° cone at half radius (REAR_RADIUS_FRAC).

        yaw: heading in radians (0 = +x). Extracted from the robot's rotation
        matrix by the caller. Only marks free cells (grid==0).
        """
        self._expand_to_grid()
        H, W = self.explored.shape
        gx, gy = self.world_to_grid(world_xy[0], world_xy[1])
        r = self.SENSOR_RADIUS_CELLS
        r_rear = int(r * self.REAR_RADIUS_FRAC)
        R = max(r, r_rear)
        x_lo, x_hi = max(0, gx - R), min(W, gx + R + 1)
        y_lo, y_hi = max(0, gy - R), min(H, gy + R + 1)
        if x_lo >= x_hi or y_lo >= y_hi:
            return
        yy, xx = np.ogrid[y_lo:y_hi, x_lo:x_hi]
        dx = xx - gx
        dy = yy - gy
        dist2 = dx ** 2 + dy ** 2
        # Angle of each cell relative to heading. world_to_grid maps world x→grid
        # col (gx), world y→grid row (gy). The robot heading yaw is in world frame
        # (atan2 of world y, world x), so cell angle in the same convention is
        # atan2(dy, dx) — but grid rows increase downward (= -y in world). We
        # negate dy to convert grid-row-delta to world-y-delta before atan2.
        cell_angle = np.arctan2(-dy, dx)  # world-frame angle of each cell
        rel_angle = cell_angle - yaw      # relative to heading
        # Wrap to [-pi, pi]
        rel_angle = (rel_angle + np.pi) % (2 * np.pi) - np.pi
        abs_rel = np.abs(rel_angle)
        # Front cone: within FRONT_HALF_ANGLE → full radius.
        # Rear cone: outside front → rear radius.
        in_front = abs_rel <= self.FRONT_HALF_ANGLE
        in_disk_front = in_front & (dist2 <= r * r)
        in_disk_rear = (~in_front) & (dist2 <= r_rear * r_rear)
        observed = in_disk_front | in_disk_rear
        free = (self.gmap.grid[y_lo:y_hi, x_lo:x_hi] == 0)
        self.explored[y_lo:y_hi, x_lo:x_hi] |= observed & free

    # --- frontier detection ---
    def detect_frontiers(self):
        """Compute frontier cells + cluster centroids.
        Frontier = explored-free cell adjacent (8-conn) to unexplored-free cell.
        Also (re)computes free-space connected components (see
        _compute_components) so each cluster carries the comp_id of its centroid
        — used by assign_targets to enforce same-component targeting.
        Sets self.frontier_cells (boolean HxW) and self.frontier_clusters
        (list of (cy, cx, size, comp_id)). Returns the cluster list."""
        self._expand_to_grid()
        self._compute_components()  # refresh self._conn_label for this frame
        self._inflated_cache = None  # grid changed → invalidate obstacle cache
        self._inflated_cache_n = -1
        free = (self.gmap.grid == 0)
        known_free = self.explored & free
        unexp_free = (~self.explored) & free
        # 1-cell dilation marks any explored-free cell touching unexplored-free
        unexp_dilated = _nd_dilate(unexp_free, np.ones((3, 3), dtype=bool))
        frontier = known_free & unexp_dilated
        self.frontier_cells = frontier
        if not frontier.any():
            self.frontier_clusters = []
            return []
        labeled, n = _nd_label(frontier, np.ones((3, 3), dtype=bool))
        out = []
        for cid in range(1, n + 1):
            mask = labeled == cid
            size = int(mask.sum())
            if size < self.MIN_CLUSTER_SIZE:
                continue
            ys, xs = np.where(mask)
            cy, cx = int(ys.mean()), int(xs.mean())
            # Use _comp_at (with nearest-free fallback) instead of raw
            # conn_label[cy,cx]: a frontier centroid can land on a cell that's
            # free (g==0) but inside the obstacle-inflation band (excluded from
            # the labeled free space), so conn_label[cy,cx] would be 0 even
            # though the frontier is genuinely reachable. _comp_at finds the
            # nearest labeled cell so the frontier keeps its real component id.
            comp_id = self._comp_at(cy, cx) if self._conn_label is not None else 0
            out.append((cy, cx, size, comp_id))
        # Sort by size desc — bigger frontiers are more interesting targets
        out.sort(key=lambda c: -c[2])
        self.frontier_clusters = out
        return out

    # --- target assignment ---
    def assign_targets(self, world_a, world_b):
        """Pick one frontier cluster per agent with four rules:

        0. SAME COMPONENT (hard constraint, all stages): a target is only ever
           assigned if it lies in the same free-space connected component as
           the agent. Without this, an agent on one side of a wall would be sent
           toward a frontier directly on the other side (straight-line close,
           but unreachable) and grind against the wall forever.
        1. Stickiness: if an agent's previous target is still near a current
           same-component cluster (< SENSOR_RADIUS), keep going to it. Prevents
           ping-pong between equally-scored clusters as frontiers shift.
        2. Separation: skip clusters within 1.5×SENSOR_RADIUS of the other
           agent's position (don't both pile into the same region).
        3. Region memory: skip clusters whose 5m cell has been visited by
           this agent before (encourages forward exploration, not re-sweeps).
           Cleared when no cluster survives the filters.
        4. Edge toward unreachable: if an agent's component has NO frontier
           cluster at all (every frontier is on the far side of a wall), send
           it to the in-component free cell closest (Euclidean) to the nearest
           unreachable frontier centroid — i.e. creep up to the wall so that,
           as the map fills in and a doorway/gap appears (components merge),
           the frontier becomes reachable next round. Still no candidate =>
           target None (hold).

        Single-robot mode: pass world_b=None — agent 1 is skipped entirely
        (no position, no target, no separation check for agent 0).

        Returns the list [(gy, gx) | None, (gy, gx) | None].
        """
        clusters = self.detect_frontiers()  # refreshes self._conn_label too
        gi_a, gy_a = self.world_to_grid(world_a[0], world_a[1])
        # Single-robot: world_b=None means agent 1 is absent. Use a far-away
        # sentinel so separation never triggers for agent 0, and skip agent 1's
        # target assignment (it stays None).
        single = world_b is None
        if not single:
            gi_b, gy_b = self.world_to_grid(world_b[0], world_b[1])
            pos = [(gy_a, gi_a), (gy_b, gi_b)]
        else:
            pos = [(gy_a, gi_a), (-99999, -99999)]
        comp = [self._comp_at(gy, gx) for (gy, gx) in pos]
        sensor = self.SENSOR_RADIUS_CELLS

        if not clusters:
            self.targets = [None, None]
            return self.targets

        # Pre-split clusters into reachable / unreachable per component so each
        # stage can test reachability in O(1) by comp_id membership.
        def reachable_clusters(vid):
            return [c for c in clusters if c[3] == vid and vid != 0]

        # 1. STICKINESS — keep current target if a same-component cluster is near
        #    AND the target is still reachable (same connected component as the
        #    robot). If the map updated and a new obstacle cut off the path,
        #    abandon the stale target and fall through to stage 2.
        new_targets = [None, None]
        for i in ((0,) if single else (0, 1)):
            if self.targets[i] is None:
                continue
            gy, gx = self.targets[i]
            # Reachability check: is the target still in the robot's component?
            tgt_comp = self._comp_at(gy, gx)
            if tgt_comp != comp[i] or comp[i] == 0:
                continue  # target became unreachable — drop it, reassign in stage 2
            nearest = None
            nearest_d = float("inf")
            for cy, cx, _sz, cpid in clusters:
                if cpid != comp[i]:           # hard: must be same component
                    continue
                d = float(np.hypot(gy - cy, gx - cx))
                if d < nearest_d:
                    nearest_d, nearest = d, (cy, cx)
            if nearest is not None and nearest_d < sensor:
                # Snap to the current cluster centroid — handles centroid drift
                # as the frontier shrinks while being explored. Ensure the
                # snapped cell isn't on an obstacle.
                new_targets[i] = self._safe_target(nearest[0], nearest[1])

        # 2. NEW TARGETS for agents that lost their sticky target.
        taken = {t for t in new_targets if t is not None}

        def cell_key(cy, cx, cell_size=100):
            """Quantize grid coords to 5m cells for region memory."""
            return (cy // cell_size, cx // cell_size)

        # Update visited memory for each agent at current position
        _agents = (0,) if single else (0, 1)
        for i in _agents:
            gy, gx = pos[i]
            self.visited[i].add(cell_key(gy, gx))

        for i in _agents:
            if new_targets[i] is not None:
                continue
            pos_self = pos[i]
            pos_other = pos[1 - i]
            scored = []
            for cy, cx, sz, cpid in clusters:
                if cpid != comp[i]:           # hard: same component
                    continue
                if (cy, cx) in taken:
                    continue  # already assigned to other agent this round
                d_other = float(np.hypot(pos_other[0] - cy, pos_other[1] - cx))
                if d_other < sensor * 1.5:
                    continue  # too close to other agent
                if cell_key(cy, cx) in self.visited[i]:
                    continue  # this 5m region already explored by this agent
                d_self = float(np.hypot(pos_self[0] - cy, pos_self[1] - cx))
                if d_self < sensor * 0.5:
                    continue  # agent is already on top of this cluster
                # Score: frontier size × direction clearness / distance.
                # ray_density penalizes frontiers behind walls/obstacles.
                # Exponential penalty so a ray that's 30%+ blocked drops sharply.
                ray_d = self._obstacle_density_on_ray(pos_self[0], pos_self[1], cy, cx)
                clearness = np.exp(-3.0 * ray_d)  # 0%→1.0, 30%→0.41, 60%→0.17
                utility = sz * clearness / (d_self + 1.0)
                scored.append((utility, cy, cx))
            if scored:
                scored.sort(key=lambda x: -x[0])
                best = self._safe_target(scored[0][1], scored[0][2])
                new_targets[i] = best
                taken.add(best)

        # 3. FALLBACK — filters wiped out all candidates for some agent.
        # Drop the region-memory constraint (re-sweeps are OK at this point)
        # then drop the separation constraint. The SAME-COMPONENT constraint is
        # NEVER relaxed — it is a hard safety bound, not a preference.
        for i in _agents:
            if new_targets[i] is not None:
                continue
            pos_self = pos[i]
            pos_other = pos[1 - i]
            cand = reachable_clusters(comp[i])  # only same-component clusters
            for relax in ("separation", "any"):
                scored = []
                for cy, cx, sz, cpid in cand:
                    if (cy, cx) in taken:
                        continue
                    if relax == "separation":
                        d_other = float(np.hypot(pos_other[0] - cy, pos_other[1] - cx))
                        if d_other < sensor * 1.5:
                            continue
                    d_self = float(np.hypot(pos_self[0] - cy, pos_self[1] - cx))
                    ray_d = self._obstacle_density_on_ray(pos_self[0], pos_self[1], cy, cx)
                    clearness = np.exp(-3.0 * ray_d)
                    utility = sz * clearness / (d_self + 1.0)
                    scored.append((utility, cy, cx))
                if scored:
                    scored.sort(key=lambda x: -x[0])
                    best = self._safe_target(scored[0][1], scored[0][2])
                    new_targets[i] = best
                    taken.add(best)
                    break

        # 4. EDGE TOWARD UNREACHABLE — the agent's component has no frontier at
        # all (every frontier is on the far side of a wall). Creep toward the
        # nearest unreachable frontier by aiming at the in-component free cell
        # closest to that frontier's centroid. As the map fills in and a gap
        # opens (components merge) the frontier becomes reachable next round.
        for i in _agents:
            if new_targets[i] is not None:
                continue
            if comp[i] == 0 or self._conn_label is None:
                continue
            # Nearest unreachable frontier centroid (different component).
            unre = [(cy, cx) for (cy, cx, _s, cpid) in clusters if cpid != comp[i]]
            if not unre:
                continue
            ty, tx = min(unre, key=lambda c: float(np.hypot(pos[i][0]-c[0], pos[i][1]-c[1])))
            tgt = self._nearest_free_in_component(comp[i], ty, tx, exclude=taken | {pos[0], pos[1]})
            if tgt is not None:
                safe = self._safe_target(tgt[0], tgt[1])
                new_targets[i] = safe
                taken.add(safe)

        self.targets = new_targets
        return self.targets

    def _inflated_obstacle_mask(self):
        """Cache of grid==100 dilated by INFLATE_CELLS. A cell is 'safe' for a
        target only if it's NOT in this mask. Cached with the inflate value so
        changing INFLATE_CELLS via the config slider picks up instantly."""
        if self._inflated_cache is not None and self._inflated_cache_n == self.INFLATE_CELLS:
            return self._inflated_cache
        obst = (self.gmap.grid == 100)
        if self.INFLATE_CELLS > 0:
            st = np.ones((3, 3), dtype=bool)
            for _ in range(self.INFLATE_CELLS):
                obst = _nd_dilate(obst, st)
        self._inflated_cache = obst
        self._inflated_cache_n = self.INFLATE_CELLS
        return obst

    def _obstacle_density_on_ray(self, y0, x0, y1, x1):
        """Fraction of inflated-obstacle cells along the line (y0,x0)->(y1,x1).

        Samples every cell on the line (Bresenham-style via numpy linspace).
        Returns 0.0 (fully clear path) to 1.0 (fully blocked). Used by the
        scoring formula to prefer frontiers in directions with fewer obstacles
        — the robot would rather head toward open space than toward a wall."""
        inflated = self._inflated_obstacle_mask()
        H, W = inflated.shape
        dy, dx = y1 - y0, x1 - x0
        length = max(abs(dy), abs(dx))
        if length == 0:
            return 0.0
        # Sample length+1 points along the line, rounded to cell indices.
        ts = np.linspace(0, 1, length + 1)
        ys = np.clip((y0 + ts * dy).round().astype(int), 0, H - 1)
        xs = np.clip((x0 + ts * dx).round().astype(int), 0, W - 1)
        blocked = inflated[ys, xs]
        return float(blocked.sum()) / float(len(blocked))

    def _safe_target(self, cy, cx):
        """Ensure a target cell is NOT on an obstacle OR its inflation band.

        Finds the nearest cell that is free (grid==0) AND outside the inflated
        obstacle mask, so the target never appears on top of an obstacle in the
        (inflated) gridmap panel."""
        g = self.gmap.grid
        H, W = g.shape
        inflated = self._inflated_obstacle_mask()
        # Fast path: already safe (free AND not in inflation band).
        if 0 <= cy < H and 0 <= cx < W:
            if g[cy, cx] == 0 and not inflated[cy, cx]:
                return (cy, cx)
        # Search outward for the nearest safe cell.
        r = max(1, self.INFLATE_CELLS)
        for span in range(r, max(H, W), r):
            y0, y1 = max(0, cy - span), min(H, cy + span + 1)
            x0, x1 = max(0, cx - span), min(W, cx + span + 1)
            safe_mask = (g[y0:y1, x0:x1] == 0) & (~inflated[y0:y1, x0:x1])
            nz = np.argwhere(safe_mask)
            if nz.size == 0:
                continue
            gy = nz[:, 0] + y0
            gx = nz[:, 1] + x0
            d2 = (gy - cy) ** 2 + (gx - cx) ** 2
            j = int(np.argmin(d2))
            return (int(gy[j]), int(gx[j]))
        return (cy, cx)  # fallback: return as-is

    def _nearest_free_in_component(self, comp_id, ty, tx, exclude=()):
        """Free cell of component `comp_id` nearest (Euclidean) to (ty,tx).

        Stays INFLATE_CELLS away from inflated obstacles so the picked cell is
        comfortably inside free space (not wedged in the inflation band). Cells
        in `exclude` (set of (y,x)) are skipped so two agents aren't aimed at
        the same boundary cell. Returns (gy, gx) or None."""
        if self._conn_label is None or comp_id == 0:
            return None
        lbl = self._conn_label
        mask = (lbl == comp_id)
        # Erode the component mask by INFLATE_CELLS so we only pick cells that
        # have free breathing room around them (not on the inflation fringe).
        if self.INFLATE_CELLS > 0:
            mask = mask & _nd_erode(mask, np.ones((3, 3), dtype=bool),
                                    iterations=self.INFLATE_CELLS)
        ys, xs = np.where(mask)
        if ys.size == 0:
            return None
        excl = set(exclude)
        best, best_d = None, float("inf")
        for y, x in zip(ys.tolist(), xs.tolist()):
            if (y, x) in excl:
                continue
            d = (y - ty) ** 2 + (x - tx) ** 2
            if d < best_d:
                best_d, best = d, (y, x)
        return best
