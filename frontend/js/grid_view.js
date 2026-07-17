// grid_view.js — 2D top-down occupancy grid renderer on a <canvas>.
// Cells→colors, auto-fit, wheel-zoom centered on cursor, right-drag pan,
// click→world coord (reported via onPick callback).
//
// Cell convention (from ccenter gridmap): 0=free, 100=obstacle, -1=unknown.
// Palette tuned to the deep-slate app theme (no blue) — muted warm obstacle
// tint so the panel reads as part of the same dark UI, not a foreign color.

// Cell colors — neutral gray base, desaturated warm obstacle, near-black unknown.
const COL = {
  free: '#262626',   // free space: neutral dark gray (matches panel)
  obs:  '#6e4a3a',   // obstacle: muted warm rust
  unk:  '#1a1a1a',   // unknown: darker than free
};

export class GridView {
  constructor(canvas, onPick) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.onPick = onPick || (() => {});
    // Current grid data
    this.cells = null;        // Int8Array (h*w) base occupancy (active view)
    this.w = 0; this.h = 0;
    this.origin = [0, 0];     // world meters
    this.res = 0.05;          // meters/cell
    // Multi-source base grids: { merged: {cells,w,h,origin,res}, robot_a: {...}, robot_b: {...} }
    this.grids = {};
    // Active view: 'auto' (auto-select by priority) | 'merged' | 'robot_a' | 'robot_b'
    this.activeView = 'auto';
    // Optional overlay grid (explorer coverage/frontier). Same encoding +
    // 1=explored(green tint), 2=frontier(yellow). Aligned by its own origin.
    this.ovCells = null; this.ovW = 0; this.ovH = 0;
    this.ovOrigin = [0,0]; this.ovRes = 0.05;
    // Optional semantic overlay (UNet classes / room ids). Same grid2d encoding
    // with 1-4=sem class, 10+=room id. Tinted by class color.
    this.semCells = null; this.semW = 0; this.semH = 0;
    this.semOrigin = [0,0]; this.semRes = 0.05;
    // View transform (auto-fit, then user zoom/pan)
    this.zoom = 1.0;          // multiplier on auto-fit cell size
    this.panX = 0; this.panY = 0;
    this.cellPx = 1;          // computed pixel size per cell
    this.px0 = 0; this.py0 = 0; // pixel origin of cell (0,0)
    this._dragging = false;
    this._dragStart = null;
    this._setupInput();
    this.resize();
  }

  // Receive a decoded grid2d op for a BASE occupancy grid. Routes by op.id:
  //   'merged_grid2d' → merged view, 'robot_a_grid2d' → A, 'robot_b_grid2d' → B
  // Stores all sources separately, then applies the active view (auto or manual).
  setGrid(op) {
    // Map op.id → source key. 'merged_grid2d' → 'merged'; '{rid}_grid2d' → rid.
    let key = null;
    if (op.id === 'merged_grid2d') key = 'merged';
    else if (op.id && op.id.endsWith('_grid2d')) key = op.id.replace('_grid2d', '');
    if (!key) return;
    const prev = this.grids[key];
    this.grids[key] = {
      cells: op.cells, w: op.width, h: op.height,
      origin: op.origin, res: op.resolution,
    };
    // In auto mode, re-evaluate the view whenever a NEW source first arrives
    // (prev was undefined) so we switch to it by priority.
    if (this.activeView === 'auto' && !prev) {
      this._autoSelect();
    }
    this._applyActiveView();
    this.draw();
  }

  // Select the view by auto-priority: merged > robot_a > robot_b.
  // Only switches UP in priority (merged wins) — doesn't downgrade once chosen
  // unless the source vanishes (handled by absence on next auto re-eval).
  _autoSelect() {
    if (this.grids.merged) { this._setActive('merged'); return; }
    if (this.grids.robot_a) { this._setActive('robot_a'); return; }
    if (this.grids.robot_b) { this._setActive('robot_b'); return; }
  }

  // Set activeView + update button highlight (if callback provided).
  _setActive(view) {
    if (this.activeView === view) return;
    this.activeView = view;
    if (this.onViewChange) this.onViewChange(view);
  }

  // Manual view selection via the toggle buttons. 'auto' re-enables auto-select.
  setView(view) {
    this.activeView = view;
    if (view === 'auto') this._autoSelect();
    this._applyActiveView();
    if (this.onViewChange) this.onViewChange(this.activeView);
    this.draw();
  }

  // Copy the active view's grid data into the render fields (this.cells etc.).
  _applyActiveView() {
    const view = this.activeView === 'auto'
      ? (this.grids.merged ? 'merged' : this.grids.robot_a ? 'robot_a' : this.grids.robot_b ? 'robot_b' : null)
      : this.activeView;
    const g = view ? this.grids[view] : null;
    if (g) {
      this.cells = g.cells; this.w = g.w; this.h = g.h;
      this.origin = g.origin; this.res = g.res;
      this._computeFit();
    } else {
      // Selected view has no data yet — clear the grid so the user sees the
      // switch (instead of the previous view lingering). The draw() method
      // shows "waiting for grid data..." when this.cells is null.
      this.cells = null;
      // Also clear the overlay so stale coverage/frontier from another view
      // doesn't bleed through.
      this.ovCells = null;
    }
  }

  // Receive the explorer overlay grid2d op. Rendered on top of the base grid.
  setOverlay(op) {
    this.ovCells = op.cells;
    this.ovW = op.width;
    this.ovH = op.height;
    this.ovOrigin = op.origin;
    this.ovRes = op.resolution;
    this.draw();
  }

  // Set/clear a target marker (world coords). id = 'target_a' | 'target_b'.
  setTarget(id, worldXY) {
    if (!this._targets) this._targets = {};
    if (worldXY === null) delete this._targets[id];
    else this._targets[id] = worldXY;
    this.draw();
  }

  // Set/clear a live robot position marker (world coords + heading deg).
  setRobotPos(id, worldXY, headingDeg) {
    if (!this._robotPos) this._robotPos = {};
    if (worldXY === null) delete this._robotPos[id];
    else this._robotPos[id] = { x: worldXY[0], y: worldXY[1], heading: headingDeg };
    this.draw();
  }

  // Set/clear a robot trajectory polyline (world XY). pts = [[x,y],...] or null.
  setTraj(id, pts) {
    if (!this._traj) this._traj = {};
    if (pts === null || pts === undefined) delete this._traj[id];
    else this._traj[id] = pts;
    this.draw();
  }

  // Receive the semantic overlay grid2d op (UNet classes / room ids).
  setSemOverlay(op) {
    this.semCells = op.cells;
    this.semW = op.width;
    this.semH = op.height;
    this.semOrigin = op.origin;
    this.semRes = op.resolution;
    this.draw();
  }

  resize() {
    const dpr = window.devicePixelRatio || 1;
    const rect = this.canvas.getBoundingClientRect();
    this.canvas.width = rect.width * dpr;
    this.canvas.height = rect.height * dpr;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.cssW = rect.width;
    this.cssH = rect.height;
    this._computeFit();
    this.draw();
  }

  // Auto-fit cell size so the whole grid fits, applying zoom + pan.
  _computeFit() {
    if (!this.cells || !this.cssW) return;
    const pad = 24, padTop = 30;
    const availW = Math.max(8, this.cssW - pad * 2);
    const availH = Math.max(8, this.cssH - padTop - pad);
    const base = Math.max(1, Math.min(availW / this.w, availH / this.h));
    this.cellPx = Math.max(1, Math.round(base * this.zoom));
    const totalW = this.cellPx * this.w;
    const totalH = this.cellPx * this.h;
    this.px0 = (this.cssW - totalW) / 2 + this.panX;
    this.py0 = padTop + (availH - totalH) / 2 + this.panY;
  }

  draw() {
    const ctx = this.ctx;
    ctx.fillStyle = '#181818';
    ctx.fillRect(0, 0, this.cssW, this.cssH);
    // Title
    ctx.fillStyle = '#4ec9b0';
    ctx.fillRect(12, 12, 7, 7);
    ctx.fillStyle = '#cccccc';
    ctx.font = 'bold 13px Segoe UI, sans-serif';
    ctx.fillText('GRID MAP · TOP-DOWN', 26, 20);
    // Legend
    const legend = [['free', COL.free], ['obs', COL.obs], ['unk', COL.unk]];
    let lx = this.cssW - 150;
    ctx.font = '11px Consolas, monospace';
    for (const [label, color] of legend) {
      ctx.fillStyle = color;
      ctx.fillRect(lx, 14, 9, 9);
      ctx.fillStyle = '#858585';
      ctx.fillText(label, lx + 13, 22);
      lx += 50;
    }
    if (!this.cells) {
      ctx.fillStyle = '#858585';
      ctx.font = '12px Segoe UI, sans-serif';
      ctx.fillText('waiting for grid data...', 14, 44);
      return;
    }
    // Render cells. Draw free as a base fill, then obstacles/unknown on top —
    // fewer fillRect calls than per-cell since most cells are free.
    const c = this.cellPx;
    // Base free rect
    ctx.fillStyle = COL.free;
    ctx.fillRect(this.px0, this.py0, this.w * c, this.h * c);
    // Obstacles + unknown — iterate once, batching by color.
    const rowStride = this.w;
    let obs = 0, unk = 0;
    // Y is flipped: j=0 (world +y) at top. We render with j=h-1 at top so the
    // view matches ccenter (north up). world y increases upward on screen.
    for (let j = 0; j < this.h; j++) {
      const screenY = this.py0 + (this.h - 1 - j) * c;
      for (let i = 0; i < this.w; i++) {
        const v = this.cells[j * rowStride + i];
        if (v === 0) continue;
        const screenX = this.px0 + i * c;
        if (v >= 100) { ctx.fillStyle = COL.obs; ctx.fillRect(screenX, screenY, c, c); obs++; }
        else if (v < 0) { ctx.fillStyle = COL.unk; ctx.fillRect(screenX, screenY, c, c); unk++; }
      }
    }
    // Explorer overlay: explored=green tint, frontier=yellow. Aligned by its
    // own origin (may differ slightly from the base grid after a rebuild).
    if (this.ovCells) {
      const oc = this.ovCells, ow = this.ovW, oh = this.ovH;
      // overlay cell pixel size — base grid's cellPx scaled by res ratio.
      const ocPx = Math.max(1, Math.round(this.cellPx * (this.ovRes / this.res)));
      // overlay pixel origin: base px0 + (ovOrigin - origin)/res * cellPx
      const ox0 = this.px0 + (this.ovOrigin[0] - this.origin[0]) / this.res * this.cellPx;
      const oy0 = this.py0 + (this.ovOrigin[1] - this.origin[1]) / this.res * this.cellPx;
      ctx.globalAlpha = 0.45;
      for (let j = 0; j < oh; j++) {
        const sy = oy0 + (oh - 1 - j) * ocPx;
        for (let i = 0; i < ow; i++) {
          const v = oc[j * ow + i];
          if (v === 1) { ctx.fillStyle = '#3a5a6a'; ctx.fillRect(ox0 + i*ocPx, sy, ocPx, ocPx); }
          else if (v === 2) { ctx.fillStyle = '#c9a23a'; ctx.fillRect(ox0 + i*ocPx, sy, ocPx, ocPx); }
        }
      }
      ctx.globalAlpha = 1.0;
    }
    // Semantic overlay: 1=wall(red),2=room(yellow),3=corridor(orange),
    // 4=furniture(purple), 10+=room id (cycled palette).
    if (this.semCells) {
      const sc = this.semCells, sw = this.semW, sh = this.semH;
      const scPx = Math.max(1, Math.round(this.cellPx * (this.semRes / this.res)));
      const sx0 = this.px0 + (this.semOrigin[0] - this.origin[0]) / this.res * this.cellPx;
      const sy0 = this.py0 + (this.semOrigin[1] - this.origin[1]) / this.res * this.cellPx;
      const pal = { 1:'#8a4a4a', 2:'#7a7548', 3:'#8a6a3a', 4:'#6a4a7a' };
      const roomPal = ['#4a6a8a','#5a7a5a','#6a5a7a','#7a7a5a','#5a7a7a','#7a5a5a'];
      ctx.globalAlpha = 0.45;
      for (let j = 0; j < sh; j++) {
        const sy = sy0 + (sh - 1 - j) * scPx;
        for (let i = 0; i < sw; i++) {
          const v = sc[j * sw + i];
          if (v <= 0) continue;
          let color;
          if (v >= 10) { color = roomPal[(v - 10) % roomPal.length]; }
          else { color = pal[v] || '#ffffff'; }
          ctx.fillStyle = color;
          ctx.fillRect(sx0 + i*scPx, sy, scPx, scPx);
        }
      }
      ctx.globalAlpha = 1.0;
    }
    // Trajectory polylines (drawn before robot discs so discs sit on top).
    // Also stamps the 3m sensor footprint along the path so the "explored
    // range" follows the WHOLE trail — not just the current pose (which is all
    // the single-position circle below shows).
    if (this._traj) {
      const c = this.cellPx;
      const sensorR = 3.0 / this.res * c;   // 3m footprint radius in pixels
      for (const [id, color] of [['robot_a', '#ff8800'], ['robot_b', '#cc00ff']]) {
        const pts = this._traj[id];
        if (!pts || pts.length < 1) continue;
        // Project trail to screen pixels once.
        const sp = pts.map(p => {
          const gi = (p[0] - this.origin[0]) / this.res;
          const gj = (p[1] - this.origin[1]) / this.res;
          return [this.px0 + gi * c, this.py0 + (this.h - 1 - gj) * c];
        });
        // ── explored footprint: ONE thick translucent stroke along the trail
        //    (lineWidth = 2*sensorR = 6m). round caps close the two ends into
        //    disks and round joins smooth corners, so the stroke alone is the
        //    full 3m buffer swept along the path. Do NOT also stamp a disk per
        //    trail point — overlapping translucent fills compound alpha within
        //    a single frame and turn dense sections opaque.
        //    Single-point trail (no segments to stroke) → one disk.
        ctx.save();
        ctx.globalAlpha = 0.13;
        ctx.fillStyle = color;
        ctx.strokeStyle = color;
        ctx.lineWidth = Math.max(2, sensorR * 2);
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';
        if (sp.length >= 2) {
          ctx.beginPath();
          ctx.moveTo(sp[0][0], sp[0][1]);
          for (let k = 1; k < sp.length; k++) ctx.lineTo(sp[k][0], sp[k][1]);
          ctx.stroke();
        } else {
          // Lone point: cap as a single disk.
          ctx.beginPath();
          ctx.arc(sp[0][0], sp[0][1], Math.max(2, sensorR), 0, Math.PI * 2);
          ctx.fill();
        }
        ctx.restore();
        // ── crisp trail line on top of the footprint.
        if (sp.length >= 2) {
          ctx.strokeStyle = color;
          ctx.lineWidth = Math.max(1.5, c * 0.6);
          ctx.lineJoin = 'round';
          ctx.globalAlpha = 0.85;
          ctx.beginPath();
          ctx.moveTo(sp[0][0], sp[0][1]);
          for (let k = 1; k < sp.length; k++) ctx.lineTo(sp[k][0], sp[k][1]);
          ctx.stroke();
          ctx.globalAlpha = 1.0;
        }
      }
    }
    // Robot position markers (disc + heading triangle). The single-position
    // explored circle is drawn ONLY when there's no trail yet for that robot
    // (otherwise the trail footprint above already covers the range).
    if (this._robotPos) {
      const c = this.cellPx;
      const sensorR = 3.0 / this.res * c;
      ctx.save();
      ctx.globalAlpha = 0.15;
      for (const [id, color] of [['robot_a', '#ff8800'], ['robot_b', '#cc00ff']]) {
        const rp = this._robotPos[id];
        if (!rp) continue;
        // Skip the lone circle if a trail exists — the footprint along the
        // trail (drawn above) is the real explored range.
        if (this._traj && this._traj[id] && this._traj[id].length >= 2) continue;
        const gi = (rp.x - this.origin[0]) / this.res;
        const gj = (rp.y - this.origin[1]) / this.res;
        const sx = this.px0 + gi * c;
        const sy = this.py0 + (this.h - 1 - gj) * c;
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(sx, sy, sensorR, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.restore();
      // Robot discs + heading triangles.
      for (const [id, color] of [['robot_a', '#ff8800'], ['robot_b', '#cc00ff']]) {
        const rp = this._robotPos[id];
        if (!rp) continue;
        const gi = (rp.x - this.origin[0]) / this.res;
        const gj = (rp.y - this.origin[1]) / this.res;
        const sx = this.px0 + gi * c;
        const sy = this.py0 + (this.h - 1 - gj) * c;
        const r = Math.max(6, c * 2.5);
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(sx, sy, r, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = '#fcfaf8';
        ctx.lineWidth = 1.5;
        ctx.stroke();
        const rad = rp.heading * Math.PI / 180;
        const dx = Math.cos(rad), dy = -Math.sin(rad);
        ctx.fillStyle = '#fcfaf8';
        ctx.beginPath();
        ctx.moveTo(sx + dx * r * 1.5, sy + dy * r * 1.5);
        ctx.lineTo(sx + (-dy - dx) * r * 0.7, sy + (dx - dy) * r * 0.7);
        ctx.lineTo(sx + (dy - dx) * r * 0.7, sy + (-dx - dy) * r * 0.7);
        ctx.closePath();
        ctx.fill();
      }
    }
    // Target markers (crosshair + filled square) — like ccenter's grid panel.
    // target_a = orange, target_b = magenta. Drawn at world coords.
    if (this._targets) {
      const c = this.cellPx;
      for (const [id, color] of [['target_a', '#e64a00'], ['target_b', '#e600c8']]) {
        const wxy = this._targets[id];
        if (!wxy) continue;
        const gi = (wxy[0] - this.origin[0]) / this.res;
        const gj = (wxy[1] - this.origin[1]) / this.res;
        const sx = this.px0 + gi * c;
        const sy = this.py0 + (this.h - 1 - gj) * c;
        const r = Math.max(5, c * 2);
        // Filled square body.
        ctx.fillStyle = color;
        ctx.fillRect(sx - r, sy - r, r * 2, r * 2);
        // White cross.
        ctx.fillStyle = '#fcfaf8';
        ctx.fillRect(sx - 2, sy - r * 0.6, 4, r * 1.2);
        ctx.fillRect(sx - r * 0.6, sy - 2, r * 1.2, 4);
      }
    }
    // Footer: world extent
    ctx.fillStyle = '#858585';
    ctx.font = '11px Consolas, monospace';
    const wx0 = this.origin[0], wy0 = this.origin[1];
    const wx1 = wx0 + this.w * this.res, wy1 = wy0 + this.h * this.res;
    ctx.fillText(`world X[${wx0.toFixed(1)}, ${wx1.toFixed(1)}]  Y[${wy0.toFixed(1)}, ${wy1.toFixed(1)}] m   ${this.w}×${this.h} cells   obs=${obs}`, 14, this.cssH - 8);
  }

  // --- input: wheel zoom (cursor-anchored), right-drag pan, click pick ---
  _setupInput() {
    const el = this.canvas;
    el.addEventListener('wheel', (e) => {
      e.preventDefault();
      if (!this.cells) return;
      const rect = el.getBoundingClientRect();
      const mx = e.clientX - rect.left, my = e.clientY - rect.top;
      // world cell under cursor
      const wi = (mx - this.px0) / this.cellPx;
      const wj = (my - this.py0) / this.cellPx;
      const factor = e.deltaY < 0 ? 1.2 : 1 / 1.2;
      const nz = Math.max(0.5, Math.min(20, this.zoom * factor));
      if (Math.abs(nz - this.zoom) < 1e-6) return;
      this.zoom = nz;
      this._computeFit();
      // adjust pan so the world cell stays under cursor
      const targetPx0 = mx - wi * this.cellPx;
      const targetPy0 = my - wj * this.cellPx;
      this.panX += targetPx0 - this.px0;
      this.panY += targetPy0 - this.py0;
      this._computeFit();
      this.draw();
    }, { passive: false });
    el.addEventListener('contextmenu', (e) => e.preventDefault());
    el.addEventListener('mousedown', (e) => {
      if (e.button !== 2) return; // right-drag only
      this._dragging = true;
      this._dragStart = { x: e.clientX, y: e.clientY, px: this.panX, py: this.panY };
    });
    window.addEventListener('mousemove', (e) => {
      if (!this._dragging) return;
      this.panX = this._dragStart.px + (e.clientX - this._dragStart.x);
      this.panY = this._dragStart.py + (e.clientY - this._dragStart.y);
      this._computeFit();
      this.draw();
    });
    window.addEventListener('mouseup', () => { this._dragging = false; });
    el.addEventListener('click', (e) => {
      if (!this.cells) return;
      const rect = el.getBoundingClientRect();
      const mx = e.clientX - rect.left, my = e.clientY - rect.top;
      const i = Math.floor((mx - this.px0) / this.cellPx);
      const j = (this.h - 1) - Math.floor((my - this.py0) / this.cellPx);
      if (i < 0 || i >= this.w || j < 0 || j >= this.h) return;
      const v = this.cells[j * this.w + i];
      const wx = this.origin[0] + (i + 0.5) * this.res;
      const wy = this.origin[1] + (j + 0.5) * this.res;
      this.onPick({ i, j, value: v, worldX: wx, worldY: wy });
    });
  }
}
