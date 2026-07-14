// reg_panel.js — ICP registration status + progress panel.
// Subscribes to registration_status (periodic) + registration_progress
// (per-trial) WS events. Renders a compact card: state badge, fitness/rmse,
// last trial detail, and a "Re-register" button that forces ICP to re-run.

import { icon } from './icons.js';

export class RegPanel {
  constructor(rootEl, ws) {
    this.root = rootEl;
    this.ws = ws;
    this.status = { state: 'idle', fitness: 0, rmse: 0, has_transform: false };
    this.lastTrial = null;   // most recent progress payload
    this._render();
  }

  setStatus(s) {
    this.status = s || {};
    this._render();
  }

  setProgress(p) {
    // Per-trial detail — show the latest trial's fitness/rmse/score.
    if (p.phase === 'try') {
      this.lastTrial = p;
    } else if (p.phase === 'done') {
      this.lastTrial = p;
    } else if (p.phase === 'init' || p.phase === 'start') {
      this.lastTrial = p;
    }
    this._renderProgress();
  }

  _render() {
    const s = this.status;
    const state = s.state || 'idle';
    const badge = stateBadge(state);
    this.root.innerHTML =
      `<div class="section-title">Registration (ICP)</div>` +
      `<div class="reg-card ${badge.cls}">
        <div class="reg-row">
          <span class="reg-state ${badge.cls}">${badge.label}</span>
          <div class="reg-actions">
            <button class="btn-icon" id="reg-export" title="Export trajectory PNG">${icon('camera', 14)}</button>
            <button class="btn-icon" id="reg-rerun" title="Force re-registration">${icon('refresh', 14)}</button>
          </div>
        </div>
        <div class="reg-metrics">
          <span>fitness <b>${fmt(s.fitness, 3)}</b></span>
          <span>rmse <b>${fmt(s.rmse, 4)}</b></span>
        </div>
        <div class="reg-detail" id="reg-detail">—</div>
      </div>`;
    this.root.querySelector('#reg-rerun').addEventListener('click', () => {
      this.ws.request({ type: 'register' }).then(() => {});
    });
    const exp = this.root.querySelector('#reg-export');
    if (exp) exp.addEventListener('click', () => {
      exp.disabled = true;
      this.ws.request({ type: 'export_trajectory' }).then((r) => {
        exp.disabled = false;
        this._toast(r && r.ok ? { ok: true, msg: r.path } : { ok: false, error: 'export failed' });
      });
    });
    this._renderProgress();
  }

  _renderProgress() {
    const el = this.root.querySelector('#reg-detail');
    if (!el) return;
    const p = this.lastTrial;
    if (!p) { el.textContent = '—'; return; }
    if (p.phase === 'start') {
      el.textContent = `aligning B(${p.src_pts} pts) → A(${p.tgt_pts} pts)…`;
    } else if (p.phase === 'init') {
      el.textContent = `downsampled: src ${p.src_pts}, tgt ${p.tgt_pts}, max ${p.max_trials} trials`;
    } else if (p.phase === 'try') {
      el.textContent = `R${p.round}/T${p.trial}: fit=${fmt(p.fitness,3)} rmse=${fmt(p.rmse,4)} score=${fmt(p.score,1)} [${p.is_valid?'PASS':'try'}]`;
    } else if (p.phase === 'done') {
      el.textContent = p.ok
        ? `✓ done: fit=${fmt(p.fitness,3)} rmse=${fmt(p.rmse,4)}`
        : `✗ failed${p.error ? ': '+p.error : ''}`;
    }
  }
}

function stateBadge(state) {
  if (state === 'running') return { cls: 'st-warn', label: 'Running…' };
  if (state === 'done') return { cls: 'st-ok', label: 'Aligned' };
  if (state === 'failed') return { cls: 'st-err', label: 'Failed' };
  return { cls: 'st-idle', label: 'Idle' };
}
function fmt(v, n) { return (v == null || isNaN(v)) ? '—' : Number(v).toFixed(n); }
