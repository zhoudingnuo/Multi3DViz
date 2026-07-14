// robot_panel.js — robot fleet management UI.
// Renders the list of robots with live connection state + an "add robot" form.
// Talks to the backend via ws_client: robot_add / robot_remove / robot_command.
//
// State colors follow theme.css status tokens:
//   online=green, connecting/reconnecting=yellow, disconnected/error=red.

import { icon } from './icons.js';

export class RobotPanel {
  constructor(rootEl, ws) {
    this.root = rootEl;
    this.ws = ws;
    this.robots = [];        // [{robot_id,label,host,user,state,error,last_seen}]
    this._render();
  }

  setRobots(robots) {
    this.robots = robots || [];
    this._renderList();
  }

  // --- render ---
  _render() {
    this.root.innerHTML =
      `<div class="section-title">Robots</div>` +
      `<div id="robot-list"></div>` +
      `<details class="add-form"><summary>+ Add robot</summary>` +
        this._addForm() +
      `</details>`;
    this._renderList();
    this._wireAddForm();
  }

  _renderList() {
    const el = this.root.querySelector('#robot-list');
    if (!this.robots.length) {
      el.innerHTML = `<div class="robot-empty">No robots. Add one below.</div>`;
      return;
    }
    el.innerHTML = this.robots.map(r => this._robotRow(r)).join('');
    // Wire per-robot buttons.
    el.querySelectorAll('[data-action]').forEach(btn => {
      btn.addEventListener('click', () => {
        const rid = btn.dataset.robot;
        const act = btn.dataset.action;
        if (act === 'remove') {
          if (!confirm(`Remove robot ${rid}?`)) return;
          this.ws.request({ type: 'robot_remove', robot_id: rid });
        } else if (act === 'launch' || act === 'stop' || act === 'restart') {
          btn.disabled = true;
          this.ws.request({ type: 'robot_command', robot_id: rid, action: act })
            .then(r => { btn.disabled = false; this._toast(r); });
        }
      });
    });
  }

  _robotRow(r) {
    const cls = stateClass(r.state);
    const online = r.state === 'online';
    const err = r.error ? `<span class="robot-err">${esc(r.error)}</span>` : '';
    // SSH / FAST-LIO controls: prominent labeled buttons when online.
    const sshControls = online
      ? `<div class="robot-ssh">
           <button class="ssh-btn launch" data-robot="${esc(r.robot_id)}" data-action="launch">${icon('play', 12)} Start FAST-LIO</button>
           <button class="ssh-btn stop" data-robot="${esc(r.robot_id)}" data-action="stop">${icon('stop', 12)} Stop</button>
         </div>`
      : `<div class="robot-ssh offline-note">SSH offline — connect to control</div>`;
    return `<div class="robot-card ${cls}">
        <div class="robot-head">
          <span class="robot-dot"></span>
          <div class="robot-info">
            <div class="robot-name">${esc(r.label || r.robot_id)} <span class="robot-id">${esc(r.robot_id)}</span></div>
            <div class="robot-host">${esc(r.user)}@${esc(r.host)} · ${r.state}${err}</div>
          </div>
          <button class="btn-icon danger rm-inst" data-robot="${esc(r.robot_id)}" data-action="remove" title="Remove robot">${icon('trash', 14)}</button>
        </div>
        ${sshControls}
      </div>`;
  }

  _addForm() {
    return `<div class="form-grid">
      <label>ID<input type="text" id="rf-id" placeholder="robot_a"/></label>
      <label>Label<input type="text" id="rf-label" placeholder="Unitree Go2"/></label>
      <label>Host<input type="text" id="rf-host" placeholder="10.60.77.187"/></label>
      <label>User<input type="text" id="rf-user" placeholder="unitree"/></label>
      <label>Password <small>(blank=key auth)</small><input type="password" id="rf-pw" placeholder="•••"/></label>
      <label>Data path<input type="text" id="rf-data" placeholder="C:\\robots\\unitree"/></label>
      <label>Launch cmd <small>(FAST-LIO)</small><input type="text" id="rf-launch" placeholder="roslaunch fast_lio mapping.launch"/></label>
      <button id="rf-add" class="add-btn">Add</button>
    </div>`;
  }

  _wireAddForm() {
    const btn = this.root.querySelector('#rf-add');
    btn.addEventListener('click', () => {
      const val = id => this.root.querySelector(id).value.trim();
      const payload = {
        type: 'robot_add',
        robot_id: val('#rf-id'),
        label: val('#rf-label'),
        host: val('#rf-host'),
        user: val('#rf-user'),
        password: val('#rf-pw'),
        data_path: val('#rf-data'),
        launch_cmd: val('#rf-launch'),
      };
      if (!payload.robot_id || !payload.host) {
        this._toast({ ok: false, error: 'ID and Host are required' });
        return;
      }
      this.ws.request(payload).then(r => {
        if (r && r.ok) {
          // Clear the form.
          ['#rf-id', '#rf-label', '#rf-host', '#rf-user', '#rf-pw', '#rf-data', '#rf-launch']
            .forEach(id => this.root.querySelector(id).value = '');
        }
        this._toast(r);
      });
    });
  }

  _toast(r) {
    // Minimal transient status line at the form's top.
    const el = this.root.querySelector('.form-grid');
    if (!el) return;
    let t = el.querySelector('.toast');
    if (!t) {
      t = document.createElement('div');
      t.className = 'toast';
      el.prepend(t);
    }
    t.textContent = r && r.ok ? '✓ ok' : `✗ ${r && r.error || 'failed'}`;
    t.className = 'toast ' + (r && r.ok ? 'ok' : 'err');
    clearTimeout(this._toastT);
    this._toastT = setTimeout(() => { if (t) t.remove(); }, 2500);
  }
}

function stateClass(s) {
  if (s === 'online') return 'st-online';
  if (s === 'connecting' || s === 'reconnecting') return 'st-warn';
  return 'st-err';
}
function esc(s) {
  return String(s == null ? '' : s).replace(/[&<>"']/g,
    c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}
