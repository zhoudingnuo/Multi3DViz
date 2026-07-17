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
    this._takeover = null;   // robot_id currently under keyboard control, or null
    this._takeoverLoading = null; // robot_id whose channel is opening (spinner state)
    this._keys = new Set();  // pressed keys for velocity computation
    this._velTimer = null;   // 10Hz velocity send interval
    this._exploreActive = false;  // interactive explore: first Enter=start, rest=step
    this._streamMode = {};   // {robot_id: bool} — online(stream) vs batch mode per robot
    this._exploreMode = {};  // {robot_id: bool} — auto explore on/off per robot
    // Restore from localStorage (persists across restarts).
    try {
      this._streamMode = JSON.parse(localStorage.getItem('m3v_streamMode') || '{}');
      this._exploreMode = JSON.parse(localStorage.getItem('m3v_exploreMode') || '{}');
    } catch (_) { this._streamMode = {}; this._exploreMode = {}; }
    this._render();
    // Global keyboard handler (bound once, checks _takeover).
    this._onKeyDown = (e) => {
      // Space = toggle stand/lie during takeover.
      if (this._takeover && e.key === ' ' && !e.repeat) {
        e.preventDefault();
        this._handleSpace();
        return;
      }
      // Enter = confirm frontier targets in manual explore mode (legacy path).
      // Exploration step execution is now driven by the "执行一步" button in
      // the explore-status panel (see app.js).
      if (e.key === 'Enter' && !e.repeat) {
        const anyManual = Object.values(this._exploreMode).some(v => v !== true);
        if (anyManual) {
          this.ws.request({ type: 'confirm_targets' })
            .then(r => this._toast(r));
        }
      }
      this._handleKey(e, true);
    };
    this._onKeyUp = (e) => this._handleKey(e, false);
    document.addEventListener('keydown', this._onKeyDown);
    document.addEventListener('keyup', this._onKeyUp);
  }

  setRobots(robots) {
    this.robots = robots || [];
    this._renderList();
  }

  // Sync the 在线模式 (stream/batch) checkbox from the backend's authoritative
  // state. The backend's `state` message carries every plugin instance's
  // current properties, including LocalReplay#*.stream_mode. This is the source
  // of truth — it supersedes localStorage, which can drift if the app was
  // hard-killed before the backend persisted, or if the mode changed from
  // another client. Called on every state push (connect + property broadcasts).
  // (exploreMode stays localStorage-only: DualAgentExplorer is a single global
  // instance with no per-robot robot_id, so it can't be mapped to a row.)
  syncFromInstances(instances) {
    if (!Array.isArray(instances)) return;
    let changed = false;
    for (const inst of instances) {
      if (!inst || !inst.properties) continue;
      if (inst.name !== 'LocalReplay') continue;
      const rid = inst.properties.robot_id;
      if (!rid || !('stream_mode' in inst.properties)) continue;
      const v = inst.properties.stream_mode === true;
      if (this._streamMode[rid] !== v) {
        this._streamMode[rid] = v;
        changed = true;
      }
    }
    if (changed) {
      try {
        localStorage.setItem('m3v_streamMode', JSON.stringify(this._streamMode));
      } catch (_) {}
      this._renderList();
    }
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
          const labels = { launch: '启动', stop: '停止', restart: '重启' };
          if (window.dbg) window.dbg(`→ ${labels[act] || act} ${rid}... (等待pipeline就绪)`, 'send');
          // Show immediate feedback — the SSH call takes ~30s to return.
          this._toast({ ok: true, message: `${labels[act] || act}中... (约30s)` });
          this.ws.request({ type: 'robot_command', robot_id: rid, action: act })
            .then(r => { btn.disabled = false; this._toast(r); })
            .catch(() => { btn.disabled = false; });
        } else if (act === 'toggle_explore') {
          // Toggle auto-explore state (visual + backend property).
          const on = !(this._exploreMode[rid] === true);
          this._exploreMode[rid] = on;
          try { localStorage.setItem('m3v_exploreMode', JSON.stringify(this._exploreMode)); } catch(_) {}
          this.ws.send({ type: 'set_property', name: 'DualAgentExplorer',
                         key: 'auto_explore', value: on });
          this._renderList();
        } else if (act === 'estop') {
          this.ws.request({ type: 'robot_command', robot_id: rid, action: 'estop' })
            .then(r => this._toast(r));
        } else if (act === 'takeover') {
          this._toggleTakeover(rid);
        }
      });
    // stream/batch mode toggle checkbox
    el.querySelectorAll('[data-action="stream"]').forEach(cb => {
      cb.addEventListener('change', () => {
        const rid = cb.dataset.robot;
        const on = cb.checked;
        this._streamMode[rid] = on;
        try { localStorage.setItem('m3v_streamMode', JSON.stringify(this._streamMode)); } catch(_) {}
        // Send to the matching LocalReplay instance (by instance_id).
        // LocalReplay#1 = robot_a, LocalReplay#2 = robot_b.
        // Send to ALL LocalReplay instances (set_property with name applies
        // to every matching instance). This is simpler and more robust than
        // guessing instance_ids.
        this.ws.send({ type: 'set_property', name: 'LocalReplay', key: 'stream_mode', value: on });
        this.ws.send({ type: 'set_property', name: 'LocalReplay', key: 'instant_load', value: !on });
        if (window.dbg) window.dbg(`在线模式 ${rid}: ${on ? 'ON (stream)' : 'OFF (batch)'}`, 'warn');
      });
      });
    });
  }

  _robotRow(r) {
    const cls = stateClass(r.state);
    const online = r.state === 'online';
    const err = r.error ? `<span class="robot-err">${esc(r.error)}</span>` : '';
    const isTakeover = this._takeover === r.robot_id;
    const isTakeoverLoading = this._takeoverLoading === r.robot_id;
    const dis = online ? '' : 'disabled title="等待 SSH 连接..."';
    const streamOn = this._streamMode[r.robot_id] === true;
    const exploreOn = this._exploreMode[r.robot_id] === true;
    // Takeover button: 3 states — idle / loading (channel opening) / active
    let takeoverBtn;
    if (isTakeover && !isTakeoverLoading) {
      takeoverBtn = `<button class="ssh-btn takeover active" data-robot="${esc(r.robot_id)}" data-action="takeover">◉ 接管中</button>`;
    } else if (isTakeoverLoading) {
      takeoverBtn = `<button class="ssh-btn takeover loading" disabled>⏳</button>`;
    } else {
      takeoverBtn = `<button class="ssh-btn takeover" data-robot="${esc(r.robot_id)}" data-action="takeover" ${dis} title="接管后用 WASD 键盘控制">⌨ 接管</button>`;
    }
    const controls = `<div class="robot-ssh">
         <button class="ssh-btn launch" data-robot="${esc(r.robot_id)}" data-action="launch" ${dis} title="SSH 拉起 FAST-LIO + 录制 + 桥接全套">${icon('play', 12)} 启动</button>
         <button class="ssh-btn restart" data-robot="${esc(r.robot_id)}" data-action="restart" ${dis} title="先清理（cleanup）再重新启动 pipeline">${icon('refresh', 12)} 重启</button>
         <button class="ssh-btn explore-btn ${exploreOn ? 'active' : ''}" data-robot="${esc(r.robot_id)}" data-action="toggle_explore" ${dis} title="${exploreOn ? '自动探索中（点击关闭）' : '开启自动探索'}">${icon('refresh', 12)} ${exploreOn ? '◉ 探索' : '探索'}</button>
         ${takeoverBtn}
         <button class="ssh-btn estop-btn" data-robot="${esc(r.robot_id)}" data-action="estop" ${dis} title="紧急停止：停运动+趴下">${icon('estop', 12)} 急停</button>
       </div>
       <div class="robot-mode">
         <label class="mode-toggle" title="在线模式：只加载 5 分钟内的新数据（stream），否则回放全部历史（batch）">
           <input type="checkbox" data-robot="${esc(r.robot_id)}" data-action="stream" ${streamOn ? 'checked' : ''}/>
           <span>在线模式</span>
         </label>
         ${!exploreOn ? '<span class="manual-hint">手动模式 · Enter 确认目标</span>' : ''}
       </div>`;
    // Keyboard hint shown when THIS robot is under takeover.
    const hint = isTakeover
      ? `<div class="takeover-hint">W/S 前进后退 · A/D 左右 · Q/E 转向 · 空格 站立/趴下</div>`
      : '';
    return `<div class="robot-card ${cls} ${isTakeover ? 'takeover' : ''}">
        <div class="robot-head">
          <span class="robot-dot"></span>
          <div class="robot-info">
            <div class="robot-name">${esc(r.label || r.robot_id)} <span class="robot-id">${esc(r.robot_id)}</span>${batteryTag(r.battery_pct)}</div>
            <div class="robot-host">${esc(r.user)}@${esc(r.host)} · ${r.state}${err}</div>
          </div>
          <button class="btn-icon danger rm-inst" data-robot="${esc(r.robot_id)}" data-action="remove" title="Remove robot">${icon('trash', 14)}</button>
        </div>
        ${controls}
        ${hint}
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

  // --- keyboard takeover (WASD velocity control) ---
  _toggleTakeover(rid) {
    if (window.dbg) window.dbg(`takeover toggle: ${rid} (currently ${this._takeover || 'none'})`, 'warn');
    if (this._takeover === rid) {
      // Release: close channel (dog lies down + damp).
      this._takeover = null;
      this._takeoverLoading = null;
      this._keys.clear();
      if (this._velTimer) { clearInterval(this._velTimer); this._velTimer = null; }
      this.ws.request({ type: 'robot_command', robot_id: rid, action: 'takeover_end' })
        .then(r => { if (window.dbg) window.dbg(`takeover_end: ${JSON.stringify(r)}`, r.ok ? 'ok' : 'err'); });
    } else {
      // Take over: show loading state, send takeover_start, poll for channel ready.
      this._takeover = rid;
      this._takeoverLoading = rid;  // triggers "⏳ 连接中..." button
      this._keys.clear();
      this._renderList();
      this.ws.request({ type: 'robot_command', robot_id: rid, action: 'takeover_start' })
        .then(r => {
          if (window.dbg) window.dbg(`takeover_start: ${JSON.stringify(r)}`, r.ok ? 'ok' : 'err');
          if (!r || !r.ok) {
            if (window.dbg) window.dbg(`takeover FAILED`, 'err');
            this._takeover = null;
            this._takeoverLoading = null;
            this._renderList();
            return;
          }
          // Poll backend every 500ms to check if channel is ready.
          // Timeout after ~30s — the C++ m3v_agibot waits for checkConnect()
          // internally (up to 6s) + we pkill stale processes first (~1s) +
          // init_wait, so allow generous time. If it still hasn't come up,
          // tell the backend to clean up so we don't leak a zombie process.
          const startedAt = Date.now();
          const poll = () => {
            if (this._takeoverLoading !== rid) return;  // cancelled
            if (Date.now() - startedAt > 30000) {
              if (window.dbg) window.dbg(`takeover ${rid} TIMEOUT — channel never came up (cleaning up)`, 'err');
              // Tell backend to close the channel + kill the robot-side process.
              this.ws.send({ type: 'robot_command', robot_id: rid, action: 'takeover_end' });
              this._takeover = null;
              this._takeoverLoading = null;
              this._renderList();
              return;
            }
            this.ws.request({ type: 'robot_command', robot_id: rid, action: 'channel_status' })
              .then(sr => {
                if (sr && sr.ready) {
                  // Channel ready — start velocity sender + clear loading.
                  this._takeoverLoading = null;
                  if (this._velTimer) clearInterval(this._velTimer);
                  this._velTimer = setInterval(() => this._sendVel(), 100);
                  if (window.dbg) window.dbg(`channel READY for ${rid} — press SPACE to stand, WASD to move`, 'ok');
                  this._renderList();
                } else if (this._takeoverLoading === rid) {
                  // Still loading — keep polling.
                  setTimeout(poll, 500);
                }
              });
          };
          setTimeout(poll, 1500);  // first poll after 1.5s (give checkConnect time)
        });
    }
    this._renderList();
  }

  _handleKey(e, down) {
    if (!this._takeover) return;
    if (down && e.repeat) return;
    const k = e.key.toLowerCase();
    if (!'wasdqe'.includes(k)) return;
    if (down) this._keys.add(k); else this._keys.delete(k);
    if (window.dbg) window.dbg(`key ${down ? '↓' : '↑'} ${k} → keys=[${[...this._keys].join(',')}]`, '');
  }

  _handleSpace() {
    if (!this._takeover) return;
    if (this._takeoverLoading) {
      if (window.dbg) window.dbg('space ignored — channel still opening', 'warn');
      return;
    }
    this.ws.request({ type: 'robot_command', robot_id: this._takeover,
                      action: 'toggle_pose' })
      .then(r => {
        if (window.dbg) window.dbg(`toggle_pose: ${JSON.stringify(r)}`, r.ok ? 'ok' : 'err');
      });
  }

  _sendVel() {
    if (!this._takeover) return;
    const k = this._keys;
    const FWD = 0.4, SIDE = 0.3, TURN = 0.8;
    let vx = 0, vy = 0, yaw = 0;
    if (k.has('w')) vx += FWD;
    if (k.has('s')) vx -= FWD;
    if (k.has('a')) vy += SIDE;
    if (k.has('d')) vy -= SIDE;
    if (k.has('q')) yaw += TURN;
    if (k.has('e')) yaw -= TURN;
    // ALWAYS send — including zeros. If we skip zeros the dog keeps the last
    // velocity (e.g. yaw != 0 → spins forever). The wasMoving flag just
    // reduces debug log spam for repeated zeros.
    this.ws.send({ type: 'robot_vel', robot_id: this._takeover, vx, vy, yaw });
    // Log non-zero at 1Hz, zero only on transition.
    const isZero = (vx === 0 && vy === 0 && yaw === 0);
    if (isZero) {
      if (this._wasMoving) {
        this._wasMoving = false;
        if (window.dbg) window.dbg(`vel ${this._takeover}: STOP (0,0,0 sent)`, 'warn');
      }
    } else {
      this._wasMoving = true;
      this._velLogT = (this._velLogT || 0) + 1;
      if (window.dbg && this._velLogT % 10 === 0)
        window.dbg(`vel ${this._takeover}: vx=${vx.toFixed(2)} vy=${vy.toFixed(2)} yaw=${yaw.toFixed(2)}`, 'send');
    }
  }

  _toast(r) {
    // Show the backend's message field if present (e.g. "pipeline started
    // (5 ok, 1 warn)"), otherwise fall back to ok/error.
    const ok = r && r.ok !== false;
    const text = ok
      ? (r.message || '✓ ok')
      : `✗ ${r && r.error || 'failed'}`;
    // Debug console feedback so the user sees what happened.
    if (window.dbg) {
      window.dbg(ok ? `✓ ${text}` : `✗ ${text}`, ok ? 'ok' : 'err');
      // Also dump raw output for launch/stop/restart (helps debug pipeline issues).
      if (r && r.output) window.dbg(`  output: ${String(r.output).slice(0, 200)}`, 'info');
    }
    // Minimal transient status line at the form's top.
    const el = this.root.querySelector('.form-grid');
    if (!el) return;
    let t = el.querySelector('.toast');
    if (!t) {
      t = document.createElement('div');
      t.className = 'toast';
      el.prepend(t);
    }
    t.textContent = text;
    t.className = 'toast ' + (ok ? 'ok' : 'err');
    clearTimeout(this._toastT);
    this._toastT = setTimeout(() => { if (t) t.remove(); }, 4000);
  }
}

function stateClass(s) {
  if (s === 'online') return 'st-online';
  if (s === 'connecting' || s === 'reconnecting') return 'st-warn';
  return 'st-err';
}
function batteryTag(pct) {
  if (pct == null || pct < 0) return '';
  const cls = pct > 50 ? 'bat-ok' : pct > 20 ? 'bat-warn' : 'bat-err';
  return ` <span class="battery ${cls}">🔋${pct}%</span>`;
}
function esc(s) {
  return String(s == null ? '' : s).replace(/[&<>"']/g,
    c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}
