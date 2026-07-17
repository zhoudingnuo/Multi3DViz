// --- global debug console ---
window.dbg = function(msg, level = '') {
  const log = document.getElementById('dbg-log');
  if (!log) return;
  const t = new Date().toLocaleTimeString('en', { hour12: false });
  const cls = level ? `dbg-${level}` : '';
  const line = document.createElement('div');
  line.className = `dbg-line ${cls}`;
  line.innerHTML = `<span class="dbg-time">${t}</span> ${msg}`;
  log.appendChild(line);
  // Cap at 200 lines.
  while (log.children.length > 200) log.removeChild(log.firstChild);
  log.scrollTop = log.scrollHeight;
};
// Toggle button.
document.getElementById('dbg-toggle').addEventListener('click', () => {
  document.getElementById('dbg-panel').classList.toggle('open');
});
document.getElementById('dbg-clear').addEventListener('click', () => {
  document.getElementById('dbg-log').innerHTML = '';
});

// app.js — frontend bootstrap. Wires WSClient events to SceneManager +
// GridView + PluginPanel + playback bar + status bar. Reads the backend URL
// injected by Electron's preload; falls back to a fixed port for browser dev.

import { ws } from './ws_client.js';
import { SceneManager } from './scene.js';
import { GridView } from './grid_view.js';
import { PluginPanel } from './plugin_panel.js';
import { RobotPanel } from './robot_panel.js';
import { RegPanel } from './reg_panel.js';

// Resolve the backend WS URL. Three sources, in priority order:
//   1. Electron preload injection (window.M3V.wsUrl) — dev mode
//   2. location hash "#port=XXXX" — packed mode (main.js passes the port this
//      way because dynamically writing a preload file into read-only app.asar
//      is fragile; the hash is always available to the renderer)
//   3. fixed fallback — running in a plain browser against a known port
function _resolveWsUrl() {
  if (window.M3V && window.M3V.wsUrl) return window.M3V.wsUrl;
  // Packed mode: main.js passes the port via query string (?port=XXXX).
  const m = location.search.match(/port=(\d+)/);
  if (m) return `ws://127.0.0.1:${m[1]}`;
  return 'ws://127.0.0.1:8765';
}
const WS_URL = _resolveWsUrl();
// Report lifecycle back to the Electron main process (self-test/probe).
const report = (payload) => { if (window.M3V && window.M3V.report) window.M3V.report(payload); };

// --- 3D viewport ---
const viewport = document.getElementById('viewport');
const scene = new SceneManager(viewport);
scene.start();
// Shift+click in the 3D viewport sets a manual navigation target for robot A.
scene.onPick = (wx, wy) => {
  ws.send({ type: 'set_target', robot_id: 'robot_a', world: [wx, wy] });
  report({ event: 'set_target', x: +wx.toFixed(2), y: +wy.toFixed(2) });
};

// --- 2D grid panel ---
const gridPick = document.getElementById('grid-pick');
const grid = new GridView(document.getElementById('grid-canvas'), (p) => {
  const vtext = p.value >= 100 ? 'obstacle' : p.value < 0 ? 'unknown' : 'free';
  gridPick.textContent = `cell (${p.i},${p.j})  world (${p.worldX.toFixed(2)},${p.worldY.toFixed(2)})m  ${vtext}`;
});
// Highlight the active view button. Called on auto-switch + manual click.
function highlightViewBtn() {
  // activeView is 'auto' | 'merged' | 'robot_a' | 'robot_b'
  const av = grid.activeView;
  document.querySelectorAll('#grid-view-toggle .view-btn').forEach(b => {
    b.classList.toggle('active', b.dataset.view === av);
  });
}
grid.onViewChange = highlightViewBtn;
// Wire the view toggle buttons.
document.querySelectorAll('#grid-view-toggle .view-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    grid.setView(btn.dataset.view);
    // highlightViewBtn is called by setView→onViewChange, but call directly
    // too for immediate feedback.
    highlightViewBtn();
  });
});
highlightViewBtn();  // initial highlight
window.addEventListener('resize', () => { scene.resize(); grid.resize(); });
// The gridmap canvas lives in the controlpanel — its size isn't known until
// the CSS layout settles. Poll-resize for the first 2s after load.
let _rszCount = 0;
const _rszTimer = setInterval(() => {
  grid.resize();
  if (++_rszCount > 20) clearInterval(_rszTimer);
}, 100);

// --- plugin catalog sidebar ---
const panel = new PluginPanel(document.getElementById('plugin-list'), ws);

// --- robot fleet panel ---
const robotPanel = new RobotPanel(document.getElementById('robot-panel'), ws);

// --- ICP registration panel ---
const regPanel = new RegPanel(document.getElementById('reg-panel'), ws);

// --- WS event hooks ---
ws.onReady = () => {
  ws.send({ type: 'list_plugins' });
  ws.send({ type: 'robot_list' });   // populate the fleet panel on connect
};

ws.onCatalog = (plugins) => {
  panel.setCatalog(plugins);
  report({ event: 'catalog', n: plugins.length });
};
ws.onState = (enabled) => {
  panel.setState(enabled);
  // Sync per-robot mode toggles (在线模式 / 探索) from the backend's
  // authoritative property state. This prevents the UI from showing a stale
  // checkbox state from localStorage after a restart where the backend's
  // stream_mode wasn't persisted (e.g. app was hard-killed on Windows).
  robotPanel.syncFromInstances(enabled);
  document.getElementById('empty-hint').style.display = enabled.length ? 'none' : 'flex';
  report({ event: 'state', n: enabled.length });
};

ws.onSceneOps = (ops) => {
  scene.applyOps(ops);
  // Also route target_a/target_b box positions to the gridmap so it can draw
  // crosshair markers (like ccenter's grid panel). The 3D viewport shows the
  // box; the 2D gridmap shows a crosshair at the same world coord.
  for (const op of ops) {
    if (op.op === 'remove' && (op.id === 'target_a' || op.id === 'target_b')) {
      grid.setTarget(op.id, null);
    } else if ((op.id === 'target_a' || op.id === 'target_b') && op.kind === 'box') {
      // Box pose is a 4x4 matrix; world x,y from [0][3],[1][3].
      const wx = op.pose[0][3], wy = op.pose[1][3];
      grid.setTarget(op.id, [wx, wy]);
    }
  }
};
ws.onScenePoints = (op) => {
  scene.applyPointsOp(op);
  report({ event: 'points', id: op.id, n: op.positions ? op.positions.length / 3 : 0 });
};
ws.onSceneMesh = (op) => { scene.applyMeshOp(op); };
ws.onSceneGrid = (op) => {
  // Route by id: explorer_overlay → coverage/frontier layer; sem_overlay →
  // semantic/room layer; *_grid2d → base occupancy grid.
  if (op.id === 'explorer_overlay') {
    grid.setOverlay(op);
  } else if (op.id === 'sem_overlay') {
    grid.setSemOverlay(op);
  } else {
    grid.setGrid(op);
    // Gridmap is now a resident component (always visible in controlpanel) —
    // no need to toggle visibility. Just resize on first data.
    requestAnimationFrame(() => grid.resize());
  }
  report({ event: 'grid', id: op.id, w: op.width, h: op.height });
};
ws.onPlaybackState = (sources) => {
  // Per-robot frame display in the status bar. `sources` is a list of
  // {robot_id, playing, frame, max_frame, rate}. Render each robot into its
  // own slot so the two robots' progress isn't conflated into one number.
  const slots = { robot_a: document.getElementById('st-frame-a'),
                  robot_b: document.getElementById('st-frame-b') };
  // Reset both first (a source may be absent if a robot has no data yet).
  for (const id of Object.keys(slots)) {
    if (slots[id]) slots[id].textContent = '0/0';
  }
  for (const s of (sources || [])) {
    const el = slots[s.robot_id];
    if (el) el.textContent = `${s.frame}/${s.max_frame}`;
  }
};
ws.onRobotStatus = (robots) => { robotPanel.setRobots(robots); };
ws.onRegistrationStatus = (s) => { regPanel.setStatus(s); };
ws.onRegistrationProgress = (p) => { regPanel.setProgress(p); };
ws.onInstallProgress = (p) => { panel.setInstallProgress(p); };
ws.onPluginStatus = (s) => { panel.setPluginStatus(s); };

// --- semantics controls (gridmap section) ---
const semBtn = document.getElementById('sem-trigger-btn');
const semAuto = document.getElementById('sem-auto');
const semInterval = document.getElementById('sem-interval');
semBtn.addEventListener('click', () => {
  semBtn.disabled = true;
  ws.request({ type: 'semantics_trigger' }).then(r => {
    setTimeout(() => { semBtn.disabled = false; }, 500);
  });
});
semAuto.addEventListener('change', () => {
  // Enable/disable Semantics plugin + set predict_interval.
  if (semAuto.checked) {
    ws.request({ type: 'enable_plugin', name: 'Semantics' });
  } else {
    ws.request({ type: 'disable_plugin', name: 'Semantics' });
  }
});
semInterval.addEventListener('change', () => {
  const v = parseFloat(semInterval.value) || 5;
  ws.send({ type: 'set_property', name: 'Semantics', key: 'predict_interval', value: v });
});
ws.onProcessStats = (s) => {
  document.getElementById('st-mem').textContent = s.mem_mb;
  document.getElementById('st-cpu').textContent = s.cpu_pct;
};
ws.onInfoState = (i) => {
  // Points per robot (frame count comes from onPlaybackState, which is per-robot).
  document.getElementById('st-pts-a').textContent =
    (i.pts_a || 0).toLocaleString();
  document.getElementById('st-pts-b').textContent =
    (i.pts_b || 0).toLocaleString();
  const reg = document.getElementById('st-reg');
  reg.textContent = `${i.reg_status} (${i.reg_fitness.toFixed(2)})`;
  reg.className = 'item b ' + regClass(i.reg_status);
  document.getElementById('st-front').textContent = i.n_frontiers;
  document.getElementById('st-expl').textContent = i.explored_pct + '%';
  document.getElementById('st-robots').textContent = `${i.robots_online}/${i.robots_total}`;
  // Exploration status panel.
  const fmt = (v) => v === null || v === undefined ? '—' : v;
  const esMode = document.getElementById('es-mode');
  if (esMode) {
    esMode.textContent = i.explore_mode === 'single' ? '单机探索'
      : i.explore_mode === 'dual' ? '双机协同' : '待机';
    esMode.style.color = i.explore_mode === 'idle' ? '#858585'
      : i.explore_mode === 'dual' ? '#4ec9b0' : '#c9a23a';
  }
  const esTarget = document.getElementById('es-target');
  if (esTarget) {
    esTarget.textContent = i.explore_target
      ? `(${i.explore_target[0].toFixed(1)}, ${i.explore_target[1].toFixed(1)})` : '—';
  }
  // Live position: show the exploring robot's position, or both.
  const posA = i.robot_pos_a, posB = i.robot_pos_b;
  const esPos = document.getElementById('es-pos');
  if (esPos) {
    const parts = [];
    if (posA) parts.push(`A(${posA[0].toFixed(1)},${posA[1].toFixed(1)},${posA[2].toFixed(0)}°)`);
    if (posB) parts.push(`B(${posB[0].toFixed(1)},${posB[1].toFixed(1)},${posB[2].toFixed(0)}°)`);
    esPos.textContent = parts.length ? parts.join(' ') : '—';
  }
  // Update 2D grid robot position markers.
  if (typeof grid !== 'undefined' && grid) {
    grid.setRobotPos('robot_a', posA ? [posA[0], posA[1]] : null, posA ? posA[2] : 0);
    grid.setRobotPos('robot_b', posB ? [posB[0], posB[1]] : null, posB ? posB[2] : 0);
  }
  const esDist = document.getElementById('es-dist');
  if (esDist) {
    const parts = [];
    if (i.explore_dist !== null && i.explore_dist !== undefined) parts.push(`${i.explore_dist}m`);
    if (i.explore_angle !== null && i.explore_angle !== undefined) parts.push(`${i.explore_angle}°`);
    esDist.textContent = parts.length ? parts.join(' · ') : '—';
  }
  const esStep = document.getElementById('es-step');
  if (esStep) esStep.textContent = i.explore_running ? `${i.explore_step}/50` : '0/50';
  const esAct = document.getElementById('es-action');
  if (esAct) {
    if (!i.explore_target) {
      esAct.textContent = '无目标';
    } else if (i.explore_dist !== null && i.explore_dist < 0.3) {
      esAct.textContent = '已到达';
    } else if (Math.abs(i.explore_angle || 0) > 8.5) {
      esAct.textContent = `转向 ${i.explore_angle > 0 ? '↻' : '↺'}`;
    } else {
      esAct.textContent = '前进 0.5m';
    }
    esAct.style.color = i.explore_running ? '#c9a23a' : '#858585';
  }
  // Step button state.
  const stepBtn = document.getElementById('es-step-btn');
  if (stepBtn) stepBtn.disabled = !i.explore_target;
};
ws.onLog = (msg) => { console.log('[backend]', msg.level, msg.msg); };

// Exploration step/stop buttons.
let _exploreActive = false;
document.getElementById('es-step-btn').addEventListener('click', () => {
  const btn = document.getElementById('es-step-btn');
  btn.disabled = true;
  const onlineRids = robotPanel.robots.filter(r => r.state === 'online').map(r => r.robot_id);
  const targetRid = onlineRids.length === 1 ? onlineRids[0]
    : (onlineRids.includes('robot_b') ? 'robot_b' : onlineRids[0] || 'robot_a');
  const mode = _exploreActive ? 'step' : 'start';
  ws.request({ type: 'explore_execute', robot_id: targetRid, mode })
    .then(r => {
      btn.disabled = false;
      if (window.dbg) window.dbg(`explore ${mode}: ${JSON.stringify(r)}`, r.ok ? 'ok' : 'err');
      if (r.ok && r.action) {
        const doneActions = ['ARRIVED', 'STOPPED', 'MAXSTEPS', 'STUCK', 'ERROR'];
        _exploreActive = !doneActions.includes(r.action);
        if (doneActions.includes(r.action)) {
          robotPanel._toast({ ok: true, msg: `探索结束: ${r.action}` });
        }
      } else {
        _exploreActive = false;
        robotPanel._toast(r);
      }
    });
});
document.getElementById('es-stop-btn').addEventListener('click', () => {
  const onlineRids = robotPanel.robots.filter(r => r.state === 'online').map(r => r.robot_id);
  const targetRid = onlineRids.length === 1 ? onlineRids[0]
    : (onlineRids.includes('robot_b') ? 'robot_b' : onlineRids[0] || 'robot_a');
  ws.request({ type: 'explore_execute', robot_id: targetRid, mode: 'stop' })
    .then(r => { _exploreActive = false; });
});

// --- status bar fps (renderer-side, the rest come from backend events) ---
setInterval(() => {
  document.getElementById('st-fps').textContent = scene.stats.fps;
}, 500);
function regClass(s) {
  if (s === 'done') return 'st-ok';
  if (s === 'running') return 'st-warn';
  if (s === 'failed') return 'st-err';
  return '';
}

// --- connect ---
ws.connect(WS_URL);
console.log('Multi3DViz frontend connecting to', WS_URL);
