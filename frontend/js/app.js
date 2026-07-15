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
ws.onPlaybackState = (sources) => { /* playback UI removed — data is live-streamed */ };
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
  document.getElementById('st-frame').textContent = `${i.frame}/${i.max_frame}`;
  document.getElementById('st-pts2').textContent =
    (i.pts_a + i.pts_b).toLocaleString();
  const reg = document.getElementById('st-reg');
  reg.textContent = `${i.reg_status} (${i.reg_fitness.toFixed(2)})`;
  reg.className = 'item b ' + regClass(i.reg_status);
  document.getElementById('st-front').textContent = i.n_frontiers;
  document.getElementById('st-expl').textContent = i.explored_pct + '%';
  document.getElementById('st-robots').textContent = `${i.robots_online}/${i.robots_total}`;
};
ws.onLog = (msg) => { console.log('[backend]', msg.level, msg.msg); };

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
