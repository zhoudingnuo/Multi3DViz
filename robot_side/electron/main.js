// main.js — Electron main process for the m3v-agent desktop shell (受控端).
//
// Mirrors the control-side Multi3DViz electron/main.js pattern but inverted:
// here the Python child is the m3v-agent (受控端), and the window shows the
// agent's status panel (recorder/transport/executor/driver) + emergency stop.
//
// IPC contract with m3v_agent (spawned with --ui-stdio):
//   child stdout →  READY: {...}   agent started
//                   STATE: {...}   1 Hz status snapshot
//                   ESTOP_ACK: {...}  emergency_stop() result
//                   DYING: {...}   agent shutting down
//   child stdin  ←  ESTOP          trigger emergency stop
//                   STOP           graceful shutdown
//
// The window loads shell/index.html directly from disk (no HTTP server). The
// renderer polls state via the m3v:state IPC event pushed from here; the
// emergency-stop button calls m3v:estop, which writes ESTOP to stdin.

const { app, BrowserWindow, ipcMain } = require('electron');
const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs');

const ROOT = __dirname;                          // robot_side/electron/
const PKG_ROOT = path.join(ROOT, '..');          // robot_side/
const SHELL_DIR = path.join(PKG_ROOT, 'shell');  // robot_side/shell/

// --- locate the m3v-agent entry + python interpreter ---
// Dev: python3 -m m3v_agent.agent --ui-stdio -c <config>
// Packed (deb): the frozen m3v_agent binary under /opt/m3v-agent/
function resolveAgentCmd(configPath) {
  const cfg = configPath || process.env.M3V_CONFIG
    || '/etc/m3v-agent/config.yaml';
  // Packed: /opt/m3v-agent/m3v_agent (PyInstaller frozen) --ui-stdio -c cfg
  const packed = '/opt/m3v-agent/m3v_agent';
  if (process.resourcesPath && fs.existsSync(packed)) {
    return { py: packed, args: ['--ui-stdio', '-c', cfg] };
  }
  // Dev: run the package via python3 from PKG_ROOT
  return { py: 'python3', args: ['-m', 'm3v_agent.agent', '--ui-stdio', '-c', cfg] };
}

let agentProc = null;
let mainWindow = null;

function spawnAgent(configPath) {
  const { py, args } = resolveAgentCmd(configPath);
  console.log('[shell] spawning agent:', py, args.join(' '));
  agentProc = spawn(py, args, {
    cwd: PKG_ROOT,
    env: { ...process.env, PYTHONUNBUFFERED: '1', PYTHONIOENCODING: 'utf-8' },
  });

  let buf = '';
  agentProc.stdout.on('data', (chunk) => {
    buf += chunk.toString();
    let idx;
    while ((idx = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, idx).trim();
      buf = buf.slice(idx + 1);
      if (!line) continue;
      handleAgentLine(line);
    }
  });
  agentProc.stderr.on('data', (chunk) => {
    process.stderr.write('[agent:err] ' + chunk.toString());
  });
  agentProc.on('exit', (code) => {
    console.log('[shell] agent exited with', code);
    agentProc = null;
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('m3v:agent-exit', { code });
    }
  });
}

function handleAgentLine(line) {
  // Tagged JSON protocol. Anything without a known tag is logged as a log line.
  if (line.startsWith('STATE: ')) {
    try {
      const snap = JSON.parse(line.slice(7));
      if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('m3v:state', snap);
      }
    } catch (e) {
      console.log('[agent] bad STATE line:', line);
    }
    return;
  }
  if (line.startsWith('READY: ')) {
    try {
      const info = JSON.parse(line.slice(7));
      if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('m3v:ready', info);
      }
    } catch (e) { /* ignore */ }
    return;
  }
  if (line.startsWith('ESTOP_ACK: ')) {
    try {
      const ack = JSON.parse(line.slice(11));
      if (mainWindow && !mainWindow.isDestroyed()) {
        mainWindow.webContents.send('m3v:estop-ack', ack);
      }
    } catch (e) { /* ignore */ }
    return;
  }
  if (line.startsWith('DYING:')) {
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('m3v:agent-dying', {});
    }
    return;
  }
  if (line.startsWith('ERROR: ')) {
    if (mainWindow && !mainWindow.isDestroyed()) {
      mainWindow.webContents.send('m3v:agent-error', { msg: line.slice(7) });
    }
    return;
  }
  // Untagged line → log (the agent's own logging goes here).
  console.log('[agent:out]', line);
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 720, height: 780,
    minWidth: 480, minHeight: 600,
    backgroundColor: '#1e1e1e',
    title: 'm3v-agent 受控端',
    icon: path.join(SHELL_DIR, 'icon.png'),
    autoHideMenuBar: true,
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      preload: path.join(__dirname, 'preload.js'),
    },
  });
  mainWindow.loadFile(path.join(SHELL_DIR, 'index.html'));
}

// --- IPC from renderer ---
ipcMain.on('m3v:estop', () => {
  if (agentProc && agentProc.stdin && !agentProc.killed) {
    agentProc.stdin.write('ESTOP\n');
  }
});

// --- app lifecycle ---
app.whenReady().then(() => {
  createWindow();
  spawnAgent();
});

app.on('window-all-closed', () => {
  // Desktop shell: quit when the window closes (unlike the control side,
  // which keeps running for the tray). The agent is the child; killing it
  // here stops recording/navigation too.
  if (agentProc) {
    try { agentProc.stdin.write('STOP\n'); } catch (e) { /* ignore */ }
    setTimeout(() => {
      try { agentProc.kill('SIGTERM'); } catch (e) { /* ignore */ }
    }, 1500);
  }
  app.quit();
});

app.on('before-quit', () => {
  if (agentProc) {
    try { agentProc.kill('SIGTERM'); } catch (e) { /* ignore */ }
  }
});
