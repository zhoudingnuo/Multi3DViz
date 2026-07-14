// main.js — Electron main process.
// Responsibilities:
//   1. Spawn the Python backend (sidecar) and read its bound WS port from
//      the "READY ws://127.0.0.1:PORT" line on stdout.
//   2. Create the BrowserWindow loading frontend/index.html, injecting the
//      WS URL so the renderer connects to the right port.
//   3. Healthcheck the backend before showing the window (avoid a flash of
//      "disconnected" if Python is slow to bind).
//   4. Tear down the sidecar on app quit (all exit paths) so no zombie Python.
//
// The Python interpreter resolves to the project venv (.venv) if present,
// else the system `python`. On a clean checkout without the venv this will
// fail loudly in the dev console — see docs/ARCHITECTURE.md for setup.

const { app, BrowserWindow, ipcMain } = require('electron');
const { spawn } = require('child_process');
const path = require('path');
const http = require('http');

// Log uncaught exceptions to a file instead of popping a dialog, so packed
// builds don't scare the user with "A JavaScript error occurred in the main
// process" — we capture the stack for debugging.
process.on('uncaughtException', (err) => {
  const fs = require('fs');
  const logPath = path.join(app.getPath('userData'), 'main_crash.log');
  try {
    fs.appendFileSync(logPath, `[${new Date().toISOString()}] ${err.stack || err}\n`);
  } catch (_) {}
  console.error('[main:uncaught]', err.stack || err);
});

// Self-test: when the renderer reports it connected + rendered points, log
// and (in PROBE mode) quit. Lets us verify end-to-end without a GUI eyeball.
ipcMain.on('m3v-report', (e, payload) => {
  console.log('[main:report]', JSON.stringify(payload));
  if (process.env.M3V_PROBE) {
    setTimeout(() => app.quit(), 500);
  }
});

const ROOT = path.join(__dirname, '..');
const fs = require('fs');

// Packed backend: PyInstaller-frozen sidecar lives under resources/backend/
// after electron-builder packages it. In dev, we run the .py via the venv.
const PACKED_BACKEND = (process.resourcesPath && !process.resourcesPath.includes('electron'))
  ? path.join(process.resourcesPath, 'backend', 'm3v_backend.exe')
  : null;
const DEV_BACKEND_ENTRY = path.join(ROOT, 'backend', 'main.py');
const BACKEND_ENTRY = (PACKED_BACKEND && fs.existsSync(PACKED_BACKEND))
  ? PACKED_BACKEND : DEV_BACKEND_ENTRY;

// Resolve python: in dev use the project venv; when packed the backend IS the
// frozen exe (no python needed) — BACKEND_ENTRY already points at it.
function resolvePython() {
  if (PACKED_BACKEND && fs.existsSync(PACKED_BACKEND)) return PACKED_BACKEND;
  const venvPy = path.join(ROOT, '.venv', 'Scripts', 'python.exe');
  if (fs.existsSync(venvPy)) return venvPy;
  return 'python';
}

let backendProc = null;
let wsUrl = null;
let mainWindow = null;

function spawnBackend() {
  const py = resolvePython();
  // Packed mode: py IS the frozen backend exe — spawn it directly with no args.
  // Dev mode: spawn the venv python with the .py entry as argv[1].
  const packed = py === BACKEND_ENTRY;
  const args = packed ? [] : [BACKEND_ENTRY];
  console.log('[main] spawning backend:', py, args.join(' '));
  // Packed mode: do NOT use ROOT as cwd — ROOT points inside app.asar,
  // which is a virtual archive the OS can't chdir into. spawn() then fails
  // with ENOENT (confusingly, blamed on the exe rather than the cwd). Use
  // the backend's own directory (resources/backend/) instead.
  const spawnCwd = packed
    ? path.dirname(py)
    : ROOT;
  backendProc = spawn(py, args, {
    cwd: spawnCwd,
    env: { ...process.env, PYTHONUNBUFFERED: '1', PYTHONIOENCODING: 'utf-8' },
    windowsHide: true,
  });

  let buf = '';
  backendProc.stdout.on('data', (chunk) => {
    buf += chunk.toString();
    let idx;
    while ((idx = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, idx).trim();
      buf = buf.slice(idx + 1);
      if (!line) continue;
      console.log('[backend:out]', line);
      // The backend prints exactly one READY line with the bound port.
      const m = line.match(/^READY\s+(ws:\/\/\S+)/);
      if (m && !wsUrl) {
        wsUrl = m[1];
        createWindow();
      }
    }
  });
  backendProc.stderr.on('data', (chunk) => {
    process.stderr.write('[backend:err] ' + chunk.toString());
  });
  backendProc.on('exit', (code) => {
    console.log('[main] backend exited with', code);
    backendProc = null;
  });
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1400, height: 900,
    minWidth: 900, minHeight: 600,
    backgroundColor: '#0f172a',
    title: 'Multi3DViz',
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      // No preload: the WS URL is passed via URL hash (#port=XXXX), read by
      // app.js. This keeps contextIsolation on (safe) without needing a
      // runtime-generated preload file (which broke in packed asar mode).
    },
  });
  // Guard: if the window failed to create, bail before touching webContents.
  if (!mainWindow || mainWindow.webContents == null) {
    console.error('[main] BrowserWindow creation failed — no webContents');
    app.quit();
    return;
  }
  // Pass the WS port to the renderer via query string (?port=XXXX). The
  // renderer reads location.search in app.js. Using loadFile's query option is
  // the documented, stable way to pass data to the renderer without a preload.
  const port = wsUrl.match(/:(\d+)$/)[1];
  const htmlPath = path.join(ROOT, 'frontend', 'index.html');
  try {
    mainWindow.loadFile(htmlPath, { query: { port } });
  } catch (e) {
    // Fallback: plain loadFile (renderer uses its default port guess).
    console.error('[main] loadFile with query failed, falling back:', e.message);
    mainWindow.loadFile(htmlPath);
  }
  // Surface renderer errors and load failures to the main console (errors
  // only — keeps the log clean in normal operation). Guard against mainWindow
  // being destroyed between the event firing and the handler running — that
  // race produced "Cannot read properties of null (reading 'webContents')".
  mainWindow.webContents.on('did-fail-load', (e, code, desc, url) => {
    if (mainWindow && !mainWindow.isDestroyed())
      console.error('[main] did-fail-load', { code, desc, url });
  });
  mainWindow.webContents.on('render-process-gone', (e, d) => {
    if (mainWindow && !mainWindow.isDestroyed())
      console.error('[main] render-process-gone', JSON.stringify(d));
  });
  if (process.env.M3V_DEV) mainWindow.webContents.openDevTools();
  mainWindow.on('closed', () => { mainWindow = null; });
}

app.whenReady().then(() => {
  // Spawn the backend; its READY line (parsed in spawnBackend's stdout handler)
  // triggers writePreload() + createWindow() once the port is known.
  spawnBackend();
});

// Quit when all windows are closed (except on macOS).
app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') app.quit();
});

// Ensure the backend dies with the app, on every exit path.
function killBackend() {
  if (backendProc) {
    try { backendProc.kill(); } catch (_) {}
    backendProc = null;
  }
}
app.on('before-quit', killBackend);
app.on('will-quit', killBackend);
process.on('exit', killBackend);
process.on('SIGINT', () => { killBackend(); process.exit(0); });
process.on('SIGTERM', () => { killBackend(); process.exit(0); });
