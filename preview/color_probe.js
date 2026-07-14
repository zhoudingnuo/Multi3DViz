const { app, BrowserWindow } = require('electron');
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');
const ROOT = 'C:/Users/Z790/Multi3DViz';
let bp = null, wsUrl = null, win = null;
const py = path.join(ROOT, '.venv', 'Scripts', 'python.exe');
bp = spawn(py, [path.join(ROOT, 'backend', 'main.py')], {
  cwd: ROOT, env: { ...process.env, PYTHONUNBUFFERED: '1' }, windowsHide: true });
let buf = '';
bp.stdout.on('data', d => {
  buf += d;
  let i;
  while ((i = buf.indexOf('\n')) >= 0) {
    const l = buf.slice(0, i).trim();
    buf = buf.slice(i + 1);
    const m = l.match(/^READY\s+(ws:\/\/\S+)/);
    if (m && !wsUrl) {
      wsUrl = m[1];
      fs.writeFileSync(path.join(ROOT, 'electron', 'preload.js'),
        'const { contextBridge, ipcRenderer } = require("electron");\n' +
        'contextBridge.exposeInMainWorld("M3V", { wsUrl: ' + JSON.stringify(wsUrl) +
        ', report:(p)=>ipcRenderer.send("m3v-report",p) });\n');
      win = new BrowserWindow({
        width: 1400, height: 900, backgroundColor: '#1e1e1e',
        webPreferences: { contextIsolation: true, preload: path.join(ROOT, 'electron', 'preload.js') }
      });
      win.loadFile(path.join(ROOT, 'frontend', 'index.html'));
      win.webContents.on('did-finish-load', () => {
        setTimeout(() => {
          win.webContents.executeJavaScript(
            "JSON.stringify({body:getComputedStyle(document.body).backgroundColor," +
            "sidebar:getComputedStyle(document.getElementById('sidebar')).backgroundColor," +
            "gridpanel:getComputedStyle(document.getElementById('gridpanel')).backgroundColor})"
          ).then(r => {
            console.log('COLORS:', r);
            win.capturePage().then(img => {
              fs.writeFileSync(path.join(ROOT, 'preview', 'shot.png'), img.toPNG());
              console.log('SHOT_SAVED');
              app.quit();
            });
          }).catch(e => { console.log('ERR', e.message); app.quit(); });
        }, 4000);
      });
    }
  }
});
app.whenReady();
