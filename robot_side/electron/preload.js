// preload.js — bridges the renderer (status panel) and the Electron main
// process. Runs in an isolated context; exposes a tiny `m3v` object to the
// window so the panel HTML can subscribe to state + send emergency stop.
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('m3v', {
  // Main → renderer: 1 Hz state snapshots.
  onState: (cb) => {
    const h = (_e, snap) => cb(snap);
    ipcRenderer.on('m3v:state', h);
    return () => ipcRenderer.removeListener('m3v:state', h);
  },
  // Main → renderer: agent finished starting.
  onReady: (cb) => ipcRenderer.on('m3v:ready', (_e, info) => cb(info)),
  // Main → renderer: estop was acknowledged (ok/fail).
  onEstopAck: (cb) => ipcRenderer.on('m3v:estop-ack', (_e, ack) => cb(ack)),
  // Main → renderer: agent died / exited unexpectedly.
  onAgentExit: (cb) => ipcRenderer.on('m3v:agent-exit', (_e, info) => cb(info)),
  onAgentDying: (cb) => ipcRenderer.on('m3v:agent-dying', () => cb()),
  onAgentError: (cb) => ipcRenderer.on('m3v:agent-error', (_e, info) => cb(info)),
  // Renderer → main: trigger emergency stop.
  estop: () => ipcRenderer.send('m3v:estop'),
});
