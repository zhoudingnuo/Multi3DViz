// ws_client.js — WebSocket client to the Python backend, with reconnect.
// Knows the wire protocol (see backend/core/ws_protocol.py):
//   - JSON text frames are parsed and dispatched by `type`
//   - A `scene_binary` JSON frame is ALWAYS immediately followed by one
//     binary frame; we stash the layouts and consume the binary bytes by
//     slicing float32 arrays per layout, then hand a decoded op to scene.js
//
// Exposes a singleton `ws` with: connect(url), send(obj), and event hooks
// onReady/onCatalog/onState/onSceneOp/onLog that the app sets.

export class WSClient {
  constructor() {
    this.ws = null;
    this.url = null;
    this._pendingBinaryLayouts = null; // layouts awaiting the next binary frame
    this._reconnectMs = 1000;
    this._reqId = 1;
    this._pending = new Map();          // reqId -> resolve fn
    // Event hooks (set by app)
    this.onReady = () => {};
    this.onCatalog = () => {};
    this.onState = () => {};
    this.onSceneOps = () => {};         // small JSON ops (box/line/label/remove)
    this.onScenePoints = () => {};      // decoded points op {id,positions,colors,...}
    this.onSceneMesh = () => {};        // decoded mesh op
    this.onSceneGrid = () => {};        // decoded grid2d op {id,cells,width,height,...}
    this.onPlaybackState = () => {};    // periodic playback snapshot [{playing,frame,...}]
    this.onRobotStatus = () => {};      // robot fleet list [{robot_id,state,...}]
    this.onRegistrationStatus = () => {};// periodic ICP state snapshot
    this.onRegistrationProgress = () => {};// per-trial ICP progress
    this.onProcessStats = () => {};    // periodic mem/cpu {mem_mb,cpu_pct}
    this.onInfoState = () => {};       // periodic aggregated info (frame/reg/...)
    this.onConnChange = () => {};       // 'connecting'|'open'|'closed'
  }

  connect(url) {
    this.url = url;
    this._open();
  }

  _open() {
    this.onConnChange('connecting');
    this.ws = new WebSocket(this.url);
    this.ws.binaryType = 'arraybuffer';
    this.ws.onopen = () => {
      this._reconnectMs = 1000;
      this.onConnChange('open');
      this.send({ type: 'hello', client: 'multi3dviz-frontend', version: '0.1' });
    };
    this.ws.onmessage = (ev) => this._onMessage(ev);
    this.ws.onclose = () => {
      this.onConnChange('closed');
      this.ws = null;
      // Reconnect with gentle backoff.
      setTimeout(() => this._open(), this._reconnectMs);
      this._reconnectMs = Math.min(this._reconnectMs * 1.5, 5000);
    };
    this.ws.onerror = () => { /* close handler will reconnect */ };
  }

  send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj));
    }
  }

  // Promise-based request: sets id, awaits matching response.
  request(obj) {
    const id = this._reqId++;
    return new Promise((resolve) => {
      this._pending.set(id, resolve);
      this.send({ ...obj, id });
    });
  }

  _onMessage(ev) {
    // Binary frame: must be the payload following a scene_binary header.
    if (ev.data instanceof ArrayBuffer) {
      try { this._consumeBinary(ev.data); }
      catch (e) { console.error('[ws] binary decode failed', e); }
      return;
    }
    let msg;
    try { msg = JSON.parse(ev.data); }
    catch (e) { console.error('[ws] bad json', e); return; }
    // Response to a request? resolve and stop.
    if (msg.type === 'response' && msg.id && this._pending.has(msg.id)) {
      this._pending.get(msg.id)(msg);
      this._pending.delete(msg.id);
      return;
    }
    switch (msg.type) {
      case 'ready':       this.onReady(); break;
      case 'catalog':     this.onCatalog(msg.plugins || []); break;
      case 'state':       this.onState(msg.enabled || []); break;
      case 'scene':       this.onSceneOps(msg.ops || []); break;
      case 'scene_binary': this._pendingBinaryLayouts = msg.layouts || []; break;
      case 'playback_state': this.onPlaybackState(msg.sources || []); break;
      case 'robot_status': this.onRobotStatus(msg.robots || []); break;
      case 'registration_status': this.onRegistrationStatus(msg); break;
      case 'registration_progress': this.onRegistrationProgress(msg); break;
      case 'install_progress': this.onInstallProgress && this.onInstallProgress(msg); break;
      case 'plugin_status': this.onPluginStatus && this.onPluginStatus(msg); break;
      case 'process_stats': this.onProcessStats(msg); break;
      case 'info_state':  this.onInfoState(msg); break;
      case 'log':         this.onLog && this.onLog(msg); break;
    }
  }

  // Decode one binary frame using the stashed layouts.
  _consumeBinary(buf) {
    const layouts = this._pendingBinaryLayouts;
    this._pendingBinaryLayouts = null;
    if (!layouts) return; // stray binary, ignore
    const dv = new DataView(buf);
    let off = 0;
    for (const lay of layouts) {
      if (lay.kind === 'points') {
        const n = lay.n_points;
        const positions = new Float32Array(buf, off, n * 3); off += n * 3 * 4;
        let colors = null;
        if (lay.has_colors) {
          colors = new Float32Array(buf, off, n * 3); off += n * 3 * 4;
        }
        this.onScenePoints({
          op: lay.op, id: lay.id, kind: 'points',
          positions, colors, point_size: lay.point_size || 0.04,
          meta: lay.meta || {},
        });
      } else if (lay.kind === 'mesh') {
        const nv = lay.n_vertices, nt = lay.n_triangles;
        const positions = new Float32Array(buf, off, nv * 3); off += nv * 3 * 4;
        const indices = new Uint32Array(buf, off, nt * 3); off += nt * 3 * 4;
        let colors = null;
        if (lay.has_colors) {
          colors = new Float32Array(buf, off, nv * 3); off += nv * 3 * 4;
        }
        this.onSceneMesh({
          op: lay.op, id: lay.id, kind: 'mesh',
          positions, indices, colors, meta: lay.meta || {},
        });
      } else if (lay.kind === 'grid2d') {
        // cells: W*H int8 (0=free, 100=obstacle, -1=unknown)
        const w = lay.width, h = lay.height;
        const cells = new Int8Array(buf, off, w * h); off += w * h;
        this.onSceneGrid({
          op: lay.op, id: lay.id, kind: 'grid2d',
          cells, width: w, height: h,
          origin: lay.origin || [0, 0], resolution: lay.resolution || 0.05,
          meta: lay.meta || {},
        });
      }
    }
  }
}

export const ws = new WSClient();
