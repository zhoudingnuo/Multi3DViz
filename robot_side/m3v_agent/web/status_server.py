"""status_server.py — Embedded ZCode-style status panel for the robot agent.

Runs an http.server.ThreadingHTTPServer on the robot. The panel is meant to be
opened from a browser ON the control-side machine (or any machine on the LAN)
pointed at http://<robot-ip>:<port>/ — it's a read-only monitor + emergency
stop. No auth (the robot is on a trusted field network; if you need auth, put
it behind an SSH tunnel or reverse proxy).

The panel shows, in real time (1 Hz poll):
  - Robot identity (id, host, driver kind, ROS stack)
  - Recorder: run dir, frame count, latest odom pose, gravity status
  - Transport (SCP): connection state, frames pushed, target host
  - Executor: current target (mode/local_x/local_y), nav state, file staleness
  - Driver: connected/standing, kind
  - A big red EMERGENCY STOP button (POST /api/estop)

All tokens match Multi3DViz frontend/css/theme.css so the two apps look like
one product. Static assets are inlined as module constants — no separate file
tree to ship, no path resolution at runtime.
"""
from __future__ import annotations
import json
import time
import logging
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Optional

log = logging.getLogger("m3v_agent.web")


class StatusServer:
    """Owns the HTTP server thread + a snapshot callback.

    Args:
        host/port: bind address. Default 0.0.0.0:8765 (LAN-reachable).
        snapshot: a zero-arg callable returning a dict — the agent wires this
                  to read its recorder/transport/executor/driver state.
        on_estop: a zero-arg callable triggered by the POST /api/estop button.
                  The agent wires this to driver.emergency_stop().
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8765,
                 snapshot: Optional[Callable[[], dict]] = None,
                 on_estop: Optional[Callable[[], bool]] = None):
        self.host = host
        self.port = port
        self._snapshot = snapshot or (lambda: {})
        self._on_estop = on_estop or (lambda: False)
        self._http: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._started_at = time.time()

    def start(self):
        # The handler needs a reference back to this server; closure-inject.
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # silence default stderr noise
                pass

            def _send(self, code: int, body: bytes, ctype: str):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):  # noqa: N802 (BaseHTTPRequestHandler convention)
                if self.path == "/" or self.path == "/index.html":
                    self._send(200, _INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                elif self.path == "/app.js":
                    self._send(200, _APP_JS.encode("utf-8"),
                               "application/javascript; charset=utf-8")
                elif self.path == "/api/state":
                    try:
                        snap = outer._snapshot()
                        snap.setdefault("server", {
                            "started_at": outer._started_at,
                            "uptime_s": round(time.time() - outer._started_at, 1),
                            "now": time.time(),
                        })
                        body = json.dumps(snap, default=_json_default).encode("utf-8")
                        self._send(200, body, "application/json")
                    except Exception as e:
                        log.exception("snapshot failed")
                        self._send(500, json.dumps({"error": str(e)}).encode(),
                                   "application/json")
                else:
                    self._send(404, b"not found", "text/plain")

            def do_POST(self):  # noqa: N802
                if self.path == "/api/estop":
                    try:
                        ok = bool(outer._on_estop())
                        self._send(200, json.dumps({"ok": ok}).encode(),
                                   "application/json")
                    except Exception as e:
                        log.exception("estop failed")
                        self._send(500, json.dumps({"error": str(e)}).encode(),
                                   "application/json")
                else:
                    self._send(404, b"not found", "text/plain")

        try:
            self._http = ThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as e:
            log.error("status server bind %s:%d failed: %s", self.host, self.port, e)
            return
        self._thread = threading.Thread(
            target=self._http.serve_forever, name="web-status", daemon=True)
        self._thread.start()
        log.info("status panel: http://%s:%d/", self.host, self.port)

    def stop(self):
        if self._http is not None:
            self._http.shutdown()
            self._http.server_close()
            self._http = None
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None


def _json_default(o: Any):
    """JSON fallback for non-serializable objects (np types, etc.)."""
    # numpy scalars/arrays
    for attr in ("tolist", "item"):
        if hasattr(o, attr):
            try:
                return getattr(o, attr)()
            except Exception:
                pass
    return str(o)


# ---------------------------------------------------------------------------
# Static assets (inlined so the panel ships as one module, no file tree).
# Tokens match Multi3DViz frontend/css/theme.css exactly.
# ---------------------------------------------------------------------------

_INDEX_HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>m3v-agent · 受控端状态</title>
<link rel="stylesheet" href="/app.js?css=1">
<style>
:root {
  --bg: #1e1e1e; --bg-elev-1: #252526; --bg-elev-2: #2d2d2d; --bg-inset: #181818;
  --border: #3c3c3c; --border-soft: #333333;
  --text: #cccccc; --text-value: #d4d4d4; --text-muted: #858585;
  --accent: #4ec9b0; --accent-dim: #3da89a;
  --status-ok: #4ec9b0; --status-warn: #dcdcaa; --status-err: #f48771; --status-run: #569cd6;
  --radius: 6px; --radius-sm: 4px; --gap: 8px; --gap-lg: 14px;
  --font: 'Segoe UI','Inter',system-ui,-apple-system,sans-serif;
  --font-mono: 'Cascadia Code','Consolas','Menlo',monospace;
}
* { box-sizing: border-box; }
html, body {
  margin: 0; padding: 0; min-height: 100vh;
  background: var(--bg); color: var(--text);
  font-family: var(--font); font-size: 13px;
}
body { display: flex; flex-direction: column; min-height: 100vh; }

/* topbar */
#topbar {
  display: flex; align-items: center; gap: var(--gap-lg);
  padding: 0 16px; height: 40px;
  background: var(--bg-elev-1); border-bottom: 1px solid var(--border);
  font-weight: 600;
}
#topbar .logo { display: flex; align-items: center; gap: 8px; }
#topbar .logo .dot {
  width: 9px; height: 9px; border-radius: 50%;
  background: var(--accent); box-shadow: 0 0 8px var(--accent);
}
#topbar .sub { color: var(--text-muted); font-weight: 400; font-size: 11px; }
#topbar .conn { margin-left: auto; font-family: var(--font-mono); font-size: 11px;
  color: var(--text-muted); display: flex; align-items: center; gap: 6px; }
#topbar .conn .ind { width: 8px; height: 8px; border-radius: 50%; background: var(--status-err); }
#topbar .conn.ok .ind { background: var(--status-ok); }
#topbar .conn.conn .ind { background: var(--status-warn); }

/* main */
#main {
  flex: 1; padding: 16px; max-width: 920px; margin: 0 auto; width: 100%;
  display: flex; flex-direction: column; gap: var(--gap-lg);
}

.section-title {
  font-size: 11px; text-transform: uppercase; letter-spacing: 0.08em;
  color: var(--text-muted); margin: 0 2px 6px;
  display: flex; align-items: center; gap: 6px;
}
.section-title::before {
  content: ''; width: 7px; height: 7px; border-radius: 2px; background: var(--accent);
}

/* cards */
.card {
  background: var(--bg-elev-1); border: 1px solid var(--border);
  border-radius: var(--radius); padding: 12px 14px;
  border-left: 3px solid var(--text-muted);
}
.card.st-ok   { border-left-color: var(--status-ok); }
.card.st-warn { border-left-color: var(--status-warn); }
.card.st-err  { border-left-color: var(--status-err); }
.card.st-run  { border-left-color: var(--status-run); }
.card-head { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
.card-head .pill {
  font-size: 10px; text-transform: uppercase; letter-spacing: 0.05em;
  padding: 2px 7px; border-radius: 10px; background: var(--bg-elev-2);
  color: var(--text-muted);
}
.card-head .pill.ok { color: var(--status-ok); }
.card-head .pill.warn { color: var(--status-warn); }
.card-head .pill.err { color: var(--status-err); }
.card-head .title { font-weight: 600; font-size: 12px; flex: 1; }

/* kv grid */
.grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px 18px; }
.kv { display: flex; justify-content: space-between; gap: 8px;
  font-size: 12px; padding: 2px 0; border-bottom: 1px solid var(--border-soft); }
.kv:last-child { border-bottom: none; }
.kv .k { color: var(--text-muted); }
.kv .v { font-family: var(--font-mono); color: var(--text-value); text-align: right;
  white-space: nowrap; overflow: hidden; text-overflow: ellipsis; max-width: 60%; }
.kv .v.mono-sm { font-size: 11px; }

/* target box */
.target {
  background: var(--bg-inset); border-radius: var(--radius-sm);
  padding: 8px 10px; margin-top: 6px; font-family: var(--font-mono);
  font-size: 12px; color: var(--text-value);
}
.target .mode { display: inline-block; padding: 1px 6px; border-radius: 3px;
  margin-right: 6px; font-weight: 600; }
.target .mode.explore { background: rgba(78,201,176,0.15); color: var(--status-ok); }
.target .mode.stop    { background: rgba(244,135,113,0.15); color: var(--status-err); }
.target .mode.none    { background: var(--bg-elev-2); color: var(--text-muted); }

/* estop */
#estop {
  width: 100%; padding: 14px;
  background: var(--status-err); color: #1e1e1e;
  border: none; border-radius: var(--radius);
  font-size: 15px; font-weight: 700; letter-spacing: 0.05em;
  cursor: pointer; transition: filter .12s, transform .05s;
  text-transform: uppercase;
}
#estop:hover { filter: brightness(1.12); }
#estop:active { transform: translateY(1px); }
#estop:disabled { opacity: 0.4; cursor: default; }
.estop-wrap { padding: 4px 0; }
.estop-hint { color: var(--text-muted); font-size: 11px; text-align: center; margin-top: 6px; }

/* footer */
#footer {
  padding: 8px 16px; background: var(--bg-elev-1); border-top: 1px solid var(--border);
  font-family: var(--font-mono); font-size: 11px; color: var(--text-muted);
  display: flex; gap: 18px; justify-content: center;
}
#footer b { color: var(--text-value); font-weight: 500; }

/* toast */
#toast {
  position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%);
  padding: 8px 14px; border-radius: var(--radius);
  background: var(--bg-elev-2); border: 1px solid var(--border);
  font-size: 12px; opacity: 0; transition: opacity .2s; pointer-events: none;
}
#toast.show { opacity: 1; }
#toast.ok { border-color: var(--status-ok); color: var(--status-ok); }
#toast.err { border-color: var(--status-err); color: var(--status-err); }

@media (max-width: 640px) { .grid { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<div id="topbar">
  <div class="logo"><span class="dot"></span> m3v-agent <span class="sub">受控端状态面板</span></div>
  <div class="conn" id="conn"><span class="ind"></span><span id="conn-txt">connecting…</span></div>
</div>

<div id="main">
  <div>
    <div class="section-title">机器人</div>
    <div class="card" id="card-robot">
      <div class="card-head"><span class="title" id="robot-name">—</span><span class="pill" id="robot-driver">—</span></div>
      <div class="grid" id="robot-grid"></div>
    </div>
  </div>

  <div>
    <div class="section-title">录制 (FAST-LIO → ccenter 格式)</div>
    <div class="card" id="card-rec">
      <div class="card-head"><span class="title">Recorder</span><span class="pill" id="rec-pill">idle</span></div>
      <div class="grid" id="rec-grid"></div>
    </div>
  </div>

  <div>
    <div class="section-title">回传 (SCP → Windows)</div>
    <div class="card" id="card-push">
      <div class="card-head"><span class="title">Transport</span><span class="pill" id="push-pill">idle</span></div>
      <div class="grid" id="push-grid"></div>
    </div>
  </div>

  <div>
    <div class="section-title">执行 (目标文件 → 运动)</div>
    <div class="card" id="card-exec">
      <div class="card-head"><span class="title">Executor</span><span class="pill" id="exec-pill">idle</span></div>
      <div class="grid" id="exec-grid"></div>
      <div class="target" id="exec-target"><span class="mode none">no target</span></div>
    </div>
  </div>

  <div class="estop-wrap">
    <button id="estop">⛔ 紧急停止 (Emergency Stop)</button>
    <div class="estop-hint">driver.emergency_stop() — 趴下 + 电机阻尼</div>
  </div>
</div>

<div id="footer">
  <span>uptime <b id="ft-uptime">—</b></span>
  <span>updated <b id="ft-updated">—</b></span>
  <span>poll <b>1 Hz</b></span>
</div>
<div id="toast"></div>
<script src="/app.js"></script>
</body>
</html>
"""

_APP_JS = r"""// m3v-agent status panel — 1 Hz poll + render. Matches Multi3DViz tokens.
(function () {
  const $ = (id) => document.getElementById(id);
  const conn = $("conn"), connTxt = $("conn-txt");

  function setPill(el, text, cls) {
    el.textContent = text;
    el.className = "pill " + (cls || "");
  }
  function setCard(cardEl, state) {
    cardEl.className = "card " + (state ? "st-" + state : "");
  }
  function fmtPose(p) {
    if (!p || !p.x && p.x !== 0) return "—";
    return `(${p.x.toFixed(2)}, ${p.y.toFixed(2)}, yaw ${(p.yaw||0).toFixed(2)})`;
  }
  function kv(gridEl, pairs) {
    gridEl.innerHTML = pairs.map(([k, v, sm]) =>
      `<div class="kv"><span class="k">${k}</span>` +
      `<span class="v${sm ? " mono-sm" : ""}">${v == null ? "—" : v}</span></div>`
    ).join("");
  }

  async function poll() {
    try {
      const r = await fetch("/api/state", { cache: "no-store" });
      if (!r.ok) throw new Error("HTTP " + r.status);
      const s = await r.json();
      conn.className = "conn ok";
      connTxt.textContent = "live";
      render(s);
    } catch (e) {
      conn.className = "conn conn";
      connTxt.textContent = "agent offline (" + e.message + ")";
    }
  }

  function render(s) {
    // robot
    const rob = s.robot || {};
    $("robot-name").textContent = rob.label || rob.robot_id || "m3v-agent";
    $("robot-driver").textContent = (rob.driver_kind || "?") + (rob.ros ? " · " + rob.ros : "");
    setCard($("card-robot"), rob.driver_connected ? "ok" : "err");
    kv($("robot-grid"), [
      ["robot_id", rob.robot_id],
      ["host", rob.host],
      ["ros stack", rob.ros],
      ["driver", (rob.driver_connected ? "connected" : "disconnected") + (rob.standing ? " · standing" : "")],
      ["run dir", (s.recorder && s.recorder.run_dir || "—"), true],
      ["mode", s.mode],
    ]);

    // recorder
    const rec = s.recorder || {};
    const recFrames = rec.frame_idx != null ? rec.frame_idx : "—";
    setPill($("rec-pill"),
      rec.enabled ? (rec.frame_idx > 0 ? "recording" : "waiting") : "off",
      rec.enabled ? "ok" : "");
    setCard($("card-rec"), rec.enabled ? "run" : "");
    kv($("rec-grid"), [
      ["frames", recFrames],
      ["gravity", rec.gravity_ready ? "calibrated" : (rec.gravity_enabled ? "collecting…" : "off")],
      ["pose x,y,yaw", fmtPose(rec.latest_pose)],
      ["odom topic", rec.odom_topic],
      ["cloud topic", rec.cloud_topic],
      ["naming", rec.naming],
    ]);

    // transport
    const tr = s.transport || {};
    setPill($("push-pill"),
      tr.enabled ? (tr.connected ? "pushing" : "disconnected") : "off",
      tr.enabled ? (tr.connected ? "ok" : "err") : "");
    setCard($("card-push"), tr.enabled ? (tr.connected ? "ok" : "err") : "");
    kv($("push-grid"), [
      ["frames pushed", tr.pushed_idx != null ? tr.pushed_idx : "—"],
      ["target", tr.target_host ? (tr.target_user + "@" + tr.target_host) : "—"],
      ["remote root", tr.remote_root, true],
      ["last error", tr.last_error || "—", true],
    ]);

    // executor
    const ex = s.executor || {};
    const tgt = ex.current_target;
    setPill($("exec-pill"),
      ex.enabled ? (ex.halted_for_stale ? "halted" : (ex.nav_state || "idle")) : "off",
      ex.halted_for_stale ? "err" : (ex.nav_state === "arrived" ? "ok" : ""));
    setCard($("card-exec"), ex.enabled ? (ex.halted_for_stale ? "err" : "run") : "");
    kv($("exec-grid"), [
      ["nav state", ex.nav_state],
      ["target file", ex.target_path, true],
      ["file age", ex.file_age_s != null ? ex.file_age_s.toFixed(1) + " s" : "—"],
      ["arrive thresh", ex.arrive_threshold != null ? ex.arrive_threshold + " m" : "—"],
    ]);
    // target box
    const tbox = $("exec-target");
    if (!ex.enabled) {
      tbox.innerHTML = '<span class="mode none">executor off</span>';
    } else if (!tgt) {
      tbox.innerHTML = '<span class="mode none">no target</span>';
    } else if (tgt.mode === "stop") {
      tbox.innerHTML = '<span class="mode stop">stop</span> halted';
    } else {
      tbox.innerHTML =
        '<span class="mode explore">explore</span>' +
        `local (${tgt.local_x.toFixed(2)}, ${tgt.local_y.toFixed(2)})  ` +
        `global (${tgt.global_x.toFixed(2)}, ${tgt.global_y.toFixed(2)})  ` +
        `frame ${tgt.frame}`;
    }

    // footer
    const srv = s.server || {};
    $("ft-uptime").textContent = (srv.uptime_s != null ? srv.uptime_s.toFixed(0) + "s" : "—");
    $("ft-updated").textContent = new Date(srv.now * 1000 || Date.now()).toLocaleTimeString();
  }

  // estop
  $("estop").addEventListener("click", async () => {
    const btn = $("estop");
    if (!confirm("确认紧急停止？机器人会立即趴下（passive/prone）。")) return;
    btn.disabled = true;
    const old = btn.textContent;
    btn.textContent = "…";
    try {
      const r = await fetch("/api/estop", { method: "POST" });
      const j = await r.json();
      showToast(j.ok ? "已发送急停" : "急停失败", j.ok ? "ok" : "err");
    } catch (e) {
      showToast("急停请求失败: " + e.message, "err");
    } finally {
      btn.disabled = false;
      btn.textContent = old;
    }
  });

  let toastTimer;
  function showToast(msg, cls) {
    const t = $("toast");
    t.textContent = msg;
    t.className = "show " + (cls || "");
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { t.className = ""; }, 2500);
  }

  poll();
  setInterval(poll, 1000);
})();
"""
