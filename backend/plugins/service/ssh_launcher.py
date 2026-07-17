"""ssh_launcher.py — Service plugin: run SLAM (FAST-LIO) and arbitrary commands
on connected robots via SSH.

Responsibilities (does NOT do its own heartbeat/reconnect — that lives in
RobotManager, which owns each connection's liveness):
  1. On-demand launch/stop of a robot's `launch_cmd` (e.g. FAST-LIO) via the
     `launch`/`stop` WS actions (routed through the backend like playback).
  2. Auto-launch: when a robot transitions to online AND `auto_launch` is on,
     fire its launch_cmd once (guarded so reconnects don't re-launch).
  3. A generic `run_command` action for ad-hoc SSH (check processes, tail logs).

Commands run via the robot's persistent RobotManager connection (channels are
isolated, so concurrent commands across robots are safe). Launches are wrapped
in `nohup ... &` so they survive the SSH channel closing.
"""
from __future__ import annotations
import os
import sys
import json
import shlex
import logging

_THIS = os.path.dirname(os.path.abspath(__file__))
_BACKEND = os.path.abspath(os.path.join(_THIS, "..", ".."))
if _BACKEND not in sys.path:
    sys.path.insert(0, _BACKEND)

from core.plugin_base import ServicePlugin

log = logging.getLogger("multi3dviz.service.ssh_launcher")


class SSHLauncherService(ServicePlugin):
    name = "SSHLauncher"
    category = "service"
    description = "Launch/stop FAST-LIO and run SSH commands on connected robots."
    default_enabled = True

    properties = {
        "auto_launch": {
            "type": "bool", "default": False,
            "label": "Auto-launch SLAM when a robot connects",
            "group": "Behavior",
        },
        "launch_prefix": {
            "type": "string", "default": "",
            "label": "Prefix prepended to launch_cmd (workspace setup)",
            "group": "Behavior",
        },
    }

    # Per-robot pipeline scripts (SSH-orchestrated, no agent needed on robot).
    # These match the verified scripts on the Unitree Go2. Override launch_cmd
    # per-robot from the UI for other platforms.
    PIPELINE_SCRIPTS = {
        "robot_a": "/home/unitree/sda2/restart_all.sh",     # FAST-LIO + record + bridge
        # Agibot: FAST-LIO runs in a ROS1 noetic Docker container. The pipeline
        # script (pipeline.sh) is exec'd INSIDE the container via docker exec.
        # See _launch / _stop for the container wrapping.
        "robot_b": "/scripts/m3v_agent/pipeline.sh",
    }
    # Agibot container: the noetic-fastlio image where FAST-LIO + the recorder
    # run. Started on demand by _launch if it's stopped.
    AGIBOT_CONTAINER = "fastlio_noetic"
    # Per-robot cleanup scripts (stop the pipeline).
    CLEANUP_SCRIPTS = {
        "robot_a": "/home/unitree/sda2/cleanup_ros.sh",
        "robot_b": "/scripts/m3v_agent/cleanup.sh",
    }
    EXPLORER_SCRIPTS = {
        "robot_a": "/home/unitree/sda2/run_go2_search.sh",
        "robot_b": "/home/orin-001/sda2/run_go2_search.sh",
    }

    def __init__(self, ctx):
        super().__init__(ctx)
        # robot_id -> True once we've launched on it (so reconnects don't
        # re-launch unless the user explicitly stops+launches again).
        self._launched = set()
        self._standing = {}   # robot_id → bool (tracks stand/lie state for toggle)
        self._sport_chan = {} # robot_id → paramiko.Channel (persistent a2_sport_client)
        self._vel_sock = {}   # robot_id → TCP socket (m3v_vel_server, NO SSH)
        self._vel_locks = {}  # robot_id → threading.Lock (prevent write interleave)
        self._launch_locks = {}  # robot_id → threading.Lock (prevent duplicate launch)

    def _get_vel_sock(self, rid, conn):
        """Get or create a TCP socket to the robot's m3v_vel_server (port 7777).
        The socket is kept open across calls — no health-check probes (sending
        0 bytes can trigger a disconnect on some OSes). Dead sockets are
        detected lazily when sendall fails.

        CRITICAL: connect timeout is 0.3s and failures are cached for 10s so
        we don't block the event loop with repeated slow connect attempts."""
        if rid in self._vel_sock:
            return self._vel_sock[rid]  # reuse — no probe
        if not hasattr(self, "_vel_fail_ts"):
            self._vel_fail_ts = {}
        import time as _t
        if rid in self._vel_fail_ts and _t.monotonic() - self._vel_fail_ts[rid] < 10:
            return None  # recently failed — don't retry for 10s
        host = conn.cfg.host if (conn and conn.cfg) else None
        if not host:
            # Fallback: try to find the host from the robot's config in the
            # default robots list (DEFAULT_ROBOTS in main.py).
            _DEFAULT_HOSTS = {"robot_a": "10.60.77.187", "robot_b": "10.60.77.154"}
            host = _DEFAULT_HOSTS.get(rid)
        if not host:
            return None
        import socket as _sock
        try:
            s = _sock.socket(_sock.AF_INET, _sock.SOCK_STREAM)
            s.settimeout(0.3)
            s.connect((host, 7777))
            s.settimeout(1.0)
            self._vel_sock[rid] = s
            self._vel_fail_ts.pop(rid, None)
            log.info("vel TCP socket connected to %s:7777", host)
            return s
        except Exception:
            self._vel_fail_ts[rid] = _t.monotonic()
            return None

    def on_enable(self):
        log.info("SSHLauncher ready")

    # --- WS-driven actions (backend routes 'robot_command' messages here) ---
    def command(self, robot_id, action, value=None):
        """action: 'launch' | 'stop' | 'restart' | 'estop' | 'vel' | 'run'.
        - launch/stop/restart: manage FAST-LIO SLAM on the robot.
        - estop: emergency stop — SSH-sends a stop command to the robot's
          motion controller (Unitree TCP bridge / Agibot /agibot_cmd).
        - vel: keyboard takeover — value={vx,vy,yaw} sent as a one-shot
          velocity command to the robot's motion controller via SSH.
        - run: generic ad-hoc SSH command."""
        log.info("command received: robot=%s action=%s", robot_id, action)
        conn = self.ctx.robots.get(robot_id) if self.ctx.robots else None
        if conn is None and action != "vel":
            log.warning("command %s: robot %s not found", action, robot_id)
            return {"ok": False, "error": "robot not found"}
        if action == "vel":
            # vel can work without SSH (TCP vel_server) — conn may be None.
            if conn is None:
                conn = type("C", (), {"cfg": type("Cfg", (), {
                    "robot_id": robot_id,
                    "host": "10.60.77.187" if robot_id == "robot_a" else "10.60.77.154"})()})()
            return self._send_vel(conn, value)
        if action == "launch":
            return self._launch(conn)
        if action == "stop":
            return self._stop(conn)
        if action == "restart":
            self._stop(conn)
            return self._launch(conn)
        if action == "estop":
            return self._estop(conn)
        if action == "toggle_pose":
            return self._toggle_pose(conn)
        if action == "takeover_start":
            return self._takeover_start(conn)
        if action == "takeover_end":
            return self._takeover_end(conn)
        if action == "channel_status":
            rid2 = conn.cfg.robot_id
            # Ready if EITHER the SSH sport channel OR the TCP vel_server is up.
            ssh_ok = self._channel_alive(rid2)
            tcp_ok = self._get_vel_sock(rid2, conn) is not None
            return {"ok": True, "ready": ssh_ok or tcp_ok}
        if action == "stand_up":
            return self._stand_up(conn)
        if action == "lie_down":
            return self._lie_down(conn)
        if action == "vel":
            return self._send_vel(conn, value)
        if action == "run":
            rc, out = conn.run(str(value or ""))
            return {"ok": rc == 0, "rc": rc, "output": out[-2000:]}
        if action == "battery":
            return self._query_battery(conn)
        if action == "launch_explorer":
            return self._launch_explorer(conn)
        if action == "explore_execute":
            return self._explore_execute(conn, value)
        return {"ok": False, "error": f"unknown action {action}"}

    def _query_battery(self, conn):
        """Query robot battery level via SSH. Returns {ok, pct, raw}."""
        import re as _re
        rid = conn.cfg.robot_id
        # Unitree Go2: battery_status_bar outputs "🔋 XX%".
        # Agibot: grep Bms power from dog_task.log.
        cmds = [
            # Unitree battery_status_bar (fast, ~2s)
            "timeout 5 /home/unitree/unitree_sdk2-main/build/bin/battery_status_bar eth0 -1 2>&1 | head -3",
            # Agibot Bms power from log
            "grep 'Bms power' /userdata/log/dog_task.log 2>/dev/null | tail -1",
        ]
        for cmd in cmds:
            rc, out = conn.run(cmd, timeout=8)
            if rc != 0 and not out:
                continue
            # Extract percentage.
            m = _re.search(r'(\d+)\s*%', out)
            if m:
                pct = int(m.group(1))
                log.info("battery %s: %d%%", rid, pct)
                return {"ok": True, "pct": pct, "raw": out.strip()[-100:]}
        return {"ok": False, "pct": -1, "raw": out.strip()[-100:] if out else ""}

    def _launch_explorer(self, conn):
        """Launch the frontier exploration script (go2_search) via SSH."""
        rid = conn.cfg.robot_id
        script = self.EXPLORER_SCRIPTS.get(rid, "")
        if not script:
            return {"ok": False, "error": "no explorer script for this robot"}
        full = f"nohup bash -lc {shlex.quote(script)} > /tmp/m3v_explore.log 2>&1 &"
        rc, out = conn.run(full, timeout=15)
        if rc == 0:
            log.info("launched explorer on %s: %s", rid, script)
            return {"ok": True}
        return {"ok": False, "rc": rc, "output": out[-500:]}

    def _launch(self, conn):
        """Launch the robot's full SLAM pipeline (FAST-LIO + record + bridge)
        via SSH. Uses launch_cmd if set, otherwise falls back to the known
        pipeline script for this robot_id.

        LOCK: acquires a per-robot lock so rapid double-clicks on the launch
        button don't spawn two restart_all.sh instances that kill each other
        (the script's cleanup step murders the sibling's processes).

        For the Agibot, the pipeline runs INSIDE a ROS1 noetic Docker container
        (fastlio_noetic). We ensure the container is running, then docker exec
        the pipeline script inside it. The recorder (rospy) + FAST-LIO share the
        container's host-network roscore."""
        rid = conn.cfg.robot_id
        # Per-robot lock: if a launch is already in progress, reject immediately.
        import threading as _th
        if rid not in self._launch_locks:
            self._launch_locks[rid] = _th.Lock()
        if not self._launch_locks[rid].acquire(blocking=False):
            log.warning("launch %s skipped: already in progress", rid)
            return {"ok": False, "error": "launch already in progress, please wait..."}
        try:
            return self._launch_inner(conn)
        finally:
            self._launch_locks[rid].release()

    def _launch_inner(self, conn):
        rid = conn.cfg.robot_id
        cmd = conn.cfg.launch_cmd or self.PIPELINE_SCRIPTS.get(rid, "")
        if not cmd:
            return {"ok": False, "error": "no launch_cmd or pipeline script for this robot"}
        # Agibot: FAST-LIO is in a Docker container — exec the script inside it.
        if rid == "robot_b" and self.AGIBOT_CONTAINER:
            cname = self.AGIBOT_CONTAINER
            # Start the container if it's stopped (idempotent).
            conn.run(f"docker start {cname} 2>/dev/null || true", timeout=15)
            # Kill any stale pipeline inside the container first, then launch.
            # docker exec -d detaches so the pipeline survives the SSH channel.
            cleanup = self.CLEANUP_SCRIPTS.get(rid, "")
            if cleanup:
                conn.run(f"docker exec {cname} bash {cleanup} 2>/dev/null || true", timeout=15)
            full = (f"docker exec -d {cname} bash {shlex.quote(cmd)} "
                    f"> /tmp/m3v_pipeline.log 2>&1")
            rc, out = conn.run(full, timeout=20)
            if rc == 0:
                self._launched.add(rid)
                log.info("launched Agibot pipeline in container %s: %s", cname, cmd)
                return {"ok": True}
            return {"ok": False, "rc": rc, "output": out[-500:]}
        # Unitree / generic: run the pipeline script directly on the host.
        prefix = self.get("launch_prefix", "")
        full_cmd = f"{prefix} {cmd}".strip() if prefix else cmd
        # Run SYNCHRONOUSLY (not nohup &) so we capture the exit code + output.
        # The script itself backgrounds each component with nohup internally,
        # so it returns ~30s after everything is up. We use a generous timeout.
        log.info("launching pipeline on %s: %s (waiting for completion...)", rid, full_cmd)
        full = f"bash -lc {shlex.quote(full_cmd)} 2>&1"
        rc, out = conn.run(full, timeout=90)
        out_text = out.decode(errors='replace') if isinstance(out, bytes) else str(out)
        if rc == 0:
            self._launched.add(rid)
            # Parse the script output for success/failure markers.
            steps_ok = out_text.count("✅")
            steps_fail = out_text.count("❌")
            steps_warn = out_text.count("⚠️")
            msg = f"pipeline started ({steps_ok} ok"
            if steps_warn:
                msg += f", {steps_warn} warn"
            if steps_fail:
                msg += f", {steps_fail} FAILED"
            msg += ")"
            log.info("launched pipeline on %s: %s", rid, msg)
            return {"ok": True, "message": msg, "output": out_text[-800:]}
        # Failed: extract the failure line for the debug console.
        fail_line = ""
        for line in out_text.split('\n'):
            if '❌' in line or 'fail' in line.lower():
                fail_line = line.strip()
                break
        err_msg = fail_line or out_text[-400:] or f"exit code {rc}"
        log.warning("launch %s FAILED: %s", rid, err_msg)
        return {"ok": False, "rc": rc, "error": err_msg, "output": out_text[-800:]}

    def _stop(self, conn):
        """Stop the robot's SLAM pipeline. Uses the cleanup script if known
        (kills all ROS/livox/fastlio processes), otherwise falls back to pkill.

        For the Agibot, the cleanup script runs INSIDE the container via
        docker exec (mirroring _launch)."""
        rid = conn.cfg.robot_id
        cleanup = self.CLEANUP_SCRIPTS.get(rid, "")
        if cleanup:
            if rid == "robot_b" and self.AGIBOT_CONTAINER:
                # Agibot: cleanup inside the container.
                conn.run(f"docker exec {self.AGIBOT_CONTAINER} bash {cleanup} "
                         f"2>/dev/null || true", timeout=15)
            else:
                conn.run(f"bash {cleanup}", timeout=10)
        else:
            # Fallback: kill by launch_cmd's first token.
            cmd = conn.cfg.launch_cmd.split()[0] if conn.cfg.launch_cmd else ""
            if cmd:
                conn.run(f"pkill -f {shlex.quote(cmd)}", timeout=8)
        self._launched.discard(rid)
        log.info("stopped pipeline on %s", rid)
        return {"ok": True}

    def _go2_cmd(self, conn, cmd_id, extra_args=""):
        """Send a motion command to the Go2 via DDS (unitree_sdk2 a2_sport_client).
        Direct DDS connection to the Go2 motion controller at 192.168.123.222
        via eth0 — no TCP bridge (go2_bridge_ros2.py) needed.

        cmd_id maps to a2_sport_client's menu:
          0=damp(急停阻尼) 1=balance_stand 2=stop_move 3=stand_down(趴下)
          4=recovery_stand 5=move(vx,vy,yaw) 11=stand_up(站立)
        extra_args: for cmd 5 (move), pass "vx vy yaw" as space-separated floats.
        Returns True if the command was accepted (code 0)."""
        rid = conn.cfg.robot_id
        args = f"eth0 {extra_args}".strip() if extra_args else "eth0"
        ssh_cmd = f"echo '{cmd_id}' | timeout 5 /home/unitree/unitree_sdk2-main/build/bin/a2_sport_client {args} 2>&1"
        rc, out = conn.run(ssh_cmd, timeout=8)
        ok = "Request successed" in out or "code: 0" in out
        log.info("go2_cmd %s id=%d: %s (%s)", rid, cmd_id, "OK" if ok else "FAIL", out[-80:])
        return ok

    def _tcp_bridge_cmd(self, conn, api_id, parameter=None):
        """Legacy TCP bridge command (localhost:21520). Kept as fallback for
        robots that run go2_bridge_ros2.py. New deployments use _go2_cmd (DDS
        direct) which doesn't require the bridge process to be running."""
        param_str = json.dumps(parameter or {})
        py = (
            "python3 -c \""
            "import socket,json;"
            "try:{"
            "s=socket.socket();s.settimeout(3);s.connect(('127.0.0.1',21520));"
            f"s.sendall(json.dumps({{'api_id':{api_id},'parameter':{param_str}}})+b'\\n');"
            "s.close()"
            "}except:pass"
            "\" 2>/dev/null"
        )
        rc, out = conn.run(py, timeout=6)
        return rc == 0

    def _estop(self, conn):
        """Emergency stop. Uses sport channel if open (works for both Go2 +
        Agibot), falls back to Go2 DDS exec. Sends: zero vel → stop → damp."""
        rid = conn.cfg.robot_id
        log.warning("ESTOP on %s", rid)
        # Zero velocity first (via channel or SSH exec).
        self._send_vel(conn, {"vx": 0, "vy": 0, "yaw": 0})
        chan = self._sport_chan.get(rid)
        if chan is not None:
            try:
                chan.sendall(b"stop\n")
                # Agibot uses 'passive', Go2 uses 'damp' — both scripts accept
                # 'stop' as full halt. The m3v_move/m3v_agibot watchdog + stdin
                # protocol handles the rest.
                chan.sendall(b"lie\n")
            except Exception:
                pass
        else:
            # Fallback: Go2 DDS direct (a2_sport_client).
            self._go2_cmd(conn, 2)   # stop_move
            self._go2_cmd(conn, 0)   # damp
        self._standing[rid] = False
        return {"ok": True}

    def _toggle_pose(self, conn):
        """Toggle stand/lie. Priority: TCP vel_server → SSH sport channel."""
        rid = conn.cfg.robot_id
        standing = self._standing.get(rid, False)
        cmd = "lie" if standing else "stand"
        # Try TCP vel_server first.
        sock = self._get_vel_sock(rid, conn)
        if sock is not None:
            try:
                sock.sendall((cmd + "\n").encode())
                self._standing[rid] = not standing
                log.info("toggle_pose %s: %s (TCP)", rid, cmd)
                return {"ok": True, "standing": self._standing[rid]}
            except Exception:
                self._vel_sock.pop(rid, None)
        # Fallback: SSH sport channel.
        chan = self._sport_chan.get(rid) if self._channel_alive(rid) else None
        if chan is None:
            return {"ok": False, "error": "no vel channel (TCP or SSH)"}
        try:
            chan.sendall((cmd + "\n").encode())
            self._standing[rid] = not standing
            log.info("toggle_pose %s: %s (SSH)", rid, cmd)
            return {"ok": True, "standing": self._standing[rid]}
        except Exception as e:
            self._sport_chan[rid] = None
            return {"ok": False, "error": str(e)}

    def _stand_up(self, conn):
        """Stand the dog up using recovery_stand (cmd 4) — the only command
        that works from PASSIVE mode. cmd 11 (stand_up) fails when PASSIVE."""
        rid = conn.cfg.robot_id
        log.info("STANDUP on %s (recovery_stand)", rid)
        ok = self._go2_cmd(conn, 4)   # recovery_stand — works from PASSIVE
        self._standing[rid] = True
        return {"ok": ok}

    def _lie_down(self, conn):
        """Lie the dog down (cmd 3 = stand_down). Safe resting posture."""
        rid = conn.cfg.robot_id
        log.info("LIEDOWN on %s", rid)
        ok = self._go2_cmd(conn, 3)
        self._standing[rid] = False
        return {"ok": ok}

    def _send_vel(self, conn, value):
        """Send velocity to the robot. Priority:
        1. TCP socket to m3v_vel_server (port 7777) — NO SSH, zero contention.
        2. SSH sport channel (fallback if vel_server not running).
        NO one-shot SSH exec (too slow, causes lock contention)."""
        if not isinstance(value, dict):
            return {"ok": False, "error": "vel value must be {vx,vy,yaw}"}
        vx = float(value.get("vx", 0))
        vy = float(value.get("vy", 0))
        yaw = float(value.get("yaw", 0))
        rid = conn.cfg.robot_id
        # Per-robot lock so explore + takeover don't interleave writes.
        if rid not in self._vel_locks:
            import threading as _tl
            self._vel_locks[rid] = _tl.Lock()
        cmd = f"{vx} {vy} {yaw}\n".encode()
        with self._vel_locks[rid]:
            # 1. Try TCP socket (NO SSH).
            sock = self._get_vel_sock(rid, conn)
            if sock is not None:
                try:
                    sock.sendall(cmd)
                    log.info("VEL TCP sent %s: %s", rid, cmd.decode().strip())
                    return {"ok": True}
                except Exception as e:
                    log.warning("VEL TCP send failed %s: %s", rid, e)
                    self._vel_sock.pop(rid, None)
            # 2. Fallback: SSH sport channel (still no conn.run).
            chan = self._sport_chan.get(rid) if self._channel_alive(rid) else None
            if chan is not None:
                try:
                    chan.sendall(cmd)
                    return {"ok": True}
                except Exception:
                    return {"ok": False, "error": "send failed"}
        return {"ok": False, "error": "no vel channel (TCP or SSH)"}

    # --- safe step-by-step exploration navigation ---
    # Reference: go2_search.py's move_forward_with_feedback (MAX_MOVE_DURATION_S=2.0).
    # The control end drives the robot step-by-step toward a frontier target:
    #   1. Read latest odom from the data bus (robot position + heading).
    #   2. Turn toward the target (angular P-control, max 2s per turn burst).
    #   3. Move forward (fixed speed, MAX 2s per burst — safety: don't walk
    #      long enough to hit a wall between odom checks).
    #   4. Re-read odom, check arrival (dist < threshold). Repeat until arrived
    #      or max_steps exhausted.
    # All velocity goes through the persistent sport channel (same as takeover).
    EXPLORE_MOVE_SPEED = 0.4       # m/s forward
    EXPLORE_TURN_SPEED = 1.3       # rad/s yaw (faster turning)
    EXPLORE_ANGLE_THRESH = 0.15    # rad (~8.5°) — "facing target" tolerance
    EXPLORE_ARRIVE_THRESH = 0.3    # m — arrived when dist < this
    EXPLORE_STEP_TIME = 2.0        # s — max walk/turn per step (safety boundary)
    EXPLORE_MAX_STEPS = 50         # total steps before giving up
    EXPLORE_TURN_KP = 1.5          # proportional gain for angular control
    EXPLORE_WALK_DIST = 0.5        # m — target distance per walk step

    def _explore_execute(self, conn, value):
        """Interactive step-by-step exploration. Each call does ONE step:
          - mode "start": open channel, stand up, read initial state, return
            step-1 plan for the user to confirm with Enter.
          - mode "step": execute the next planned step (turn or walk 0.5m /
            ≤2s), read updated odom, return the next step's plan.
          - mode "stop": abort, stop robot, save snapshot.

        Returns a status dict the frontend shows + waits for Enter:
          {ok, step, action: "TURN"|"WALK"|"ARRIVED"|"DONE",
           target: [x,y], pos: [x,y,yaw_deg], dist, angle_err_deg,
           need_confirm: true}
        """
        import math as _math
        import time as _time
        rid = conn.cfg.robot_id
        if not isinstance(value, dict):
            value = {}
        mode = value.get("mode", "start")

        # Per-robot explore session state (persists across calls).
        if not hasattr(self, "_explore_sess"):
            self._explore_sess = {}
        sess = self._explore_sess.get(rid, {})

        if mode == "stop":
            self._send_vel(conn, {"vx": 0, "vy": 0, "yaw": 0})
            self._explore_busy[rid] = False
            self._save_exploration_snapshot(rid)
            self._explore_sess.pop(rid, None)
            return {"ok": True, "action": "STOPPED", "step": 0}

        if mode == "start":
            # Initialize a new explore session.
            target = value.get("target")
            if not target or len(target) < 2:
                return {"ok": False, "error": "explore_execute start needs {target: [x,y]}"}
            tx, ty = float(target[0]), float(target[1])
            sess = {"tx": tx, "ty": ty, "step": 0, "last_dist": float("inf"),
                    "stuck_count": 0}
            self._explore_sess[rid] = sess
            self._explore_busy[rid] = True
            # Open channel + stand up (blocking — runs in executor).
            if not self._channel_alive(rid):
                log.info("explore %s: opening sport channel...", rid)
                self.open_sport_channel(conn)
                _time.sleep(6)
            if not self._channel_alive(rid):
                self._explore_busy[rid] = False
                return {"ok": False, "error": "sport channel not ready"}
            chan = self._sport_chan.get(rid)
            if chan and not self._standing.get(rid, False):
                log.info("explore %s: standing up...", rid)
                try:
                    chan.sendall(b"stand\n")
                    self._standing[rid] = True
                except Exception:
                    pass
                _time.sleep(3.0)
            # Read initial state + plan first step.
            return self._explore_plan_step(conn, rid, sess)

        # mode == "step": execute the current plan, then plan the next.
        if not sess:
            return {"ok": False, "error": "no active explore session (call mode=start first)"}
        # Execute one step.
        result = self._explore_do_step(conn, rid, sess)
        if result.get("action") in ("ARRIVED", "STUCK", "MAXSTEPS"):
            # Run finished — stop + save snapshot.
            self._send_vel(conn, {"vx": 0, "vy": 0, "yaw": 0})
            self._explore_busy[rid] = False
            self._save_exploration_snapshot(rid)
            self._explore_sess.pop(rid, None)
            return result
        # Plan the next step for the user to confirm.
        return self._explore_plan_step(conn, rid, sess)

    def _explore_plan_step(self, conn, rid, sess):
        """Read odom, compute the next step's action, return status for the
        user to confirm with Enter. Does NOT move the robot."""
        import math as _math
        tx, ty = sess["tx"], sess["ty"]
        frame = self.ctx.data.latest(rid)
        if frame is None or not frame.get("odom"):
            return {"ok": False, "error": "no odom", "step": sess["step"],
                    "action": "ERROR", "need_confirm": False}
        odom = frame["odom"]
        ox, oy = float(odom.get("x", 0)), float(odom.get("y", 0))
        qx = float(odom.get("qx", 0)); qy = float(odom.get("qy", 0))
        qz = float(odom.get("qz", 0)); qw = float(odom.get("qw", 1))
        oyaw = _math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        dx, dy = tx - ox, ty - oy
        dist = _math.hypot(dx, dy)
        target_angle = _math.atan2(dy, dx)
        angle_err = _math.atan2(_math.sin(target_angle - oyaw),
                                _math.cos(target_angle - oyaw))
        # Store for do_step.
        sess["last_ox"] = ox; sess["last_oy"] = oy
        sess["last_dist"] = dist; sess["last_angle_err"] = angle_err
        if dist < self.EXPLORE_ARRIVE_THRESH:
            return {"ok": True, "action": "ARRIVED", "step": sess["step"],
                    "target": [round(tx,2), round(ty,2)],
                    "pos": [round(ox,2), round(oy,2), round(_math.degrees(oyaw),1)],
                    "dist": round(dist,2), "need_confirm": False}
        action = "TURN" if abs(angle_err) > self.EXPLORE_ANGLE_THRESH else "WALK"
        return {"ok": True, "action": action, "step": sess["step"] + 1,
                "target": [round(tx,2), round(ty,2)],
                "pos": [round(ox,2), round(oy,2), round(_math.degrees(oyaw),1)],
                "dist": round(dist,2),
                "angle_err": round(_math.degrees(angle_err),1),
                "need_confirm": True}

    def _explore_do_step(self, conn, rid, sess):
        """Execute ONE movement step (turn toward target, or walk 0.5m / ≤2s).
        Updates sess['step']. Returns the post-step status."""
        import math as _math
        import time as _time
        tx, ty = sess["tx"], sess["ty"]
        angle_err = sess.get("last_angle_err", 0)
        dist = sess.get("last_dist", 0)
        ox = sess.get("last_ox", 0); oy = sess.get("last_oy", 0)
        sess["step"] += 1
        step = sess["step"]
        if abs(angle_err) > self.EXPLORE_ANGLE_THRESH:
            # TURN: rotate toward target at most EXPLORE_STEP_TIME.
            yaw_vel = self.EXPLORE_TURN_KP * angle_err
            yaw_vel = max(-self.EXPLORE_TURN_SPEED, min(self.EXPLORE_TURN_SPEED, yaw_vel))
            log.info("explore %s step %d/%d: TURN %.2f rad/s (err %.0f°) dist %.1fm",
                     rid, step, self.EXPLORE_MAX_STEPS, yaw_vel,
                     _math.degrees(angle_err), dist)
            self._send_vel(conn, {"vx": 0, "vy": 0, "yaw": yaw_vel})
        else:
            # WALK: forward at MOVE_SPEED for at most the time it takes to
            # cover EXPLORE_WALK_DIST (0.5m), capped at EXPLORE_STEP_TIME (2s).
            walk_time = min(self.EXPLORE_WALK_DIST / self.EXPLORE_MOVE_SPEED,
                            self.EXPLORE_STEP_TIME)
            log.info("explore %s step %d/%d: WALK %.1f m/s for %.1fs (dist %.1fm)",
                     rid, step, self.EXPLORE_MAX_STEPS,
                     self.EXPLORE_MOVE_SPEED, walk_time, dist)
            self._send_vel(conn, {"vx": self.EXPLORE_MOVE_SPEED, "vy": 0, "yaw": 0})
            # Walk for walk_time, checking arrival every 0.1s.
            step_start = _time.time()
            while _time.time() - step_start < walk_time:
                _time.sleep(0.1)
                f2 = self.ctx.data.latest(rid)
                if f2 and f2.get("odom"):
                    o2 = f2["odom"]
                    d2 = _math.hypot(tx - float(o2.get("x", 0)),
                                     ty - float(o2.get("y", 0)))
                    if d2 < self.EXPLORE_ARRIVE_THRESH:
                        self._send_vel(conn, {"vx": 0, "vy": 0, "yaw": 0})
                        return {"ok": True, "action": "ARRIVED", "step": step,
                                "dist": round(d2, 2), "need_confirm": False}
            return {"ok": True, "action": "WALKED", "step": step,
                    "need_confirm": False}
        # For TURN steps, wait EXPLORE_STEP_TIME then stop.
        _time.sleep(self.EXPLORE_STEP_TIME)
        self._send_vel(conn, {"vx": 0, "vy": 0, "yaw": 0})
        # Check max steps + stuck.
        if step >= self.EXPLORE_MAX_STEPS:
            return {"ok": True, "action": "MAXSTEPS", "step": step, "need_confirm": False}
        return {"ok": True, "action": "TURNED", "step": step, "need_confirm": False}

    def _save_exploration_snapshot(self, rid):
        """Save a ccenter-style trajectory PNG after each exploration run.
        Uses the explorer's gmap + trails + coverage + targets. Output goes to
        output/ next to the executable (frozen-safe via sys.executable)."""
        import os as _os
        import sys as _sys
        import time as _time
        try:
            ex = getattr(self.ctx, "explorer_ref", None)
            if ex is None or getattr(ex, "_gmap", None) is None or \
                    getattr(ex, "_explorer", None) is None:
                return
            from lib.trajectory_plot import save_trajectory_figure
            # In a PyInstaller frozen build, __file__ points inside the
            # temporary _MEIPASS extraction dir (deleted on exit). Use
            # sys.executable's directory instead so files survive.
            if getattr(_sys, "frozen", False):
                base = _os.path.dirname(_sys.executable)
            else:
                base = _os.path.dirname(_os.path.dirname(_os.path.dirname(
                    _os.path.abspath(__file__))))
            out_dir = _os.path.join(base, "output")
            _os.makedirs(out_dir, exist_ok=True)
            ts = _time.strftime("%Y%m%d_%H%M%S")
            path = _os.path.join(out_dir, f"explore_{ts}_{rid}.png")
            # Build robots arg (single-robot: only the active trail has data).
            robots = [
                {"name": "Robot A", "trail": ex._traj_a, "color": "#E74C3C"},
                {"name": "Robot B", "trail": ex._traj_b, "color": "#3498DB"},
            ]
            # Targets as (wx, wy, color, label) tuples.
            targets = []
            e = ex._explorer
            target_colors = [("#E74C3C", "A"), ("#3498DB", "B")]
            for i in (0, 1):
                if e.targets[i] is not None:
                    gy, gx = e.targets[i]
                    wx, wy = e.grid_to_world(gy, gx)
                    tc, tl = target_colors[i]
                    targets.append((wx, wy, tc, tl))
            coverage = e.explored if e.explored is not None else None
            result = save_trajectory_figure(ex._gmap, robots, path,
                                            coverage_mask=coverage, targets=targets)
            if result:
                log.info("exploration snapshot saved: %s", result)
        except Exception:
            log.warning("exploration snapshot save failed", exc_info=True)

    def _takeover_start(self, conn):
        """Called when user enters keyboard takeover mode.
        PRIORITY: if the TCP vel_server (port 7777) is reachable, use it — no
        SSH channel needed, zero contention. Otherwise fall back to opening the
        SSH sport channel in a background thread."""
        rid = conn.cfg.robot_id
        # Try TCP vel_server first — if reachable, takeover is instant.
        sock = self._get_vel_sock(rid, conn)
        if sock is not None:
            log.info("takeover %s: using TCP vel_server (7777), no SSH channel needed", rid)
            return {"ok": True, "msg": "TCP vel_server ready"}
        # No TCP server — fall back to SSH sport channel (background, slow).
        if rid in self._sport_chan and self._sport_chan[rid] is not None:
            return {"ok": True, "msg": "channel already open"}
        # Open channel in background — don't block the WS response.
        import threading as _th
        _th.Thread(target=self.open_sport_channel, args=(conn,), daemon=True,
                   name=f"sport-chan-{rid}").start()
        return {"ok": True, "msg": "m3v_move starting (safe mode, dog not moving)"}

    def _takeover_end(self, conn):
        """Called when user exits takeover mode. Closes the m3v_move channel
        (which sends stand_down on stdin close → dog lies down)."""
        rid = conn.cfg.robot_id
        self.close_sport_channel(rid)
        return {"ok": True, "msg": "m3v_move closed, dog lying down"}

    def open_sport_channel(self, conn):
        """Open persistent sport controller on robot. Picks the right binary
        per robot type:
        - robot_a (Unitree Go2): /home/unitree/m3v_move (C++ DDS binary)
        - robot_b (Agibot D1):   /home/orin-001/m3v_agibot (C++ mc_sdk binary)
        Both have the same stdin protocol: 'stand'/'lie'/'stop'/'passive'/'vx vy yaw'
        SAFE MODE: no auto-stand, watchdog 500ms, stdin close → safe shutdown."""
        rid = conn.cfg.robot_id
        if rid in self._sport_chan and self._sport_chan[rid] is not None:
            return True
        if conn.client is None:
            return False
        # Kill any stale sport-controller process from a previous session
        # before opening a new one. A leftover m3v_agibot/m3v_move holds the
        # UDP port (Agibot) / DDS topic (Unitree), causing the new process to
        # fail "bind: Address already in use" and never reach checkConnect().
        if rid.endswith('b') or rid.endswith('B'):
            conn.run("pkill -f m3v_agibot", timeout=4)
        else:
            conn.run("pkill -f m3v_move", timeout=4)
        import time as _t; _t.sleep(0.5)  # let the port free up
        # Pick the right command per robot.
        if rid.endswith('b') or rid.endswith('B'):
            # Agibot: C++ binary (mc_sdk C++ API — verified path per control
            # doc §9.1; Python binding has known issues §8.5).
            # Uses the same stdin protocol as m3v_move.
            cmd = "/home/orin-001/m3v_agibot"
            init_wait = 8.0  # internal checkConnect() loop can take up to 6s
        else:
            # Unitree Go2: C++ binary via DDS
            cmd = "/home/unitree/m3v_move eth0"
            init_wait = 4.0  # DDS init
        try:
            chan = conn.client.get_transport().open_session()
            chan.get_pty()
            chan.exec_command(cmd)
            chan.settimeout(5)
            import time as _t; _t.sleep(init_wait)
            while chan.recv_ready():
                chan.recv(4096)
            # After the init wait, confirm the process is still running. If
            # it died during init (e.g. m3v_agibot.py crashed with a
            # SyntaxError, or the SDK couldn't connect), report failure so the
            # takeover UI shows an error instead of "ready".
            if chan.exit_status_ready():
                code = chan.recv_exit_status()
                tail = ""
                try:
                    while chan.recv_ready():
                        tail += chan.recv(4096).decode("utf-8", "replace")
                except Exception:
                    pass
                log.warning("open_sport_channel %s: process exited early "
                            "(code=%d) cmd=%s tail=%s",
                            rid, code, cmd, tail[-300:])
                self._sport_chan[rid] = None
                return False
            self._sport_chan[rid] = chan
            self._standing[rid] = False  # safe mode: not standing
            log.info("sport channel opened for %s via: %s", rid, cmd)
            return True
        except Exception as e:
            log.warning("open_sport_channel %s failed: %s", rid, e)
            self._sport_chan[rid] = None
            return False

    def close_sport_channel(self, rid):
        """Close the persistent sport client (on takeover release)."""
        chan = self._sport_chan.pop(rid, None)
        if chan is not None:
            try:
                chan.sendall(b"3\n")  # stand_down (lie down safely)
                chan.close()
            except Exception:
                pass
            log.info("sport channel closed for %s", rid)

    def _channel_alive(self, rid):
        """True if the sport channel is open AND the remote process hasn't
        exited. The dict can still hold a Channel whose underlying process died
        (e.g. m3v_agibot.py crashed at startup) — checking exit_status_ready +
        recv_exit_status distinguishes a live channel from a zombie one."""
        chan = self._sport_chan.get(rid)
        if chan is None:
            return False
        try:
            # exit_status_ready is True only when the remote process has ended.
            if getattr(chan, "exit_status_ready", lambda: False)():
                code = chan.recv_exit_status()
                log.warning("sport channel for %s died (exit=%d)", rid, code)
                self._sport_chan[rid] = None
                return False
            return True
        except Exception:
            self._sport_chan[rid] = None
            return False

    # --- per-tick: auto-launch on online transition ---
    def update(self, dt):
        if not self.get("auto_launch", False):
            return None
        if not self.ctx.robots:
            return None
        for rid, conn in self.ctx.robots.all().items():
            if conn.state == "online" and rid not in self._launched:
                res = self._launch(conn)
                if not res.get("ok"):
                    log.warning("auto-launch %s failed: %s", rid, res.get("error"))
        return None
