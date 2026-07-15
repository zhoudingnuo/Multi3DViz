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
        "robot_b": "/home/orin-001/sda2/restart_all.sh",     # Agibot equivalent (if exists)
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
        conn = self.ctx.robots.get(robot_id) if self.ctx.robots else None
        if conn is None:
            log.warning("command %s: robot %s not found", action, robot_id)
            return {"ok": False, "error": "robot not found"}
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
        pipeline script for this robot_id."""
        rid = conn.cfg.robot_id
        cmd = conn.cfg.launch_cmd or self.PIPELINE_SCRIPTS.get(rid, "")
        if not cmd:
            return {"ok": False, "error": "no launch_cmd or pipeline script for this robot"}
        prefix = self.get("launch_prefix", "")
        full_cmd = f"{prefix} {cmd}".strip() if prefix else cmd
        # nohup + & so the pipeline keeps running after the SSH channel closes.
        full = f"nohup bash -lc {shlex.quote(full_cmd)} > /tmp/m3v_pipeline.log 2>&1 &"
        rc, out = conn.run(full, timeout=20)
        if rc == 0:
            self._launched.add(rid)
            log.info("launched pipeline on %s: %s", rid, full_cmd)
            return {"ok": True}
        return {"ok": False, "rc": rc, "output": out[-500:]}

    def _stop(self, conn):
        """Stop the robot's SLAM pipeline. Uses the cleanup script if known
        (kills all ROS/livox/fastlio processes), otherwise falls back to pkill."""
        rid = conn.cfg.robot_id
        cleanup = {"robot_a": "/home/unitree/sda2/cleanup_ros.sh",
                   "robot_b": "/home/orin-001/sda2/cleanup_ros.sh"}.get(rid, "")
        if cleanup:
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
        """Emergency stop: send zero velocity, then stop_move, then damp (lie
        down). The zero-vel ensures the dog isn't mid-stride when damp hits."""
        rid = conn.cfg.robot_id
        log.warning("ESTOP on %s: zero_vel + stop_move + damp", rid)
        self._send_vel(conn, {"vx": 0, "vy": 0, "yaw": 0})
        self._go2_cmd(conn, 2)   # stop_move
        self._go2_cmd(conn, 0)   # damp — motors to damping, dog lies down safely
        self._standing[rid] = False
        return {"ok": True}

    def _toggle_pose(self, conn):
        """Toggle stand/lie via the m3v_move channel (spacebar during takeover).
        Writes 'stand' or 'lie' to stdin — m3v_move handles the DDS calls."""
        rid = conn.cfg.robot_id
        standing = self._standing.get(rid, False)
        chan = self._sport_chan.get(rid)
        if chan is None:
            return {"ok": False, "error": "no sport channel open"}
        cmd = "lie" if standing else "stand"
        try:
            chan.sendall((cmd + "\n").encode())
            self._standing[rid] = not standing
            log.info("toggle_pose %s: %s", rid, cmd)
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
        """Send velocity to Go2 via the persistent m3v_move process.
        Writes 'vx vy yaw\\n' to its stdin — sub-millisecond (no DDS re-init).
        Falls back to one-shot SSH exec if no channel is open."""
        if not isinstance(value, dict):
            return {"ok": False, "error": "vel value must be {vx,vy,yaw}"}
        vx = float(value.get("vx", 0))
        vy = float(value.get("vy", 0))
        yaw = float(value.get("yaw", 0))
        rid = conn.cfg.robot_id
        # Fast path: persistent m3v_move channel (opened at takeover start).
        chan = self._sport_chan.get(rid)
        if chan is not None:
            try:
                chan.sendall(f"{vx} {vy} {yaw}\n".encode())
                return {"ok": True}
            except Exception:
                self._sport_chan[rid] = None
        # Fallback: one-shot exec (slow).
        cmd = (f"echo '{vx} {vy} {yaw}' | timeout 3 /home/unitree/m3v_move eth0 2>&1")
        rc, out = conn.run(cmd, timeout=5)
        return {"ok": rc == 0}

    def _takeover_start(self, conn):
        """Called when user enters keyboard takeover mode. Opens the persistent
        m3v_move channel so velocity commands are sub-ms latency. The m3v_move
        process auto-sends recovery_stand on startup (dog stands up)."""
        rid = conn.cfg.robot_id
        ok = self.open_sport_channel(conn)
        return {"ok": ok, "msg": "m3v_move started, dog standing" if ok else "failed"}

    def _takeover_end(self, conn):
        """Called when user exits takeover mode. Closes the m3v_move channel
        (which sends stand_down on stdin close → dog lies down)."""
        rid = conn.cfg.robot_id
        self.close_sport_channel(rid)
        return {"ok": True, "msg": "m3v_move closed, dog lying down"}

    def open_sport_channel(self, conn):
        """Open persistent m3v_move on robot. SAFE MODE:
        - Does NOT auto-stand on startup (dog stays lying until user presses space)
        - Watchdog sends 0,0,0 if no velocity command in 500ms
        - stdin close → stop + lie + damp (safe shutdown)
        User must explicitly send 'stand' via spacebar to make the dog stand."""
        rid = conn.cfg.robot_id
        if rid in self._sport_chan and self._sport_chan[rid] is not None:
            return True
        if conn.client is None:
            return False
        try:
            chan = conn.client.get_transport().open_session()
            chan.get_pty()
            chan.exec_command("/home/unitree/m3v_move eth0")
            chan.settimeout(5)
            import time as _t; _t.sleep(4)  # DDS init only (no auto-stand)
            while chan.recv_ready():
                chan.recv(4096)
            self._sport_chan[rid] = chan
            self._standing[rid] = False  # dog is NOT standing (safe mode)
            log.info("m3v_move channel opened for %s (SAFE: dog not standing, awaiting 'stand' cmd)", rid)
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
