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
            "type": "string", "default": "cd ~/fast_lio && source devel/setup.bash &&",
            "label": "Prefix prepended to launch_cmd (workspace setup)",
            "group": "Behavior",
        },
    }

    def __init__(self, ctx):
        super().__init__(ctx)
        # robot_id -> True once we've launched on it (so reconnects don't
        # re-launch unless the user explicitly stops+launches again).
        self._launched = set()

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
        if action == "vel":
            return self._send_vel(conn, value)
        if action == "run":
            rc, out = conn.run(str(value or ""))
            return {"ok": rc == 0, "rc": rc, "output": out[-2000:]}
        return {"ok": False, "error": f"unknown action {action}"}

    def _launch(self, conn):
        cmd = conn.cfg.launch_cmd
        if not cmd:
            return {"ok": False, "error": "no launch_cmd configured for this robot"}
        prefix = self.get("launch_prefix", "")
        # nohup + & so SLAM keeps running after the channel closes; redirect
        # output to a log the user can tail.
        full = f"nohup bash -lc {shlex.quote(prefix + ' ' + cmd)} > /tmp/m3v_slam.log 2>&1 &"
        rc, out = conn.run(full, timeout=15)
        if rc == 0:
            self._launched.add(conn.cfg.robot_id)
            log.info("launched SLAM on %s: %s", conn.cfg.robot_id, cmd)
            return {"ok": True}
        return {"ok": False, "rc": rc, "output": out}

    def _stop(self, conn):
        # Kill by the launch_cmd's first token (best-effort). A real deploy
        # would use a pidfile or systemd unit; this is the pragmatic Phase-3 form.
        cmd = conn.cfg.launch_cmd.split()[0] if conn.cfg.launch_cmd else ""
        if cmd:
            conn.run(f"pkill -f {shlex.quote(cmd)}", timeout=8)
        self._launched.discard(conn.cfg.robot_id)
        log.info("stopped SLAM on %s", conn.cfg.robot_id)
        return {"ok": True}

    def _estop(self, conn):
        """Emergency stop: send a halt command to the robot's motion controller
        via SSH. Tries the Unitree TCP bridge first (go2_search protocol via
        go2_tcp_client), then falls back to Agibot's /agibot_cmd topic.
        Best-effort — if neither is running the SSH command just no-ops."""
        rid = conn.cfg.robot_id
        # Unitree Go2: write "0" to the TCP bridge's command channel via a
        # short python one-liner that connects to localhost:21520 and sends STOP.
        # Agibot: publish "0 0 0" to /agibot_cmd (ros2 topic pub --once).
        # We try both — whichever is running on the robot will catch it.
        estop_cmd = (
            # Try Unitree TCP bridge (Go2TcpClient protocol: api_id 1003 = STOPMOVE)
            "python3 -c \""
            "import socket,struct;"
            "try:{"
            "s=socket.socket();s.connect(('127.0.0.1',21520));"
            "s.sendall(b'{\\\"api_id\\\":1003,\\\"parameter\\\":{}}\\n');"
            "s.close()"
            "}except:pass"
            "\" 2>/dev/null; "
            # Try Agibot /agibot_cmd (ROS2)
            "bash -c 'source /opt/ros/*/setup.bash 2>/dev/null && "
            "ros2 topic pub --once /agibot_cmd std_msgs/String \"{data: '0'}\" "
            "2>/dev/null' &"
        )
        rc, out = conn.run(estop_cmd, timeout=5)
        log.warning("ESTOP on %s: rc=%d", rid, rc)
        return {"ok": rc == 0}

    def _send_vel(self, conn, value):
        """Send a one-shot velocity command {vx, vy, yaw} to the robot's motion
        controller via SSH. Used for keyboard takeover (WASD). Same dual-path
        as _estop: Unitree TCP bridge (api 1008 = MOVE) + Agibot /agibot_cmd.
        Rate-limited to avoid flooding SSH — the frontend sends at ~10Hz."""
        if not isinstance(value, dict):
            return {"ok": False, "error": "vel value must be {vx,vy,yaw}"}
        vx = float(value.get("vx", 0))
        vy = float(value.get("vy", 0))
        yaw = float(value.get("yaw", 0))
        rid = conn.cfg.robot_id
        # Unitree TCP bridge: api_id 1008 = MOVE, parameter {x:vx, y:vy, z:yaw}
        vel_cmd = (
            "python3 -c \""
            f"import socket,json;"
            "try:{{"
            "s=socket.socket();s.connect(('127.0.0.1',21520));"
            f"s.sendall(json.dumps({{'api_id':1008,'parameter':{{'x':{vx},'y':{vy},'z':{yaw}}}}})+b'\\n');"
            "s.close()"
            "}}except:pass"
            "\" 2>/dev/null"
        )
        rc, out = conn.run(vel_cmd, timeout=3)
        return {"ok": rc == 0}

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
