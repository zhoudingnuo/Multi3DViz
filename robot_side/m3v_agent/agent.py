"""agent.py — RobotAgent: orchestrate recorder + transport + executor.

Single entry point. Run on each robot:

    python -m m3v_agent.agent --config templates/unitree/config.yaml
    python -m m3v_agent.agent --config templates/agibot/config.yaml --mode record
    python -m m3v_agent.agent --config <yaml> --mode execute

Modes:
    record   — recorder + scp pusher only (no motion). For capturing data.
    execute  — executor only (target file → driver). Assumes recorder runs
               separately or the odom comes from another process. NOTE: the
               driver reads pose from the recorder's odom cache, so in pure
               execute mode you still need a recorder running for pose — or
               use the 'both' mode.
    both     — recorder + scp pusher + executor (default). Full closed loop.

Lifecycle / signal handling:
    SIGINT/SIGTERM → lie_down (safest posture) + flush + stop ROS.
    Exception in driver.connect() → non-fatal; the agent keeps running the
    recorder and retries the driver on the next manual restart.
"""
from __future__ import annotations
import os
import sys
import time
import signal
import logging
import argparse

from .config import RobotSideConfig
from .recorder.cloud_sink import FastlioRecorder
from .transport.scp_pusher import ScpPusher
from .executor.navigator import Navigator
from .executor.target_poller import TargetPoller
from .drivers import make_driver
from .web.status_server import StatusServer

log = logging.getLogger("m3v_agent")


class RobotAgent:
    """Owns recorder, transport, executor, web panel. Wires them together."""

    def __init__(self, cfg: RobotSideConfig):
        self.cfg = cfg
        self.recorder: FastlioRecorder = None  # type: ignore[assignment]
        self.pusher: ScpPusher = None           # type: ignore[assignment]
        self.driver = None
        self.navigator: Navigator = None       # type: ignore[assignment]
        self.poller: TargetPoller = None       # type: ignore[assignment]
        self.web: StatusServer = None          # type: ignore[assignment]
        self._stop = False

    def start(self):
        mode = (self.cfg.mode or "both").lower()
        log.info("starting agent mode=%s driver=%s", mode, self.cfg.driver.kind)
        # Always need the recorder if we record or drive (driver needs pose).
        # Recorder is wrapped in try — if ROS (rclpy/rospy) is unavailable the
        # agent still runs its other subsystems and the panel reports the gap.
        if mode in ("record", "both") or (mode == "execute" and self.cfg.executor.enabled):
            try:
                self.recorder = FastlioRecorder(self.cfg.recorder)
                self.recorder.start()
            except Exception as e:
                log.error("recorder failed to start (ROS unavailable?): %s", e)
                self.recorder = None
            # Transport (scp push) only makes sense while recording.
            if self.recorder and mode in ("record", "both") and self.cfg.transport.enabled:
                try:
                    self.pusher = ScpPusher(self.cfg.transport, self.recorder)
                    self.pusher.start()
                except Exception as e:
                    log.error("transport failed to start: %s", e)
                    self.pusher = None
        # Executor path.
        if mode in ("execute", "both") and self.cfg.executor.enabled:
            self.driver = make_driver(self.cfg.driver.kind, self.cfg.driver, self.recorder)
            self.driver.recorder = self.recorder  # so get_pose() works (same-process mode)
            # Split-process fallback: when there is no local recorder (execute
            # mode), tail the odom_stream.jsonl the recorder writes on the
            # shared filesystem. This bridges pose across the container/host
            # boundary for the Agibot ROS1 split deployment.
            if self.recorder is None:
                odom_path = self._resolve_odom_file()
                if odom_path:
                    from .executor.odom_file_pose import OdomFilePoseProvider
                    self.driver.odom_file_pose = OdomFilePoseProvider(odom_path)
                    log.info("execute mode: pose from %s", odom_path)
                else:
                    log.warning("execute mode: no odom_stream.jsonl found yet — "
                                "driver will have no pose until the recorder creates it")
            ok = False
            try:
                ok = self.driver.connect()
            except Exception:
                log.exception("driver.connect failed; executor disabled (recorder keeps running)")
            if ok:
                try:
                    self.driver.stand_up()
                except Exception:
                    log.exception("initial stand_up failed; continuing anyway")
                self.navigator = Navigator(self.driver, self.cfg.executor)
                self.navigator.start()
                self.poller = TargetPoller(self.cfg.executor, self.navigator)
                self.poller.start()
            else:
                log.warning("driver not connected — executor will not run. "
                            "recorder/transport still active.")
        # Web status panel (always last — it reads the others via snapshot()).
        if self.cfg.web.enabled:
            self.web = StatusServer(
                host=self.cfg.web.host, port=self.cfg.web.port,
                snapshot=self.snapshot, on_estop=self.emergency_stop,
            )
            self.web.start()
        log.info("agent ready")

    def _resolve_odom_file(self) -> str:
        """Find the latest run dir's odom_stream.jsonl for the file-pose fallback.

        Returns the path to <data_root>/<robot>/data/run_<ts>/Odometry/odom_stream.jsonl,
        picking the newest run dir, or "" if none exists yet (the recorder
        hasn't started / written its first frame)."""
        import glob
        rc = self.cfg.recorder
        base = os.path.join(rc.data_root, rc.robot, "data")
        runs = sorted(glob.glob(os.path.join(base, "run_*")))
        if not runs:
            return ""
        return os.path.join(runs[-1], "Odometry", "odom_stream.jsonl")

    # --- status snapshot (read by the web panel via GET /api/state) ---
    def snapshot(self) -> dict:
        """Collect a JSON-serializable snapshot of every subsystem's state.

        Each sub-config/instance is best-effort: a missing/errored subsystem
        still yields a usable panel rather than 500ing. Reads happen on the
        HTTP thread, so we only touch already-thread-safe attributes
        (recorder's _lock guards _frame_idx + _latest_pose)."""
        import os as _os
        cfg = self.cfg
        out: dict = {"mode": cfg.mode}

        # --- robot identity ---
        out["robot"] = {
            "robot_id": cfg.recorder.robot,
            "label": cfg.recorder.robot,
            "host": _host_ip(),
            "ros": cfg.recorder.ros,
            "driver_kind": cfg.driver.kind,
            "driver_connected": bool(getattr(self.driver, "_connected", False)
                                     or getattr(self.driver, "_initialized", False)),
            "standing": bool(getattr(self.driver, "_standing", False)),
        }

        # --- recorder ---
        rec_cfg = cfg.recorder
        latest = self.recorder.latest_pose() if self.recorder else {}
        out["recorder"] = {
            "enabled": bool(self.recorder),
            "run_dir": getattr(self.recorder, "run_dir", "") or "",
            "frame_idx": getattr(self.recorder, "_frame_idx", 0) if self.recorder else 0,
            "gravity_enabled": rec_cfg.gravity_enabled,
            "gravity_ready": bool(getattr(self.recorder.gravity, "ready", True))
                             if (self.recorder and self.recorder.gravity) else True,
            "latest_pose": {
                "x": latest.get("x", 0.0), "y": latest.get("y", 0.0),
                "yaw": latest.get("yaw", 0.0),
            } if latest else None,
            "cloud_topic": rec_cfg.cloud_topic,
            "odom_topic": rec_cfg.odom_topic,
            "naming": rec_cfg.naming,
        }

        # --- transport ---
        tr_cfg = cfg.transport
        out["transport"] = {
            "enabled": bool(self.pusher),
            "connected": bool(self.pusher and self.pusher._sftp is not None),
            "pushed_idx": getattr(self.pusher, "_last_idx", 0) if self.pusher else 0,
            "target_host": tr_cfg.host,
            "target_user": tr_cfg.user,
            "remote_root": tr_cfg.remote_root,
            "last_error": "",
        }

        # --- executor ---
        ex_cfg = cfg.executor
        tgt = None
        nav_state = "idle"
        halted = False
        file_age = None
        if self.poller:
            halted = bool(getattr(self.poller, "_halted_for_stale", False))
            # Reconstruct the current target from the poller's last parse.
            if getattr(self.poller, "_last_target", None):
                tgt = {"mode": "explore",
                       "local_x": self.poller._last_target[0],
                       "local_y": self.poller._last_target[1],
                       "global_x": self.poller._last_target[0],
                       "global_y": self.poller._last_target[1],
                       "frame": 0}
            # File age for staleness transparency.
            try:
                import time as _t
                file_age = _t.time() - _os.path.getmtime(ex_cfg.target_path)
            except (OSError, ValueError):
                file_age = None
        if self.navigator:
            nav_state = self.navigator.state
        out["executor"] = {
            "enabled": bool(self.poller),
            "nav_state": nav_state,
            "halted_for_stale": halted,
            "current_target": tgt,
            "target_path": ex_cfg.target_path,
            "file_age_s": file_age,
            "arrive_threshold": ex_cfg.arrive_threshold,
        }
        return out

    def emergency_stop(self) -> bool:
        """Web-panel ESTOP callback: drop the robot to a safe posture."""
        log.warning("EMERGENCY STOP triggered from web panel")
        if self.driver is None:
            return False
        try:
            ok = self.driver.emergency_stop()
            # Also abort the navigator so it stops issuing move() commands.
            if self.navigator:
                self.navigator.abort()
            return bool(ok)
        except Exception:
            log.exception("emergency_stop failed")
            return False

    def stop(self):
        if self._stop:
            return
        self._stop = True
        log.info("agent stopping...")
        # Order matters: stop the poller first so it can't issue new gotos,
        # then the navigator (which calls driver.stop), then lie_down + disconnect,
        # then the recorder (flush odom + gravity + ROS).
        if self.poller:
            self.poller.stop()
        if self.navigator:
            self.navigator.stop()
        if self.driver is not None:
            try:
                self.driver.lie_down()
            except Exception:
                log.exception("lie_down during shutdown failed")
            try:
                self.driver.disconnect()
            except Exception:
                log.exception("driver disconnect failed")
        if self.pusher:
            self.pusher.stop()
        if self.recorder:
            self.recorder.stop()
        # Stop the web panel last so it can serve the "shutting down" state
        # up to the moment we tear everything down.
        if self.web:
            self.web.stop()
        log.info("agent stopped")


def _host_ip() -> str:
    """Best-effort primary LAN IP of this robot (for the panel's display only).

    Connects a UDP socket to a public addr (no packets actually sent — UDP
    connect just sets the routing) and reads the bound local IP. Falls back
    to the hostname if anything goes wrong. Pure display value."""
    import socket as _sock
    try:
        s = _sock.socket(_sock.AF_INET, _sock.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        finally:
            s.close()
    except Exception:
        try:
            return _sock.gethostname()
        except Exception:
            return "?"


def _setup_logging(level: str, to_stderr: bool = False):
    """Configure logging.

    In --ui-stdio mode stdout is the IPC channel (only tagged JSON lines may
    appear there), so logs go to stderr. Otherwise logs go to stdout (the
    normal console mode) — that's friendlier when running in a terminal."""
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stderr if to_stderr else sys.stdout,
    )


def main(argv=None):
    p = argparse.ArgumentParser(prog="m3v-agent",
                                description="Multi3DViz 受控端 agent")
    p.add_argument("--config", "-c", required=True,
                   help="Path to YAML config (templates/<robot>/config.yaml)")
    p.add_argument("--mode", "-m",
                   choices=["record", "execute", "both"],
                   help="Override config.mode")
    p.add_argument("--ui-stdio", action="store_true",
                   help="UI IPC mode: emit 'STATE: <json>' on stdout every "
                        "second + read ESTOP/STOP commands from stdin. Used by "
                        "the Electron desktop shell — no HTTP server started.")
    args = p.parse_args(argv)

    cfg = RobotSideConfig.from_yaml(args.config)
    cfg.apply_env_overrides()
    if args.mode:
        cfg.mode = args.mode
    # In stdio mode, the Electron shell owns the window — disable the HTTP
    # status server (we use stdout/stdin IPC instead).
    if args.ui_stdio:
        cfg.web.enabled = False
    _setup_logging(cfg.log_level, to_stderr=bool(args.ui_stdio))

    agent = RobotAgent(cfg)
    # Signal handlers: graceful shutdown so we always lie_down + flush.
    def _sig(signum, frame):
        log.info("caught signal %d, shutting down", signum)
        agent.stop()
        # Give threads a beat to unwind, then exit.
        time.sleep(0.5)
        sys.exit(0)
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    # --- stdio UI loop: emit state JSON + read commands ---
    if args.ui_stdio:
        import json as _json
        _stdio_loop(agent)
        return

    try:
        agent.start()
        # Block main thread until killed.
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        agent.stop()


def _stdio_loop(agent: "RobotAgent"):
    """UI IPC loop for the Electron desktop shell.

    Protocol (one JSON object per line on stdout, prefix-tagged):
      STATE: {...}        — emitted every UI_POLL_HZ seconds, the snapshot()
      READY: {...}        — emitted once after agent.start() succeeds
      LOG:   "..."        — forwarded log lines (best-effort)
      DYING: {...}        — emitted once before exit

    Commands read from stdin (one per line):
      ESTOP   — trigger emergency_stop()
      STOP    — trigger agent.stop() + exit
    """
    import json as _json
    log.info("starting in --ui-stdio mode (Electron desktop shell IPC)")
    try:
        agent.start()
    except Exception as e:
        sys.stdout.write("ERROR: " + _json.dumps({"msg": str(e)}) + "\n")
        sys.stdout.flush()
        return
    # Announce ready so the shell can swap out its "starting…" screen.
    sys.stdout.write("READY: " + _json.dumps({"mode": agent.cfg.mode,
                                              "driver": agent.cfg.driver.kind}) + "\n")
    sys.stdout.flush()

    # Non-blocking stdin reader on a daemon thread.
    import threading as _th
    cmd_q: "list" = []
    def _stdin_reader():
        # readline blocks; reading happens on this thread, commands queued.
        while True:
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                break  # EOF — shell closed
            cmd_q.append(line.strip())
    t = _th.Thread(target=_stdin_reader, name="ui-stdin", daemon=True)
    t.start()

    period = 1.0  # 1 Hz state emit (matches the old HTTP poll rate)
    try:
        while True:
            # Drain pending commands.
            while cmd_q:
                cmd = cmd_q.pop(0).upper()
                if cmd == "ESTOP":
                    ok = agent.emergency_stop()
                    sys.stdout.write("ESTOP_ACK: " + _json.dumps({"ok": ok}) + "\n")
                    sys.stdout.flush()
                elif cmd == "STOP":
                    raise KeyboardInterrupt
            # Emit state snapshot.
            try:
                snap = agent.snapshot()
                sys.stdout.write("STATE: " + _json.dumps(snap, default=_json_default) + "\n")
                sys.stdout.flush()
            except Exception as e:
                log.exception("snapshot failed in stdio loop: %s", e)
            time.sleep(period)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        sys.stdout.write("DYING: " + _json.dumps({}) + "\n")
        sys.stdout.flush()
        agent.stop()


def _json_default(o):
    """JSON fallback for non-serializable objects (np types, etc.)."""
    for attr in ("tolist", "item"):
        if hasattr(o, attr):
            try:
                return getattr(o, attr)()
            except Exception:
                pass
    return str(o)


if __name__ == "__main__":
    main()
