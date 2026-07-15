"""robot_manager.py — dynamic multi-robot connection management.

Replaces ccenter's hardcoded 2-robot remote_flag.py with a runtime registry:
the user adds/removes robots at will. Each RobotConnection holds a persistent
paramiko SSHClient (so we don't open/close per command like ccenter did), runs
a background heartbeat that detects drops, and auto-reconnects with backoff.

A Robot is identified by a caller-chosen `robot_id` (string) and carries:
    - SSH config: host/port/user/password (None password → key auth)
    - data_path: where its cloud_registered/Odometry live (for DataSources)
    - connection state: disconnected → connecting → online → (drop) → reconnecting

The manager is NOT a plugin — it's a core service held by Backend and exposed
to plugins via ctx.robots. Plugins (SSHLauncherService, ConnectionMonitor) and
the frontend (robot_add/remove WS messages) drive it.

Thread model: SSH ops run on the manager's heartbeat thread + caller threads.
paramiko clients are not fully thread-safe across channels, so each run() call
opens its own exec_command channel on the shared client (channels ARE isolated)
and we guard client creation/reconnection with a per-robot lock.
"""
from __future__ import annotations
import os
import time
import threading
import logging
from dataclasses import dataclass, field
from typing import Optional, Callable

log = logging.getLogger("multi3dviz.robots")

# Heartbeat interval + reconnect backoff bounds.
HEARTBEAT_INTERVAL = 3.0      # seconds between pings while online
CONNECT_TIMEOUT = 4.0         # SSH connect/auth timeout
RECONNECT_MIN = 2.0           # first reconnect backoff
RECONNECT_MAX = 30.0          # cap


# Connection states (surface to UI as strings).
S_DISCONNECTED = "disconnected"
S_CONNECTING = "connecting"
S_ONLINE = "online"
S_RECONNECTING = "reconnecting"
S_ERROR = "error"


@dataclass
class RobotConfig:
    """User-supplied robot identity. Persisted so the fleet survives restart."""
    robot_id: str
    host: str
    port: int = 22
    user: str = ""
    password: Optional[str] = None   # None → SSH key/agent auth
    # Where this robot's recorded data lives (for local-replay DataSources).
    # May be a local path (if data is synced) or remote (future: SFTP pull).
    data_path: str = ""
    # Optional: a command to launch SLAM on the robot (used by SSHLauncher).
    launch_cmd: str = ""
    label: str = ""                 # human-friendly name


class RobotConnection:
    """One robot's live connection. Owns the SSH client + heartbeat thread."""

    def __init__(self, cfg: RobotConfig, on_state_change: Callable):
        self.cfg = cfg
        self._on_state = on_state_change
        self.state = S_DISCONNECTED
        self.client = None           # paramiko.SSHClient | None
        self.last_error = ""
        self.last_seen = 0.0         # monotonic time of last successful ping
        self.battery_pct = -1        # battery %, -1 = unknown
        self._shell_chan = None      # persistent shell channel for low-latency cmds
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._reconnect_delay = RECONNECT_MIN

    # --- state transitions ---
    def _set_state(self, state, error=""):
        if state == self.state and not error:
            return
        self.state = state
        if error:
            self.last_error = error
        log.info("robot %s state -> %s%s", self.cfg.robot_id, state,
                 f" ({error})" if error else "")
        try:
            self._on_state(self.cfg.robot_id, state, error)
        except Exception:
            log.exception("on_state_change callback failed")

    # --- connection lifecycle ---
    def _open_client(self) -> bool:
        """Open a fresh SSH client. Returns True on success. Caller holds _lock."""
        try:
            import paramiko
        except ImportError:
            self._set_state(S_ERROR, "paramiko not installed")
            return False
        use_key = not self.cfg.password
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(
                self.cfg.host, port=self.cfg.port,
                username=self.cfg.user,
                password=(self.cfg.password if not use_key else None),
                timeout=CONNECT_TIMEOUT, banner_timeout=CONNECT_TIMEOUT,
                auth_timeout=CONNECT_TIMEOUT,
                allow_agent=use_key, look_for_keys=use_key,
            )
        except Exception as e:
            self.client = None
            self._set_state(S_RECONNECTING, f"connect failed: {e}")
            return False
        self.client = client
        self.last_seen = time.monotonic()
        self._reconnect_delay = RECONNECT_MIN  # reset backoff on success
        self._set_state(S_ONLINE)
        return True

    def _close_client(self):
        self.close_shell()
        if self.client is not None:
            try:
                self.client.close()
            except Exception:
                pass
            self.client = None

    # --- heartbeat loop ---
    def start(self):
        """Spawn the heartbeat thread. Idempotent."""
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name=f"hb-{self.cfg.robot_id}")
        self._thread.start()

    def stop(self):
        """Tear down the connection + heartbeat."""
        self._stop.set()
        self._close_client()
        self._set_state(S_DISCONNECTED)
        if self._thread:
            self._thread.join(timeout=2)

    def _loop(self):
        """Heartbeat: connect, ping on a cadence, reconnect on drop with backoff."""
        while not self._stop.is_set():
            if self.client is None:
                with self._lock:
                    if self.client is None:
                        self._set_state(S_CONNECTING)
                        if not self._open_client():
                            # backoff before retrying
                            self._stop.wait(self._reconnect_delay)
                            self._reconnect_delay = min(
                                self._reconnect_delay * 1.5, RECONNECT_MAX)
                            continue
            # client open — ping
            if self._ping():
                self.last_seen = time.monotonic()
            else:
                # ping failed — drop and let the loop reconnect
                with self._lock:
                    self._close_client()
                self._set_state(S_RECONNECTING, "ping failed")
            self._stop.wait(HEARTBEAT_INTERVAL)

    def _ping(self) -> bool:
        """Run a trivial remote command to verify the connection is alive."""
        rc, _ = self.run("true", timeout=5)
        return rc == 0

    # --- command execution (used by SSHLauncher / target dispatch) ---
    def run(self, command: str, stdin_text: Optional[str] = None,
            timeout: int = 8) -> tuple[int, str]:
        """Run a remote command on this robot. Returns (exit_code, stdout).

        Opens a fresh channel on the shared client (channels are isolated, so
        concurrent run() calls from different threads are safe). If the client
        is missing/closed, returns (-1, 'not connected')."""
        with self._lock:
            client = self.client
        if client is None:
            return -1, "not connected"
        try:
            stdin, stdout, stderr = client.exec_command(command, timeout=timeout)
            if stdin_text is not None:
                stdin.write(stdin_text)
                stdin.channel.shutdown_write()
            rc = stdout.channel.recv_exit_status()
            out = stdout.read().decode("utf-8", errors="replace")
            return rc, out
        except Exception as e:
            return -1, f"exec failed: {e}"

    # --- persistent shell channel (low-latency velocity commands) ---
    def open_shell(self) -> bool:
        """Open a persistent interactive shell channel for low-latency commands.
        Used by keyboard takeover: instead of exec_command per 10Hz velocity
        packet (each ~200ms overhead), we write one line to stdin (~5ms).
        Returns True if the shell is open (or was already open)."""
        with self._lock:
            if self._shell_chan is not None and self._shell_chan.recv_ready() is not None:
                try:
                    self._shell_chan.send("\n")  # probe
                    return True
                except Exception:
                    self._shell_chan = None
            client = self.client
            if client is None:
                return False
            try:
                chan = client.invoke_shell()
                chan.settimeout(2)
                self._shell_chan = chan
                # Drain the initial banner.
                import time as _t; _t.sleep(0.1)
                while chan.recv_ready():
                    chan.recv(4096)
                log.info("robot %s: persistent shell opened", self.cfg.robot_id)
                return True
            except Exception as e:
                log.warning("robot %s: open_shell failed: %s", self.cfg.robot_id, e)
                self._shell_chan = None
                return False

    def shell_send(self, line: str) -> bool:
        """Send a single line to the persistent shell. Non-blocking — does NOT
        wait for output (fire-and-forget for velocity commands). Returns False
        if no shell is open."""
        with self._lock:
            chan = self._shell_chan
        if chan is None:
            return False
        try:
            chan.sendall((line + "\n").encode())
            return True
        except Exception:
            with self._lock:
                self._shell_chan = None
            return False

    def close_shell(self):
        """Close the persistent shell channel (on takeover release / disconnect)."""
        with self._lock:
            if self._shell_chan is not None:
                try:
                    self._shell_chan.close()
                except Exception:
                    pass
                self._shell_chan = None

    def write_file(self, remote_path: str, content: str) -> bool:
        """Write a file on the robot via `cat > path` over stdin. Best-effort."""
        import shlex as _shlex
        parent = os.path.dirname(remote_path) or "."
        cmd = f"mkdir -p {_shlex.quote(parent)} && cat > {_shlex.quote(remote_path)}"
        rc, msg = self.run(cmd, stdin_text=content, timeout=8)
        if rc != 0:
            log.warning("robot %s write_file %s rc=%s: %s",
                        self.cfg.robot_id, remote_path, rc, msg)
            return False
        return True


class RobotManager:
    """Holds all live RobotConnections. Plugins/UI add and remove robots here.
    Notifies a listener (Backend wires it to broadcast robot_status to UI)."""

    def __init__(self, on_status: Optional[Callable] = None):
        self._robots: dict[str, RobotConnection] = {}
        self._lock = threading.Lock()
        self._on_status = on_status   # callable(robot_id, state, error)

    # --- registry ops ---
    def add(self, cfg: RobotConfig) -> bool:
        """Register a robot + start its heartbeat. Returns False on dup id."""
        with self._lock:
            if cfg.robot_id in self._robots:
                return False
            conn = RobotConnection(cfg, self._on_status_change)
            self._robots[cfg.robot_id] = conn
        conn.start()
        log.info("added robot %s (%s@%s)", cfg.robot_id, cfg.user, cfg.host)
        self._emit(cfg.robot_id, conn.state, "")
        return True

    def remove(self, robot_id: str) -> bool:
        with self._lock:
            conn = self._robots.pop(robot_id, None)
        if conn is None:
            return False
        conn.stop()
        log.info("removed robot %s", robot_id)
        return True

    def get(self, robot_id: str) -> Optional[RobotConnection]:
        with self._lock:
            return self._robots.get(robot_id)

    def all(self) -> dict[str, RobotConnection]:
        with self._lock:
            return dict(self._robots)

    def list_state(self) -> list[dict]:
        """Snapshot of all robots for the UI."""
        with self._lock:
            return [
                {
                    "robot_id": c.cfg.robot_id,
                    "label": c.cfg.label or c.cfg.robot_id,
                    "host": c.cfg.host,
                    "user": c.cfg.user,
                    "state": c.state,
                    "error": c.last_error,
                    "last_seen": round(c.last_seen, 1),
                    "battery_pct": c.battery_pct,
                }
                for c in self._robots.values()
            ]

    def shutdown(self):
        """Stop every robot (on app exit)."""
        with self._lock:
            conns = list(self._robots.values())
        for c in conns:
            c.stop()

    # --- callback plumbing ---
    def _on_status_change(self, robot_id, state, error):
        self._emit(robot_id, state, error)

    def _emit(self, robot_id, state, error):
        if self._on_status:
            try:
                self._on_status(robot_id, state, error)
            except Exception:
                log.exception("on_status callback failed")
