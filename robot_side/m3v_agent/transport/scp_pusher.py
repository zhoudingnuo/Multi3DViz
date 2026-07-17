"""scp_pusher.py — Daemon that pushes recorded frames to the Windows control side.

The control side (Multi3DViz) reads ONLY local disk. There is no robot→control
data transport in the existing codebase — the `data_path` field on RobotConfig
is a documented-but-unwired hook ("future: SFTP pull"). So the robot must push.

This daemon watches the recorder's run dir for new .npy frames and uploads them
via SFTP to the Windows box. Because the control side's LocalReplaySource scans
`<data_root>/<robot>/data/<run>/`, we mirror that exact layout on the remote.

Upload strategy:
  - Per-frame .npy  : uploaded individually as they appear (incremental).
  - odom_stream.jsonl: re-uploaded whole each cycle (small, append-only).
  - gravity_calibration.json: uploaded once when it appears.
  - Atomic remote rename: sftp.put(tmp) then posix_rename(tmp, final) so the
    control side's reader (which polls *.npy via sorted glob) never sees a
    half-uploaded file.

Threading: runs on its own daemon thread so it never blocks the recorder. The
SFTP client is opened lazily and reconnected on drop. Failures are retried with
backoff; already-uploaded frames are skipped via the last_pushed_idx counter.
"""
from __future__ import annotations
import os
import sys
import time
import logging
import threading
import glob as _glob

log = logging.getLogger("m3v_agent.transport.scp")


class ScpPusher:
    """Background SFTP pusher.

    Args:
        cfg: TransportCfg instance (host/port/user/password/remote_root/interval).
        recorder: the FastlioRecorder — read its run_dir/cloud_dir/odom_path.
    """

    def __init__(self, cfg, recorder):
        self.cfg = cfg
        self.recorder = recorder
        self._stop = threading.Event()
        self._thread = None
        self._client = None          # paramiko.SSHClient
        self._sftp = None            # paramiko.SFTPClient
        self._last_idx = 0           # next frame index to upload (when NOT deleting)
        self._uploaded = set()       # basenames already uploaded (when deleting)
        self._remote_root = cfg.remote_root
        self._pushed_gravity = False
        # Convert Windows-style backslash remote root to forward slashes (SFTP
        # is POSIX path semantics even when talking to Windows OpenSSH).
        self._remote_root = self._remote_root.replace("\\", "/")

    # --- lifecycle ---
    def start(self):
        if not self.cfg.enabled:
            log.info("scp pusher disabled by config")
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="scp-push", daemon=True)
        self._thread.start()
        log.info("scp pusher started → %s@%s:%s", self.cfg.user, self.cfg.host, self._remote_root)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)
        self._close_sftp()

    # --- connection ---
    def _connect(self) -> bool:
        try:
            import paramiko
        except ImportError:
            log.error("paramiko not installed; scp pusher cannot run")
            return False
        use_key = not self.cfg.password
        try:
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(
                self.cfg.host, port=self.cfg.port,
                username=self.cfg.user,
                password=(self.cfg.password if not use_key else None),
                timeout=5, banner_timeout=5, auth_timeout=5,
                allow_agent=use_key, look_for_keys=use_key,
            )
            self._client = client
            self._sftp = client.open_sftp()
            log.info("scp connected to %s@%s", self.cfg.user, self.cfg.host)
            return True
        except Exception as e:
            log.warning("scp connect failed: %s", e)
            self._close_sftp()
            return False

    def _close_sftp(self):
        for closer in (self._sftp, self._client):
            if closer is not None:
                try:
                    closer.close()
                except Exception:
                    pass
        self._sftp = None
        self._client = None

    def _ensure_connected(self) -> bool:
        if self._sftp is not None:
            return True
        return self._connect()

    # --- remote path helpers ---
    def _remote_mkdirs(self, remote_path: str):
        """mkdir -p on the remote (SFTP has no recursive mkdir).

        Handles both POSIX absolute paths (/home/foo) and Windows drive paths
        (C:/Users/foo). For Windows drive paths we must NOT prefix with '.'
        (./C:/... is a bogus relative path); instead we seed `cur` with the
        drive letter (e.g. 'C:') and build from there."""
        if not remote_path or remote_path in ("/", ".", ""):
            return
        parts = [p for p in remote_path.split("/") if p]
        # Determine the seed: '/foo' → '' (then first part becomes absolute);
        # 'C:/...' → start from 'C:'; relative 'foo/...' → start from '.'.
        cur = ""
        if remote_path.startswith("/"):
            cur = ""
        elif parts and len(parts[0]) == 2 and parts[0][1] == ":":
            # Windows drive letter like 'C:' — seed with it, don't prefix '.'.
            cur = parts[0]
            parts = parts[1:]
        else:
            cur = "."
        for p in parts:
            cur = cur + "/" + p if cur else p
            try:
                self._sftp.stat(cur)  # raises if missing
            except IOError:
                try:
                    self._sftp.mkdir(cur)
                except IOError:
                    pass  # race or exists

    def _remote_put_atomic(self, local_path: str, remote_path: str):
        """Upload local→remote atomically via tmp + posix_rename.

        posix-rename@openssh.com is atomic and overwrites the destination,
        supported by OpenSSH server >= 5.x. Falls back to plain put if the
        server doesn't advertise it."""
        if self.cfg.atomic_remote:
            tmp = remote_path + ".m3vtmp"
            try:
                self._sftp.put(local_path, tmp)
                # posix_rename is an extension; paramiko exposes it via the
                # request method name. Newer paramiko has sftp.posix_rename.
                if hasattr(self._sftp, "posix_rename"):
                    self._sftp.posix_rename(tmp, remote_path)
                else:
                    # Fallback: rename (fails if dest exists) → unlink + rename.
                    try:
                        self._sftp.remove(remote_path)
                    except IOError:
                        pass
                    self._sftp.rename(tmp, remote_path)
                return
            except Exception as e:
                log.warning("atomic put %s failed (%s); falling back to direct put",
                            os.path.basename(remote_path), e)
                try:
                    self._sftp.remove(tmp)
                except Exception:
                    pass
        # Direct (non-atomic) fallback.
        self._sftp.put(local_path, remote_path)

    # --- main loop ---
    def _loop(self):
        backoff = 1.0
        while not self._stop.is_set():
            # Wait for the recorder to have a run dir.
            run = getattr(self.recorder, "run_dir", "")
            if not run or not os.path.isdir(run):
                self._stop.wait(1.0)
                continue
            try:
                if not self._ensure_connected():
                    self._stop.wait(min(backoff, 30.0))
                    backoff = min(backoff * 1.5, 30.0)
                    continue
                backoff = 1.0
                self._push_cycle(run)
            except Exception:
                log.exception("scp push cycle error; will reconnect")
                self._close_sftp()
            self._stop.wait(self.cfg.interval)

    def _push_cycle(self, run_dir: str):
        """One upload pass. LOW-LATENCY MODE: only upload the LATEST frame +
        latest odom, then delete all older frames locally. This prevents
        backlog when the network is slower than the 10Hz recording rate —
        instead of falling further behind uploading every frame in order, we
        skip to the newest and discard the backlog."""
        robot = self.recorder.cfg.robot
        cloud_local = os.path.join(run_dir, "cloud_registered")
        run_name = os.path.basename(run_dir)
        remote_run = f"{self._remote_root}/{robot}/data/{run_name}"
        remote_cloud = f"{remote_run}/cloud_registered"
        remote_odom_dir = f"{remote_run}/Odometry"
        self._remote_mkdirs(remote_cloud)
        self._remote_mkdirs(remote_odom_dir)

        # 1. Upload ONLY the latest .npy frame. Delete all others (backlog).
        files = sorted(_glob.glob(os.path.join(cloud_local, "*.npy")))
        # Filter out temp files from atomic writes.
        files = [f for f in files if not f.endswith(".tmp") and ".m3vtmp" not in f]
        if files:
            latest = files[-1]
            base = os.path.basename(latest)
            remote_path = f"{remote_cloud}/{base}"
            try:
                self._remote_put_atomic(latest, remote_path)
                # Delete ALL local frames (including the one we just uploaded)
                # so the next cycle only sees frames that arrived after this one.
                for f in files:
                    try:
                        os.remove(f)
                    except OSError:
                        pass
                self._uploaded.add(base)
                log.info("pushed latest frame %s (purged %d backlog)", base, len(files))
            except Exception as e:
                log.warning("upload latest %s failed: %s", base, e)

        # 2. Upload only the LAST line of odom_stream.jsonl (latest pose).
        #    The control side's OdomFilePoseProvider tails this file, so we
        #    rewrite it to contain just the latest pose — keeps it tiny and
        #    the host never reads stale history.
        odom_local = os.path.join(run_dir, "Odometry", "odom_stream.jsonl")
        if os.path.exists(odom_local):
            remote_odom = f"{remote_odom_dir}/odom_stream.jsonl"
            try:
                # Read all lines, keep only the last one, upload that.
                with open(odom_local, "r", encoding="utf-8") as fh:
                    lines = fh.readlines()
                if lines:
                    last_line = lines[-1]
                    # Upload the last line as the full remote file.
                    import io as _io
                    self._sftp.putfo(_io.BytesIO(last_line.encode("utf-8")), remote_odom)
                    # Truncate the local odom file to just the last line too
                    # (prevents it from growing unbounded).
                    with open(odom_local, "w", encoding="utf-8") as fh:
                        fh.write(last_line)
            except Exception as e:
                log.warning("odom upload failed: %s", e)

        # 3. Gravity calibration (once).
        if not self._pushed_gravity:
            g_local = os.path.join(run_dir, "gravity_calibration.json")
            if os.path.exists(g_local):
                try:
                    self._remote_put_atomic(g_local, f"{remote_run}/gravity_calibration.json")
                    self._pushed_gravity = True
                    log.info("pushed gravity_calibration.json")
                except Exception as e:
                    log.warning("gravity upload failed: %s", e)
