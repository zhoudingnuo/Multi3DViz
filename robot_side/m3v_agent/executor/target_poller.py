"""target_poller.py — Read the control-side target file + drive the navigator.

The control side (Multi3DViz) SSH-writes `ccenter_target_a.txt` /
`ccenter_target_b.txt` to a fixed local path on each robot. The file is plain
text, one `key: value` per line:

    mode: explore
    global_x: 2.340
    global_y: 1.080
    local_x: 2.340
    local_y: 1.080
    frame: 116
    timestamp: 2026-07-13 14:22:05

We poll that file, parse it, and feed local_x/local_y to the navigator. See
docs/DATA_CONTRACT.md §3 for the full contract.

Coordinate semantics:
  - local_x/local_y are in THIS robot's own odometry frame (camera_init at
    boot), in meters. For robot A (origin of the merged map) local==global;
    for robot B the control side pre-transforms via inv(T_b_to_a). So we
    always navigate to local_* directly — no coordinate math on the robot.

mode values:
  - explore : navigate to (local_x, local_y)
  - stop    : halt (target is None; coords are 0.000)

Safety: if the file's mtime is older than stale_timeout (default 10s), we
assume the control side died and halt the robot.
"""
from __future__ import annotations
import os
import time
import logging
import threading
from typing import Optional

from .navigator import Navigator

log = logging.getLogger("m3v_agent.executor.poller")


def parse_target_file(text: str) -> dict:
    """Parse the `key: value` text format → dict with typed values.

    Tolerant: skips blank lines, lines without ':', and unparseable values
    (keeps them as raw strings). Returns {} for an empty/garbage file.
    """
    out = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, _, raw = line.partition(":")
        key = key.strip()
        raw = raw.strip()
        if not key:
            continue
        # Try numeric coercion for known numeric fields; keep mode/timestamp as str.
        if key in ("global_x", "global_y", "local_x", "local_y"):
            try:
                out[key] = float(raw)
            except ValueError:
                out[key] = raw
        elif key == "frame":
            try:
                out[key] = int(raw)
            except ValueError:
                out[key] = raw
        else:
            out[key] = raw
    return out


class TargetPoller:
    """Polls the target file + drives the navigator.

    Args:
        cfg: ExecutorCfg (target_path, poll_interval, stale_timeout, target_deadband).
        navigator: a Navigator bound to the robot's driver.
    """

    def __init__(self, cfg, navigator: Navigator):
        self.cfg = cfg
        self.nav = navigator
        self._stop = threading.Event()
        self._thread = None
        self._last_target = None       # (lx, ly) last issued, for deadband
        self._last_mtime = 0.0
        self._halted_for_stale = False

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="tgt-poll", daemon=True)
        self._thread.start()
        log.info("target poller started: %s", self.cfg.target_path)

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _loop(self):
        while not self._stop.is_set():
            try:
                self._tick()
            except Exception:
                log.exception("target poll tick error")
            self._stop.wait(self.cfg.poll_interval)

    def _tick(self):
        # 1. Does the file exist?
        path = self.cfg.target_path
        if not os.path.exists(path):
            # No target file ever written: be safe and halt.
            self._maybe_halt("target file absent")
            return
        # 2. Staleness check.
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return
        age = time.time() - mtime
        if age > self.cfg.stale_timeout:
            self._maybe_halt(f"target file stale ({age:.1f}s old)")
            return
        # File is live again after a stale halt — resume polling.
        if self._halted_for_stale:
            self._halted_for_stale = False
            log.info("target file fresh again, resuming")
        # 3. Re-read only when the file changed (mtime bumped).
        if mtime == self._last_mtime:
            return  # control side rewrites even when unchanged; we only act on change
        self._last_mtime = mtime
        try:
            with open(path, "r", encoding="utf-8") as f:
                text = f.read()
        except OSError as e:
            log.warning("target read failed: %s", e)
            return
        fields = parse_target_file(text)
        mode = str(fields.get("mode", "")).lower()
        if mode == "stop":
            self.nav.abort()
            self._last_target = None
            log.info("mode=stop → halted")
            return
        if mode == "explore":
            lx = fields.get("local_x")
            ly = fields.get("local_y")
            if not isinstance(lx, (int, float)) or not isinstance(ly, (int, float)):
                log.warning("explore mode but local_x/local_y missing/invalid: %r", fields)
                self.nav.abort()
                return
            # Deadband: don't re-issue if the target barely moved.
            if self._last_target is not None:
                dx = lx - self._last_target[0]
                dy = ly - self._last_target[1]
                if math_hypot(dx, dy) < self.cfg.target_deadband:
                    return
            self.nav.goto(lx, ly)
            self._last_target = (lx, ly)
            return
        log.debug("unknown mode %r — ignoring", mode)

    def _maybe_halt(self, reason: str):
        """Halt the robot once when we detect the control side is gone."""
        if not self._halted_for_stale:
            self._halted_for_stale = True
            log.warning("halting robot: %s", reason)
            try:
                self.nav.abort()
            except Exception:
                log.exception("abort failed during halt")


# Tiny indirection so the import is visible + works without numpy for the math.
def math_hypot(dx, dy):
    import math
    return math.hypot(dx, dy)
