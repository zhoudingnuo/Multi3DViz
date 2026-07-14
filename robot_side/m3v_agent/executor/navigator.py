"""navigator.py — Simple P-controller waypoint navigation.

Drives the robot from its current odom pose to a target (x, y) in the robot's
own odometry frame. This is the same coordinate frame the target file's
local_x/local_y are expressed in (control side pre-transforms global→local
via ICP for robot B; see docs/DATA_CONTRACT.md §3.3).

Algorithm (intentionally simple — no nav2/A* dependency, works on both robots):
  1. face_target: turn in place until heading error < ~10°
  2. drive_forward: move straight at the target, re-aiming as needed
  3. arrive: distance < arrive_threshold (0.3m, matching explorer.py:118)
  → done; report back so the poller can wait for a new target.

This runs on its own thread, polling driver.get_pose() at ~20 Hz. The target
poller calls goto()/abort() to steer it. A goto() while one is in flight
replaces the goal.
"""
from __future__ import annotations
import math
import time
import logging
import threading
from typing import Optional

log = logging.getLogger("m3v_agent.executor.nav")

# State machine labels (surfaced to logs/UI).
S_IDLE = "idle"
S_TURN = "turn_to_face"
S_DRIVE = "drive_forward"
S_ARRIVED = "arrived"
S_ABORTED = "aborted"


class Navigator:
    """Closed-loop waypoint driver.

    Args:
        driver: a BaseDriver.
        cfg: ExecutorCfg (arrive_threshold, max_fwd, max_turn, turn_kp).
    """

    def __init__(self, driver, cfg):
        self.driver = driver
        self.cfg = cfg
        self._target = None            # (x, y) or None
        self._state = S_IDLE
        self._stop = threading.Event()
        self._thread = None
        # Internal control cadence.
        self._tick_hz = 20.0

    # --- lifecycle ---
    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="nav", daemon=True)
        self._thread.start()
        log.info("navigator started")

    def stop(self):
        self._target = None
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        try:
            self.driver.stop()
        except Exception:
            log.exception("driver.stop failed during nav shutdown")

    # --- target management (called by target_poller) ---
    def goto(self, x: float, y: float):
        """Set or replace the navigation goal. Returns immediately; the loop
        does the actual driving."""
        self._target = (float(x), float(y))
        log.info("goto (%.2f, %.2f)", x, y)

    def abort(self):
        """Cancel the current goal and halt."""
        self._target = None
        self._state = S_ABORTED
        try:
            self.driver.stop()
        except Exception:
            log.exception("driver.stop failed during abort")
        log.info("navigation aborted")

    @property
    def state(self) -> str:
        return self._state

    @property
    def target(self):
        return self._target

    def arrived(self) -> bool:
        """True if we've arrived at the current target (or have none)."""
        return self._state == S_ARRIVED

    # --- main control loop ---
    def _loop(self):
        period = 1.0 / self._tick_hz
        while not self._stop.is_set():
            tgt = self._target
            if tgt is None:
                # No active goal: ensure we're stopped and idle.
                if self._state not in (S_IDLE, S_ABORTED):
                    try:
                        self.driver.stop()
                    except Exception:
                        pass
                    self._state = S_IDLE
                self._stop.wait(period)
                continue
            pose = self.driver.get_pose()
            if pose is None:
                # No odom yet — can't navigate; hold still.
                self._stop.wait(period)
                continue
            self._step(tgt, pose)
            self._stop.wait(period)

    def _step(self, target, pose):
        """One control tick. target=(tx,ty), pose=(x,y,yaw)."""
        tx, ty = target
        x, y, yaw = pose
        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)
        # Arrived?
        if dist < self.cfg.arrive_threshold:
            try:
                self.driver.stop()
            except Exception:
                pass
            self._state = S_ARRIVED
            log.info("arrived at (%.2f, %.2f) (dist=%.3f)", tx, ty, dist)
            # Hold the target as achieved; poller will issue the next goto.
            return
        # Heading to target.
        desired_yaw = math.atan2(dy, dx)
        err_yaw = self.driver._wrap_angle(desired_yaw - yaw)
        # Phase 1: turn to face (within ~10°).
        if abs(err_yaw) > math.radians(10) and self._state != S_DRIVE:
            self._state = S_TURN
            turn = self.driver._clamp(
                self.cfg.turn_kp * err_yaw,
                -self.cfg.max_turn, self.cfg.max_turn)
            # Slow forward nudge while turning so we don't stall in place.
            try:
                self.driver.move(0.05, 0.0, turn)
            except Exception:
                log.exception("move failed in turn phase")
            return
        # Phase 2: drive forward, re-aiming with a gentle yaw correction.
        self._state = S_DRIVE
        fwd = self.cfg.max_fwd * min(1.0, dist / 0.5)  # slow down near goal
        turn = self.driver._clamp(
            self.cfg.turn_kp * err_yaw * 0.5,           # gentler while driving
            -self.cfg.max_turn, self.cfg.max_turn)
        try:
            self.driver.move(fwd, 0.0, turn)
        except Exception:
            log.exception("move failed in drive phase")
