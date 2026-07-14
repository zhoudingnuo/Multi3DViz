"""test_fake_driver.py — FakeDriver + end-to-end executor pipeline test.

FakeDriver implements BaseDriver purely in-memory (no SDK, no ROS). It also
simulates motion: each move() call nudges an internal pose toward the commanded
velocity, so the navigator can actually "arrive" at a target. This lets us test
the full poller→navigator→driver loop without any hardware.

Run with:  python -m pytest tests/test_fake_driver.py -v
"""
import os
import sys
import time
import math
import threading

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from m3v_agent.executor.base_driver import BaseDriver
from m3v_agent.executor.navigator import Navigator, S_ARRIVED
from m3v_agent.executor.target_poller import TargetPoller, parse_target_file
from m3v_agent.config import ExecutorCfg


class FakeDriver(BaseDriver):
    """In-memory driver: applies commanded velocity to an internal pose.

    A background tick thread integrates the last commanded velocity so motion
    happens even though the navigator only sees discrete move() calls.
    """

    def __init__(self, cfg=None, recorder=None):
        self.recorder = recorder
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        self._standing = True
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._tick, daemon=True)
        self._t.start()
        # Track all commands issued (for assertions).
        self.commands = []

    def _tick(self):
        """Integrate velocity at ~20 Hz so the pose actually moves."""
        dt = 0.05
        while not self._stop.is_set():
            with self._lock:
                # Body-frame velocity → world: rotate (vx,vy) by yaw.
                c, s = math.cos(self._yaw), math.sin(self._yaw)
                wx = c * self._vx - s * self._vy
                wy = s * self._vx + c * self._vy
                self._x += wx * dt
                self._y += wy * dt
                self._yaw = self._wrap_angle(self._yaw + self._vyaw * dt)
            time.sleep(dt)

    def stop_tick(self):
        self._stop.set()

    # --- BaseDriver impl ---
    def connect(self):
        return True

    def disconnect(self):
        self._stop.set()

    def stand_up(self):
        self._standing = True
        return True

    def lie_down(self):
        self._standing = False
        return True

    def move(self, vx, vy, yaw_rate):
        with self._lock:
            self._vx = float(vx)
            self._vy = float(vy)
            self._vyaw = float(yaw_rate)
            self.commands.append((vx, vy, yaw_rate))
        return True

    def stop(self):
        with self._lock:
            self._vx = self._vy = self._vyaw = 0.0
        return True

    def emergency_stop(self):
        self.stop()
        return True

    def get_pose(self):
        with self._lock:
            return (self._x, self._y, self._yaw)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.fixture
def fake():
    d = FakeDriver()
    yield d
    d.stop_tick()


def test_fake_driver_pose_moves_forward(fake):
    """move(1,0,0) for 0.5s should advance x by ~0.5m."""
    fake.move(1.0, 0.0, 0.0)
    time.sleep(0.5)
    fake.stop()
    time.sleep(0.1)
    x, y, _ = fake.get_pose()
    assert x > 0.3, f"expected forward motion, got x={x}"


def test_fake_driver_turns(fake):
    fake.move(0.0, 0.0, 1.0)
    time.sleep(0.5)
    fake.stop()
    _, _, yaw = fake.get_pose()
    assert abs(yaw) > 0.3, f"expected yaw change, got yaw={yaw}"


def test_navigator_arrives_at_target(fake):
    """Full navigator loop: drive from origin to (1.0, 0.0) and arrive."""
    cfg = ExecutorCfg(arrive_threshold=0.3, max_fwd=0.5, max_turn=1.0, turn_kp=1.2)
    nav = Navigator(fake, cfg)
    nav.start()
    try:
        nav.goto(1.0, 0.0)
        # Wait up to 8s for arrival.
        deadline = time.time() + 8.0
        while time.time() < deadline and not nav.arrived():
            time.sleep(0.1)
        assert nav.arrived(), "navigator did not arrive in time"
        x, y, _ = fake.get_pose()
        assert math.hypot(x - 1.0, y) < 0.35
    finally:
        nav.stop()


def test_navigator_arrives_at_diagonal_target(fake):
    """Drive to (-0.5, 0.8): requires turning + forward in a non-trivial dir."""
    cfg = ExecutorCfg(arrive_threshold=0.3, max_fwd=0.5, max_turn=1.0, turn_kp=1.2)
    nav = Navigator(fake, cfg)
    nav.start()
    try:
        nav.goto(-0.5, 0.8)
        deadline = time.time() + 10.0
        while time.time() < deadline and not nav.arrived():
            time.sleep(0.1)
        assert nav.arrived(), "navigator did not arrive at diagonal target"
        x, y, _ = fake.get_pose()
        assert math.hypot(x + 0.5, y - 0.8) < 0.35
    finally:
        nav.stop()


def test_target_poller_drives_robot_to_target(tmp_path, fake):
    """End-to-end: write a target file → poller → navigator → driver arrives.

    This exercises the real TargetPoller._tick path including mtime-based
    re-read + mode handling, then verifies the FakeDriver's pose ends up at
    the commanded local_x/local_y."""
    target_file = tmp_path / "ccenter_target.txt"
    cfg = ExecutorCfg(
        target_path=str(target_file),
        poll_interval=0.1,
        stale_timeout=1000.0,       # disable staleness for this test
        arrive_threshold=0.3,
        max_fwd=0.5,
        max_turn=1.0,
        turn_kp=1.2,
        target_deadband=0.05,
    )
    nav = Navigator(fake, cfg)
    poller = TargetPoller(cfg, nav)
    nav.start()
    poller.start()
    try:
        # Write an explore target.
        target_file.write_text(
            "mode: explore\nlocal_x: 1.0\nlocal_y: 0.0\nglobal_x: 1.0\nglobal_y: 0.0\n"
            "frame: 1\ntimestamp: test\n")
        deadline = time.time() + 10.0
        while time.time() < deadline and not nav.arrived():
            time.sleep(0.1)
        assert nav.arrived(), "robot didn't reach target via poller"
        x, y, _ = fake.get_pose()
        assert math.hypot(x - 1.0, y) < 0.35
    finally:
        poller.stop()
        nav.stop()


def test_target_poller_stop_mode_halts(tmp_path, fake):
    """mode: stop should abort the navigator (no new motion issued)."""
    target_file = tmp_path / "ccenter_target.txt"
    cfg = ExecutorCfg(target_path=str(target_file), poll_interval=0.1,
                      stale_timeout=1000.0)
    nav = Navigator(fake, cfg)
    poller = TargetPoller(cfg, nav)
    nav.start()
    poller.start()
    try:
        # Issue an explore first.
        target_file.write_text("mode: explore\nlocal_x: 5.0\nlocal_y: 0.0\n"
                               "global_x: 5.0\nglobal_y: 0.0\nframe: 1\ntimestamp: t\n")
        time.sleep(0.3)
        assert nav.target is not None
        # Now stop.
        target_file.write_text("mode: stop\nlocal_x: 0.0\nlocal_y: 0.0\n"
                               "global_x: 0.0\nglobal_y: 0.0\nframe: 2\ntimestamp: t\n")
        time.sleep(0.5)
        assert nav.target is None, "navigator should have aborted on stop mode"
    finally:
        poller.stop()
        nav.stop()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
