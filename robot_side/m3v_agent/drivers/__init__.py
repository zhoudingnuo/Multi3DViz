"""Drivers subpackage: per-robot SDK adapters.

Each driver implements m3v_agent.executor.base_driver.BaseDriver so the
navigator/target_poller stay robot-agnostic. Two templates ship:

  - AgibotDriver : mc_sdk_zsl_1_py (UDP 43988), Agibot D1 Edu-Ultra
  - UnitreeDriver : unitree_sdk2py SportClient (CycloneDDS), Unitree Go2
"""
from .agibot_driver import AgibotDriver
from .unitree_driver import UnitreeDriver


def make_driver(kind: str, driver_cfg, recorder=None):
    """Factory: build a driver by `kind` ("agibot"|"unitree"|"fake")."""
    k = (kind or "").lower()
    if k == "agibot":
        return AgibotDriver(driver_cfg, recorder)
    if k == "unitree":
        return UnitreeDriver(driver_cfg, recorder)
    if k == "fake":
        # Late import so the test-only driver doesn't load in production.
        from tests.test_fake_driver import FakeDriver
        return FakeDriver(driver_cfg, recorder)
    raise ValueError(f"unknown driver kind: {kind!r}")


__all__ = ["AgibotDriver", "UnitreeDriver", "make_driver"]
