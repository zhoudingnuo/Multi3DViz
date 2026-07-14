"""unitree_driver.py — Unitree Go2 driver (unitree_sdk2py SportClient, DDS).

Based on the official unitree_sdk2_python API (the public, maintained path).
`go2-search` (a community exploration wrapper) is not publicly visible, so we
target the underlying official SDK directly — this is the same transport layer
any Go2 exploration project uses.

Key API (unitree_sdk2py >= 1.x):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.go2.sport.sport_client import SportClient

    ChannelFactoryInitialize(0, network_iface)   # 0 = domain id
    client = SportClient()
    client.SetTimeout(10.0)
    client.Init()

Motions we map from BaseDriver:
    stand_up()   → client.StandUp()       (roSportMode_StandUp / stand up from prone)
    lie_down()   → client.Prone()          (drop to lying)
    move(vx,vy,yaw) → client.Move(vx, vy, yaw)
    stop()       → client.Move(0, 0, 0)
    emergency_stop() → client.StopMove() + client.Prone()

As with the Agibot driver, pose comes from the recorder's odom cache, not the
SDK's state — keeps both drivers identical and avoids needing to subscribe to
the low-level SportModeState DDS topic just for (x,y,yaw). See
docs/UNITREE_SDK_NOTES.md for the full state-subscription reference.
"""
from __future__ import annotations
import time
import logging
import threading

from ..executor.base_driver import BaseDriver

log = logging.getLogger("m3v_agent.driver.unitree")


class UnitreeDriver(BaseDriver):
    """unitree_sdk2py SportClient adapter for the Go2."""

    def __init__(self, cfg, recorder=None):
        self.cfg = cfg
        self.recorder = recorder
        self._client = None          # SportClient
        self._initialized = False
        self._standing = False
        self._lock = threading.Lock()

    # --- lifecycle ---
    def connect(self) -> bool:
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient
        except ImportError as e:
            log.error("unitree_sdk2py not installed: %s", e)
            log.error("pip install unitree_sdk2py cyclonedds  "
                      "(see docs/UNITREE_SDK_NOTES.md)")
            return False
        try:
            # Domain 0 is the Go2 default. Network iface selects which NIC
            # CycloneDDS binds to — must be the one reachable to the dog.
            iface = self.cfg.unitree_network_iface or None
            if iface:
                # The factory needs the iface name to be set via CYCLONEDDS_URI
                # OR passed as the second arg in newer versions. We set env so
                # both paths work; the explicit arg below is a no-op on old builds.
                import os
                os.environ.setdefault("CYCLONEDDS_URI", _dds_uri(iface))
            ChannelFactoryInitialize(0, iface or "")
            client = SportClient()
            client.SetTimeout(10.0)
            client.Init()
            self._client = client
            self._initialized = True
            log.info("unitree sport client initialized (iface=%s)", iface)
            return True
        except Exception:
            log.exception("unitree sport client init failed")
            return False

    def disconnect(self):
        self._client = None
        self._initialized = False
        self._standing = False

    # --- motion primitives ---
    def stand_up(self) -> bool:
        with self._lock:
            if self._client is None:
                return False
            try:
                # SportClient returns a code; 0 = success in recent SDKs.
                self._client.StandUp()
                time.sleep(2.0)   # let the stance stabilize
                self._standing = True
                log.info("StandUp ok")
                return True
            except Exception:
                log.exception("StandUp failed")
                return False

    def lie_down(self) -> bool:
        with self._lock:
            if self._client is None:
                return False
            try:
                self._client.Prone()
                self._standing = False
                log.info("Prone ok")
                return True
            except Exception:
                log.exception("Prone failed")
                return False

    def move(self, vx: float, vy: float, yaw_rate: float) -> bool:
        with self._lock:
            if self._client is None:
                return False
            if not self._standing:
                # Go2 Move requires the sport client to be in a moving-capable
                # mode; StandUp first if needed.
                try:
                    self._client.StandUp()
                    time.sleep(1.5)
                    self._standing = True
                except Exception:
                    log.exception("StandUp-before-move failed")
                    return False
            try:
                self._client.Move(float(vx), float(vy), float(yaw_rate))
                return True
            except Exception:
                log.exception("Move failed")
                return False

    def stop(self) -> bool:
        with self._lock:
            if self._client is None:
                return True
            try:
                self._client.Move(0.0, 0.0, 0.0)
                return True
            except Exception:
                log.exception("stop Move(0,0,0) failed")
                return False

    def emergency_stop(self) -> bool:
        with self._lock:
            if self._client is None:
                return False
            try:
                self._client.StopMove()
                self._client.Prone()
                self._standing = False
                log.warning("unitree emergency stop (StopMove + Prone)")
                return True
            except Exception:
                log.exception("emergency_stop failed")
                return False


def _dds_uri(iface: str) -> str:
    """Build a CycloneDDS config XML snippet pinning the NIC.

    Used as CYCLONEDDS_URI so the DDS stack binds to the interface that
    reaches the dog. The Go2's default network setup expects the control
    machine on the same L2 segment."""
    return (
        '<CycloneDDS><Domain><General>'
        f'<NetworkInterfaceAddress>{iface}</NetworkInterfaceAddress>'
        '<AllowMulticast>true</AllowMulticast>'
        '</General></Domain></CycloneDDS>'
    )
