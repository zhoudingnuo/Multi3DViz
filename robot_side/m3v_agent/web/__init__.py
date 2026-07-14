"""Web subpackage: ZCode-style status panel for the robot-side agent.

A tiny embedded HTTP server (stdlib only — no Flask/FastAPI dep) that serves:
  GET  /           → the status panel HTML
  GET  /api/state  → JSON snapshot of recorder/transport/executor/driver
  POST /api/estop  → trigger emergency stop on the driver

The panel mirrors the control-side Multi3DViz look (same zinc/gray + green
accent tokens from frontend/css/theme.css) so the two UIs feel like one app.
"""
from .status_server import StatusServer

__all__ = ["StatusServer"]
