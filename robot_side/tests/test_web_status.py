"""test_web_status.py — Verify the embedded status panel server.

Starts the real StatusServer on a random port, hits all 4 endpoints
(GET /, GET /app.js, GET /api/state, POST /api/estop), and checks the
HTML actually contains the ZCode-style panel + the JS boots. No browser
needed — this is a server contract test.
"""
import os
import sys
import time
import json
import socket
import urllib.request

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from m3v_agent.web.status_server import StatusServer


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]
    finally:
        s.close()


@pytest.fixture
def server():
    estop_hit = {"v": False}
    port = _free_port()

    def snap():
        return {"mode": "both", "robot": {"robot_id": "test_bot"},
                "recorder": {"frame_idx": 7, "enabled": True,
                              "latest_pose": {"x": 1.5, "y": -0.5, "yaw": 0.3}},
                "executor": {"nav_state": "drive_forward", "enabled": True,
                             "current_target": None}}

    def estop():
        estop_hit["v"] = True
        return True

    srv = StatusServer(host="127.0.0.1", port=port, snapshot=snap, on_estop=estop)
    srv.start()
    time.sleep(0.2)
    yield srv, port, estop_hit
    srv.stop()


def _base(port):
    return f"http://127.0.0.1:{port}"


def test_index_html_has_panel(server):
    srv, port, _ = server
    html = urllib.request.urlopen(f"{_base(port)}/").read().decode("utf-8")
    # The panel must carry the ZCode token identity + the emergency button.
    assert "m3v-agent" in html
    assert "Emergency Stop" in html
    # Tokens match the control-side theme.css.
    assert "#4ec9b0" in html            # --accent
    assert "#1e1e1e" in html            # --bg


def test_app_js_served(server):
    srv, port, _ = server
    js = urllib.request.urlopen(f"{_base(port)}/app.js").read().decode("utf-8")
    assert "fetch" in js
    assert "/api/state" in js
    assert "/api/estop" in js


def test_state_endpoint_returns_snapshot_plus_server(server):
    srv, port, _ = server
    state = json.loads(urllib.request.urlopen(f"{_base(port)}/api/state").read())
    # Snapshot data passes through.
    assert state["recorder"]["frame_idx"] == 7
    assert state["robot"]["robot_id"] == "test_bot"
    # The server injects its own metadata block.
    assert "server" in state
    assert "uptime_s" in state["server"]
    assert state["server"]["uptime_s"] >= 0


def test_estop_endpoint_fires_callback(server):
    srv, port, estop_hit = server
    req = urllib.request.Request(f"{_base(port)}/api/estop", method="POST")
    resp = json.loads(urllib.request.urlopen(req).read())
    assert resp["ok"] is True
    assert estop_hit["v"] is True


def test_404_for_unknown_path(server):
    srv, port, _ = server
    try:
        urllib.request.urlopen(f"{_base(port)}/nope")
        assert False, "should have raised"
    except urllib.error.HTTPError as e:
        assert e.code == 404


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
