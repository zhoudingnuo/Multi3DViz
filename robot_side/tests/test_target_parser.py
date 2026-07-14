"""test_target_parser.py — Verify target-file parsing + stop semantics.

The target file is written by the control side (Multi3DViz
explorer_service.py:233) as newline-separated `key: value` lines. Our parser
must handle: numeric fields, the mode string, stop-mode zeroed coords, and
robustly skip junk lines.
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from m3v_agent.executor.target_poller import parse_target_file


def test_parse_explore_target_all_fields():
    text = """mode: explore
global_x: 2.340
global_y: 1.080
local_x: 2.340
local_y: 1.080
frame: 116
timestamp: 2026-07-13 14:22:05
"""
    d = parse_target_file(text)
    assert d["mode"] == "explore"
    assert d["global_x"] == 2.340
    assert d["global_y"] == 1.080
    assert d["local_x"] == 2.340
    assert d["local_y"] == 1.080
    assert d["frame"] == 116
    assert d["timestamp"] == "2026-07-13 14:22:05"


def test_parse_stop_mode_zeroed():
    """stop mode: control side writes all coords as 0.000."""
    text = """mode: stop
global_x: 0.000
global_y: 0.000
local_x: 0.000
local_y: 0.000
frame: 0
timestamp: 2026-07-13 14:22:05
"""
    d = parse_target_file(text)
    assert d["mode"] == "stop"
    assert d["local_x"] == 0.0
    assert d["local_y"] == 0.0


def test_parse_negative_coords():
    """Robots explore in all quadrants — negative coords are valid."""
    text = """mode: explore
local_x: -1.500
local_y: -3.200
"""
    d = parse_target_file(text)
    assert d["local_x"] == -1.5
    assert d["local_y"] == -3.2


def test_parse_tolerates_junk_lines():
    text = """# comment line
mode: explore

malformed line without colon
local_x: 5.0
local_y: not_a_number
global_x: 5.0
"""
    d = parse_target_file(text)
    # Good fields survive.
    assert d["mode"] == "explore"
    assert d["local_x"] == 5.0
    assert d["global_x"] == 5.0
    # Unparseable numeric stays as raw string (caller can detect).
    assert d["local_y"] == "not_a_number"


def test_parse_empty_file():
    assert parse_target_file("") == {}
    assert parse_target_file("\n\n  \n") == {}


def test_parse_extra_whitespace():
    text = "mode:    explore\nlocal_x:   1.5 \n"
    d = parse_target_file(text)
    assert d["mode"] == "explore"
    assert d["local_x"] == 1.5


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
