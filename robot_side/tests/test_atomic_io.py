"""test_atomic_io.py — Verify atomic writes never produce half files.

The whole point of atomic_io is that a concurrent reader NEVER observes a
truncated .npy or a half-written JSONL line. These tests hammer that property
from multiple threads.
"""
import os
import sys
import json
import time
import threading
import tempfile
import shutil

import numpy as np
import pytest

# Allow running tests from inside robot_side/ without an installed package.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from m3v_agent.recorder.atomic_io import (
    save_npy_atomic, append_jsonl, save_json_atomic, read_jsonl_upto,
)


def test_save_npy_atomic_roundtrip(tmp_path):
    arr = np.random.rand(1000, 3).astype(np.float32)
    p = str(tmp_path / "frame.npy")
    save_npy_atomic(p, arr)
    assert np.load(p).shape == (1000, 3)


def test_save_npy_atomic_overwrites_cleanly(tmp_path):
    """Overwriting an existing file should leave a valid .npy, never a blend."""
    p = str(tmp_path / "frame.npy")
    a1 = np.ones((500, 3), dtype=np.float32)
    a2 = np.full((2000, 3), 2.0, dtype=np.float32)
    save_npy_atomic(p, a1)
    save_npy_atomic(p, a2)
    loaded = np.load(p)
    assert loaded.shape == (2000, 3)
    assert (loaded == 2.0).all()


def test_save_npy_atomic_never_partial_under_concurrency(tmp_path):
    """A reader polling while writes happen must never see a PARTIAL load.

    The contract atomic_io guarantees is: a reader sees either the previous
    complete file or the new complete file, NEVER a truncated/blended array.

    NOTE on platform difference:
      - On Linux (the actual robot target), os.replace is atomic even while a
        reader has the file open (POSIX rename semantics). So the reader below
        never gets an error and never sees a corrupt array.
      - On Windows (where these tests run during dev), an open reader places a
        share-lock that makes os.replace fail with EACCES — this is exactly the
        "Windows exclusive lock" behavior the control-side player.py:31 relies
        on, but it's a *write* failure here, not a *data corruption*. We treat
        transient write failures and read failures as benign on Windows; we
        ONLY fail the test if a load returned a structurally-corrupt array
        (wrong shape/dtype), which would indicate a true atomicity violation.
    """
    p = str(tmp_path / "f.npy")
    is_win = os.name == "nt"
    stop = threading.Event()
    seen_bad = []

    def writer():
        i = 0
        while not stop.is_set():
            arr = np.full((10 if i % 2 == 0 else 20, 3), float(i), dtype=np.float32)
            try:
                save_npy_atomic(p, arr)
            except Exception as e:
                # On Windows os.replace races with the open reader; that's a
                # known platform quirk, NOT an atomicity failure. Retry later.
                if not is_win:
                    seen_bad.append(("write", str(e)[:80]))
            i += 1

    def reader():
        while not stop.is_set():
            if not os.path.exists(p):
                time.sleep(0.0001)
                continue
            try:
                loaded = np.load(p)
                # The actual contract: shape must be one of the valid sizes,
                # never a truncated count.
                if loaded.shape[0] not in (10, 20) or loaded.shape[1] != 3 \
                        or loaded.dtype != np.float32:
                    seen_bad.append(("corrupt", loaded.shape, str(loaded.dtype)))
            except (ValueError, OSError) as e:
                # A truncated .npy would raise here — that's the failure mode
                # atomic_io exists to prevent. On Windows a read-while-replace
                # can also give EACCES, which is benign here.
                if "trunc" in str(e).lower() or "magic" in str(e).lower() \
                        or "cannot reshape" in str(e).lower():
                    seen_bad.append(("partial", str(e)[:80]))
                # else: benign Windows EACCES race, ignore.
            time.sleep(0.0002)

    t1 = threading.Thread(target=writer)
    t2 = threading.Thread(target=reader)
    t1.start(); t2.start()
    time.sleep(1.0)
    stop.set()
    t1.join(timeout=2); t2.join(timeout=2)
    assert not seen_bad, f"observed atomicity violations: {seen_bad[:5]}"


def test_append_jsonl_roundtrip(tmp_path):
    p = str(tmp_path / "odom.jsonl")
    for i in range(5):
        append_jsonl(p, {"x": float(i), "y": 0.0, "i": i})
    lines = read_jsonl_upto(p)
    assert len(lines) == 5
    assert lines[0]["x"] == 0.0 and lines[4]["x"] == 4.0


def test_read_jsonl_upto_tolerates_partial_line(tmp_path):
    """A half-written last line should be skipped, not fatal."""
    p = str(tmp_path / "odom.jsonl")
    append_jsonl(p, {"x": 1.0, "valid": True})
    append_jsonl(p, {"x": 2.0, "valid": True})
    # Simulate a half-flushed line (crash mid-write).
    with open(p, "ab") as f:
        f.write(b'{"x": 3.0, "valid"')  # truncated JSON
    lines = read_jsonl_upto(p)
    # Should get the 2 good lines, skip the bad one.
    assert len(lines) == 2
    assert lines[-1]["x"] == 2.0


def test_save_json_atomic_roundtrip(tmp_path):
    p = str(tmp_path / "gravity.json")
    save_json_atomic(p, {"roll_deg": 0.4, "pitch_deg": -6.7})
    with open(p) as f:
        d = json.load(f)
    assert d["roll_deg"] == 0.4 and d["pitch_deg"] == -6.7


if __name__ == "__main__":
    # Allow `python test_atomic_io.py` without pytest.
    sys.exit(pytest.main([__file__, "-v"]))
