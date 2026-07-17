"""odom_file_pose.py — file-backed pose provider for split-process deployment.

In the split deployment (Agibot ROS1), the recorder (rospy) runs inside the
noetic container while the driver (mc_sdk) runs on the host. They are separate
processes, so the driver cannot read the recorder's in-memory odom cache.

The bridge is the odom_stream.jsonl file the recorder already writes — one JSON
object per line (x, y, yaw, qx..qw, stamp). This provider tails that file and
caches the most recent valid pose. Reading only the incremental tail (bytes
appended since last read) keeps it cheap at 10 Hz / thousands of lines.

Thread-safety: the provider is read from the navigator loop (one thread) and
self-guarded with a lock. The file is written by the recorder in another
process; a torn last line (read mid-write) is detected by JSON parse failure and
skipped — the previous cached pose is returned instead.
"""
from __future__ import annotations
import json
import logging
import os
import threading
from typing import Optional

log = logging.getLogger("m3v_agent.executor.odom_file")


class OdomFilePoseProvider:
    """Tail an odom_stream.jsonl file and expose the latest (x, y, yaw).

    Set this on a driver's `odom_file_pose` attribute as a fallback for when
    `driver.recorder` is None (split-process execute mode). The base driver's
    get_pose() prefers the in-memory recorder; if absent it falls back here.
    """

    def __init__(self, odom_path: str):
        self.odom_path = odom_path
        self._lock = threading.Lock()
        self._last_size = 0          # bytes read so far
        self._last_pose: Optional[tuple] = None   # (x, y, yaw)

    def latest_pose(self) -> Optional[tuple]:
        """Return (x, y, yaw) from the last valid odom line, or None if no
        odom has been seen yet. Returns the cached pose if the file is
        unchanged or the latest line is unreadable (torn write)."""
        try:
            size = os.path.getsize(self.odom_path)
        except OSError:
            # File not created yet (recorder hasn't received the first frame).
            return self._last_pose
        with self._lock:
            # If the file shrank (new run dir, rotated) re-read from the tail.
            if size < self._last_size:
                self._last_size = 0
            if size == self._last_size:
                return self._last_pose  # unchanged since last read
            delta = size - self._last_size
            try:
                with open(self.odom_path, "rb") as fh:
                    # Seek so we read at most the newly-appended bytes, plus a
                    # little overlap in case the last read ended mid-line.
                    overlap = 0
                    seek_to = 0
                    if self._last_size > 0:
                        overlap = min(self._last_size, 2048)
                        seek_to = self._last_size - overlap
                        delta += overlap
                    fh.seek(seek_to)
                    chunk = fh.read(delta)
            except OSError:
                return self._last_pose
            # If we overlapped into the previous read, the chunk starts with a
            # partial line — drop everything up to the first newline so we only
            # parse whole lines. When reading from offset 0 (fresh/rotated
            # file) the first line is complete, so don't skip it.
            text = chunk.decode("utf-8", errors="replace")
            if overlap > 0:
                first_nl = text.find("\n")
                if first_nl >= 0:
                    text = text[first_nl + 1:]
            # Parse each line; keep the last valid one. Torn final line (no
            # trailing newline) is parsed too since append_jsonl writes a full
            # JSON object then a newline atomically per line — but if a read
            # catches it half-written, json.loads raises and we skip it.
            pose = self._last_pose
            for line in text.splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    d = json.loads(line)
                except json.JSONDecodeError:
                    continue  # torn write — skip, keep previous
                if {"x", "y"} <= set(d):
                    pose = (float(d["x"]), float(d["y"]), float(d.get("yaw", 0.0)))
            self._last_pose = pose
            self._last_size = size
            return pose
