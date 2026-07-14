"""atomic_io.py — Atomic file writes for the recorder.

This is the SINGLE most important robustness mechanism in the robot-side app.

Why: the control-side reader (Multi3DViz `backend/lib/player.py`) treats a
present-but-truncated `.npy` as a corrupt frame, which desyncs the strict
`frames[i] ↔ odom[i]` index alignment the whole pipeline depends on.

On Windows, `player.py:_is_ready` detects half-written files via an exclusive
lock probe (O_WRONLY|O_APPEND fails). On Linux there is no such lock signal —
a partial file just looks like a small file, and `np.load` happily returns a
truncated array. So on the robot we MUST publish files atomically:
write-to-temp + fsync + os.replace (POSIX rename is atomic on the same FS).

Contract every public function honors:
  - At NO observable instant does a reader see a partial file at the final path.
  - On crash mid-write, the temp file is orphaned and the final path still
    holds the previous good version (or doesn't exist yet).
"""
from __future__ import annotations
import os
import json
import errno
import logging
import numpy as np

log = logging.getLogger("m3v_agent.recorder.atomic")

# PIPE_BUF on Linux is 4096; writes <= this size to a pipe/FIFO are atomic.
# For regular files on local FS, a single write() of a few KB is effectively
# atomic too. Our JSONL lines are small (<1KB), so one write() call is safe.
# We still open in binary and write bytes-once to be explicit.


def _fsync_dir(path: str) -> None:
    """fsync the parent directory so the rename is durable across power loss.

    Best-effort: on read-only FSes or where the dir fd can't be opened we skip.
    The rename itself is still atomic even without the fsync."""
    d = os.path.dirname(path) or "."
    try:
        fd = os.open(d, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except (OSError, PermissionError) as e:
        log.debug("fsync_dir(%s) skipped: %s", d, e)


def save_npy_atomic(path: str, arr: np.ndarray) -> None:
    """Write `arr` to `path` atomically.

    1. Write to <path>.<pid>.tmp
    2. fsync the temp file (data durable)
    3. os.replace(tmp, path)  — atomic rename on POSIX
    4. fsync parent dir (rename durable)

    Readers polling `path` will either see the previous version or the new one,
    NEVER a half-written file. np.save is buffered, so we explicitly flush+fsync
    before the rename. Crash between steps leaves a stray .tmp (harmless).
    """
    tmp = f"{path}.{os.getpid()}.tmp"
    # Write + fsync the temp file. np.save produces the standard .npy header +
    # raw array bytes in one go.
    with open(tmp, "wb") as f:
        np.save(f, arr, allow_pickle=False)
        f.flush()
        os.fsync(f.fileno())
    # Atomic swap. os.replace overwrites the destination atomically on POSIX
    # and works even if `path` already exists (unlike os.rename which fails).
    try:
        os.replace(tmp, path)
    except OSError as e:
        # Clean up the orphan temp on failure so we don't litter.
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise RuntimeError(f"atomic replace {tmp} -> {path} failed: {e}") from e
    _fsync_dir(path)


def append_jsonl(path: str, obj: dict) -> None:
    """Append one JSON object as a line to `path`.

    A single open()/write()/close() cycle. The line is small (<PIPE_BUF for our
    odom records), so the write() is atomic at the kernel level — a reader
    tailing the file sees either the whole line or nothing. If a reader does
    hit a half-flushed line (rare race), it raises JSONDecodeError; the control
    side's player.py:140-141 catches that and retries on the next tick. So the
    contract holds even without fsync-per-line (which would kill throughput).

    We do NOT fsync per line — that would cap us at ~50 Hz. Instead we rely on
    the write() being one syscall and accept that a power loss could truncate
    the very last line (which the reader tolerates).
    """
    line = json.dumps(obj) + "\n"
    data = line.encode("utf-8")
    # O_APPEND guarantees the write goes to EOF atomically even with multiple
    # writers (we're the only writer, but the flag is cheap insurance).
    with open(path, "ab", buffering=0) as f:
        f.write(data)  # type: ignore[arg-type]


def read_jsonl_upto(path: str, max_lines: int = 0) -> list:
    """Read complete JSONL lines, stopping (tolerantly) at the first bad line.

    Mirrors the control side's poll_new_odometry behavior: a half-written last
    line is skipped, not fatal. Returns the list of parsed objects."""
    out = []
    if not os.path.exists(path):
        return out
    try:
        with open(path, "r", encoding="utf-8") as f:
            for i, line in enumerate(f):
                if max_lines and i >= max_lines:
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    out.append(json.loads(line))
                except json.JSONDecodeError:
                    # Half-written trailing line — stop here, reader will retry.
                    break
    except OSError as e:
        log.debug("read_jsonl %s failed: %s", path, e)
    return out


def save_json_atomic(path: str, obj: dict, indent: int = 2) -> None:
    """Write a JSON file atomically (same tmp+replace pattern as save_npy).

    Used for gravity_calibration.json."""
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=indent)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    _fsync_dir(path)


def ensure_dir(path: str) -> None:
    """mkdir -p, ignoring the 'already exists' error."""
    try:
        os.makedirs(path, exist_ok=True)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise
