import numpy as np
import glob
import os
import json

DATA_ROOT = os.path.join(os.path.dirname(__file__), "data")

# A real cloud_registered frame is ~200KB+; anything smaller is a half-written
# file still being flushed by the recorder. We never read a truncated file —
# frames are index-aligned with odometry, so a partial array would corrupt the
# whole downstream pipeline. Locked/short files are retried on the NEXT poll
# tick by the main loop instead of blocking the (UI) thread.
MIN_NPY_BYTES = 1024


def _is_ready(path):
    """True if `path` is fully written and not held with an exclusive lock.

    On Windows the recorder holds an exclusive lock on the file it is writing;
    a half-flushed .npy is also smaller than MIN_NPY_BYTES. We probe both
    WITHOUT sleeping — the caller (main loop) retries next tick. This keeps the
    UI/window-message pump from starving (which is what crashed the app: a
    several-second block on a locked file made the Open3D window unresponsive
    and then native-crashed on the next geometry update)."""
    try:
        if os.path.getsize(path) < MIN_NPY_BYTES:
            return False
    except OSError:
        return False
    try:
        fd = os.open(path, os.O_WRONLY | os.O_APPEND)  # fails if exclusively locked
        os.close(fd)
        return True
    except (PermissionError, OSError):
        return False


def _try_load(path):
    """Load a .npy WITHOUT blocking. Returns (array | None).

    None means "not ready yet" (locked or partial) — the caller retries on a
    later poll tick. Never sleeps; never blocks the UI thread."""
    if not _is_ready(path):
        return None
    try:
        return np.load(path)
    except (ValueError, OSError):
        return None  # header present but corrupt — genuinely bad file


def _latest_run_dir(data_root=DATA_ROOT):
    """Pick the most recent run subdir. Returns None if data_root is missing
    or has no subdirs — caller is expected to gate on this."""
    if not os.path.isdir(data_root):
        return None
    try:
        subdirs = [d for d in os.listdir(data_root)
                   if os.path.isdir(os.path.join(data_root, d))]
    except OSError:
        return None
    if not subdirs:
        return None
    return max([os.path.join(data_root, d) for d in subdirs])


# Lazily-computed default. Use _latest_run_dir() explicitly when you need to
# handle missing data; the module-level DATA_DIR is kept for back-compat with
# callers that pass it as a default arg.
DATA_DIR = _latest_run_dir() or DATA_ROOT


def load_frames(data_dir=DATA_DIR):
    """Eager load ALL complete frames at startup. Locked/partial files (the one
    the recorder is currently writing) are silently skipped here — they get
    picked up by poll_new_frames once flushed. Non-blocking by design."""
    files = sorted(glob.glob(f"{data_dir}/cloud_registered/*.npy"))
    out = []
    for f in files:
        arr = _try_load(f)
        if arr is not None:
            out.append(arr)
    return out

def poll_new_frames(data_dir=DATA_DIR, known_count=0):
    """Incremental poll: load frames[known_count:] that are ready now.

    Returns (new_frames, total_files). Files not yet flushed are simply absent
    from new_frames; the caller keeps its known_count advancing only by how
    many it actually loaded (see poll_new_frames_nonblocking) so a locked file
    is retried next tick rather than skipped forever."""
    files = sorted(glob.glob(f"{data_dir}/cloud_registered/*.npy"))
    new = files[known_count:]
    out = []
    for f in new:
        arr = _try_load(f)
        if arr is None:
            break  # stop at first not-ready file — preserves index alignment
        out.append(arr)
    return out, len(files)

def poll_new_frames_nonblocking(data_dir=DATA_DIR, known_count=0):
    """Non-blocking incremental poll that preserves index alignment.

    Returns (new_frames, next_known_count). We read consecutive files from
    known_count; the moment one is locked/partial we STOP (don't skip it),
    advancing next_known_count only by the files actually loaded. The locked
    file is retried at the same index on the next call once flushed. This
    guarantees frames[i] keeps lining up with odom[i]."""
    files = sorted(glob.glob(f"{data_dir}/cloud_registered/*.npy"))
    out = []
    loaded = 0
    for f in files[known_count:]:
        arr = _try_load(f)
        if arr is None:
            break  # not ready — retry this same file next tick
        out.append(arr)
        loaded += 1
    return out, known_count + loaded

def _odom_jsonl_path(data_dir):
    """Path to the streaming odom JSONL file, or None if it doesn't exist."""
    p = os.path.join(data_dir, "Odometry", "odom_stream.jsonl")
    return p if os.path.exists(p) else None


def load_odometry(data_dir=DATA_DIR):
    # Prefer the streaming JSONL (one line per frame, appended by the remote
    # persistent SSH pipe) — it's far faster to read than thousands of tiny
    # JSON files. Fall back to per-frame .json files if no JSONL.
    jsonl = _odom_jsonl_path(data_dir)
    if jsonl:
        data = []
        try:
            with open(jsonl, "r", encoding="utf-8") as fh:
                for line in fh:
                    line = line.strip()
                    if line:
                        try:
                            data.append(json.loads(line))
                        except json.JSONDecodeError:
                            break  # partial last line — stop here
        except (PermissionError, OSError):
            pass
        return data
    # Per-frame JSON files (legacy / agibot)
    files = sorted(glob.glob(f"{data_dir}/Odometry/*.json"))
    data = []
    for f in files:
        try:
            with open(f) as fh:
                data.append(json.load(fh))
        except (PermissionError, OSError, json.JSONDecodeError):
            continue
    return data

def _odom_ready(path):
    """True if an odom JSON is fully written and not locked. On Windows, open()
    on a file the recorder holds with an exclusive lock can BLOCK for seconds
    (not raise), so we probe writability first — same trick as _is_ready for
    .npy files. This prevents the main loop from freezing."""
    try:
        fd = os.open(path, os.O_WRONLY | os.O_APPEND)
        os.close(fd)
        return True
    except (PermissionError, OSError):
        return False


def poll_new_odometry(data_dir=DATA_DIR, known_count=0):
    """Non-blocking incremental odom poll.

    Returns (new_odom, next_known_count). If a streaming JSONL exists, reads it
    from the last known line count (instant — one file read, seek to offset).
    Otherwise probes each per-frame .json file with _odom_ready before open()."""
    jsonl = _odom_jsonl_path(data_dir)
    if jsonl:
        # Read ALL lines from the JSONL; return those past known_count.
        # The file is append-only so re-reading is safe; we just slice.
        try:
            with open(jsonl, "r", encoding="utf-8") as fh:
                lines = fh.readlines()
        except (PermissionError, OSError):
            return [], known_count
        new_lines = lines[known_count:]
        out = []
        for line in new_lines:
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                break  # partial line — stop, retry next tick
        return out, known_count + len(out)
    # Per-frame JSON files (legacy / agibot)
    files = sorted(glob.glob(f"{data_dir}/Odometry/*.json"))
    out = []
    loaded = 0
    for f in files[known_count:]:
        if not _odom_ready(f):
            break  # locked/incomplete — retry this same file next tick
        try:
            with open(f) as fh:
                out.append(json.load(fh))
            loaded += 1
        except (PermissionError, OSError, json.JSONDecodeError):
            break  # not ready — retry this same file next tick
    return out, known_count + loaded
