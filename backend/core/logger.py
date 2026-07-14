"""logger.py — crash logging + process monitoring for Multi3DViz.

Adapted from ccenter/logger.py:
  - mem_mb() / cpu_pct(): psutil-based resource readouts for the status bar
  - install_excepthooks(): catch uncaught exceptions (main thread + threading
    module) and write a crash log with stack trace + environment, so a crash
    is never silent. Crash logs land in logs/crash_<timestamp>.log.
  - log(): append a timestamped line to logs/multi3dviz.log (rotated by size).

Crash logs include version + env so a user can attach them to a bug report.
"""
from __future__ import annotations
import os
import sys
import time
import traceback
import platform
import logging

_LOG_DIR = os.environ.get(
    "M3V_LOG_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs"))
os.makedirs(_LOG_DIR, exist_ok=True)

_VERSION = "0.1.0"

# psutil is optional — if missing, mem/cpu just report 0.
try:
    import psutil
    _PROC = psutil.Process(os.getpid())
except ImportError:
    psutil = None
    _PROC = None


def mem_mb() -> float:
    """RSS of this process in MB, or 0 if psutil unavailable."""
    if _PROC is None:
        return 0.0
    try:
        return _PROC.memory_info().rss / (1024 * 1024)
    except Exception:
        return 0.0


def cpu_pct() -> float:
    """CPU% of this process (cumulative since last call), or 0."""
    if _PROC is None:
        return 0.0
    try:
        return _PROC.cpu_percent(interval=None)
    except Exception:
        return 0.0


def _write_crash(kind: str, exc_type, exc_value, tb) -> str:
    """Write a crash log file with full context. Returns the path."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(_LOG_DIR, f"crash_{kind}_{ts}.log")
    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write(f"Multi3DViz crash log — {kind}\n")
            f.write(f"timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"version: {_VERSION}\n")
            f.write(f"python: {sys.version.split()[0]}\n")
            f.write(f"platform: {platform.platform()}\n")
            f.write(f"pid: {os.getpid()}\n")
            f.write("\n=== traceback ===\n")
            traceback.print_exception(exc_type, exc_value, tb, file=f)
            f.write("\n=== last log lines ===\n")
            try:
                main_log = os.path.join(_LOG_DIR, "multi3dviz.log")
                with open(main_log, "r", encoding="utf-8") as lf:
                    lines = lf.readlines()[-30:]
                    f.writelines(lines)
            except Exception:
                pass
    except Exception:
        pass  # never let crash-logging itself crash
    return path


def install_excepthooks() -> None:
    """Install sys.excepthook + threading.excepthook so uncaught exceptions
    are logged to a crash file instead of vanishing (especially in the
    frozen/packed build where there's no console)."""
    import threading

    def _sys_hook(exc_type, exc_value, tb):
        # KeyboardInterrupt is a normal exit, not a crash.
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc_value, tb)
            return
        path = _write_crash("main", exc_type, exc_value, tb)
        # Still print to stderr for dev visibility.
        sys.__excepthook__(exc_type, exc_value, tb)
        print(f"[CRASH] logged to {path}", file=sys.stderr)

    def _thread_hook(args):
        path = _write_crash("thread", args.exc_type, args.exc_value, args.exc_traceback)
        traceback.print_exception(args.exc_type, args.exc_value, args.exc_traceback)
        print(f"[CRASH] thread {args.thread.name} -> {path}", file=sys.stderr)

    sys.excepthook = _sys_hook
    threading.excepthook = _thread_hook
