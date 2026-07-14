"""optional_deps.py — In-app optional dependency installation.

The slim PyInstaller build excludes heavy optional deps (torch = 4.3G) to keep
the bundle at ~489M. This module lets users install them ON DEMAND from the UI:
click "Install" → pip downloads CPU-only torch → the SemanticsService plugin
becomes functional.

Key design point: PyInstaller onedir's sys.path points at read-only _MEIPASS,
NOT user site-packages. So we install into a writable dir (userData/torch_runtime/)
and insert it onto sys.path at startup. pip's --target flag does exactly this.

pip is invoked via its Python module API (pip._internal.cli.main.main) which
works inside a frozen bundle because pip is pure Python — no subprocess to an
external interpreter needed. The spec must list pip as a hiddenimport.
"""
from __future__ import annotations
import os
import sys
import logging
import threading
import importlib.util
from typing import Callable, Optional

log = logging.getLogger("multi3dviz.optional_deps")

# Map plugin name → list of pip-installable packages it needs but the slim
# build doesn't ship. When a plugin's catalog entry shows missing_deps, the
# frontend renders an "Install" button.
PLUGIN_DEPS: dict[str, list[str]] = {
    "Semantics": ["torch"],
}

# CPU-only PyTorch index (no CUDA = ~200M instead of ~2.5G).
_PIP_INDEX_URLS = {
    "torch": "https://download.pytorch.org/whl/cpu",
}


def _runtime_dir() -> str:
    """Writable dir for pip --target installs. Lives in userData (frozen) or
    a local dir (dev) so it survives app restarts."""
    if getattr(sys, "frozen", False):
        # appdata dir — e.g. %APPDATA%/Multi3DViz on Windows
        base = os.environ.get("APPDATA") or os.path.expanduser("~")
        app_dir = os.path.join(base, "Multi3DViz")
    else:
        app_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    return os.path.join(app_dir, "torch_runtime")


def setup_runtime_path():
    """Insert the torch_runtime dir onto sys.path at startup so previously-
    installed packages are importable. Call this BEFORE any import that might
    need them (i.e. very early in main.py, before plugin discovery)."""
    d = _runtime_dir()
    if os.path.isdir(d) and d not in sys.path:
        sys.path.insert(0, d)
        log.info("optional_deps: torch_runtime on sys.path: %s", d)


def is_installed(pkg: str) -> bool:
    """True if `pkg` is importable right now (either in the bundle or in the
    torch_runtime dir that's on sys.path). Uses importlib.util.find_spec which
    is the canonical check."""
    try:
        return importlib.util.find_spec(pkg) is not None
    except (ImportError, ValueError):
        return False


def get_missing(plugin_name: str) -> list[str]:
    """Return the subset of plugin_name's deps that aren't currently installed.
    Empty list = plugin is fully functional (or has no optional deps)."""
    deps = PLUGIN_DEPS.get(plugin_name, [])
    return [d for d in deps if not is_installed(d)]


# --- install machinery ---

# Current install state (surfaced via install_status WS events).
_install_lock = threading.Lock()
_current: Optional[dict] = None  # {"pkg": str, "phase": str, "pct": float, "msg": str}


def _set_state(pkg: str, phase: str, pct: float = 0, msg: str = ""):
    global _current
    with _install_lock:
        _current = {"pkg": pkg, "phase": phase, "pct": round(pct, 1), "msg": msg}


def clear_state():
    global _current
    with _install_lock:
        _current = None


def state_snapshot() -> Optional[dict]:
    with _install_lock:
        return dict(_current) if _current else None


def install(pkg: str,
            on_progress: Optional[Callable[[dict], None]] = None,
            on_done: Optional[Callable[[bool, str], None]] = None):
    """Install `pkg` via pip into the torch_runtime dir. Runs in a daemon
    thread so the caller (WS handler) returns immediately.

    on_progress: called with {phase, pct, msg} dicts during install.
    on_done:     called with (success: bool, message: str) when finished.
    """
    def _emit(phase, pct=0, msg=""):
        _set_state(pkg, phase, pct, msg)
        if on_progress:
            try:
                on_progress({"pkg": pkg, "phase": phase, "pct": round(pct, 1), "msg": msg})
            except Exception:
                log.exception("on_progress callback failed")

    def _worker():
        _emit("starting", 0, f"Preparing to install {pkg}...")
        try:
            # Late import — pip must be in the bundle (spec hiddenimport).
            from pip._internal.cli.main import main as pip_main
        except ImportError:
            _emit("error", 0, "pip not available in this build")
            if on_done:
                on_done(False, "pip module not bundled — cannot install")
            return

        target = _runtime_dir()
        os.makedirs(target, exist_ok=True)

        args = ["install", "--target", target, "--no-warn-script-location"]
        index_url = _PIP_INDEX_URLS.get(pkg)
        if index_url:
            args += ["--index-url", index_url, "--trusted-host", "download.pytorch.org"]
        else:
            args += ["--trusted-host", "pypi.org", "--trusted-host", "files.pythonhosted.org"]
        args.append(pkg)

        _emit("downloading", 10, f"Downloading {pkg} (CPU-only)...")
        log.info("pip %s", " ".join(args))

        # Capture pip's stdout+stderr so we can report the real error message
        # if it fails (the default pip_main prints to sys.stdout/stderr which
        # in a frozen GUI build goes nowhere). Redirect to a StringIO.
        import io
        from contextlib import redirect_stdout, redirect_stderr
        buf_out, buf_err = io.StringIO(), io.StringIO()
        try:
            with redirect_stdout(buf_out), redirect_stderr(buf_err):
                rc = pip_main(args)
        except SystemExit as e:
            rc = int(e.code) if e.code is not None else 1
        except Exception as e:
            _emit("error", 0, f"pip crashed: {e}")
            if on_done:
                on_done(False, str(e))
            return

        if rc != 0:
            err_text = (buf_err.getvalue() + buf_out.getvalue()).strip()[-300:]
            _emit("error", 0, f"pip exit {rc}: {err_text[:200]}")
            if on_done:
                on_done(False, f"pip install failed (exit {rc}): {err_text[:200]}")
            return

        # Success — make sure the new package is importable.
        if target not in sys.path:
            sys.path.insert(0, target)
        # Invalidate import caches so find_spec sees the new package.
        importlib.invalidate_caches()

        if is_installed(pkg):
            _emit("done", 100, f"{pkg} installed successfully")
            if on_done:
                on_done(True, f"{pkg} installed")
        else:
            _emit("error", 0, f"{pkg} installed but not importable")
            if on_done:
                on_done(False, f"{pkg} installed but import failed")

    t = threading.Thread(target=_worker, daemon=True, name=f"pip-install-{pkg}")
    t.start()
