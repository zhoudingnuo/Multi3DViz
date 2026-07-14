# -*- mode: python ; coding: utf-8 -*-
# m3v_backend.spec — PyInstaller spec for the Multi3DViz control-side backend.
#
# Freezes backend/main.py + all of backend/{core,plugins,lib} into a single
# --onedir bundle that electron-builder packages as extraResources. The frozen
# entry dist/m3v_backend/m3v_backend.exe is what electron/main.js spawns in
# packed mode (see PACKAGED_BACKEND in electron/main.js).
#
# Build:
#   .venv/Scripts/python.exe -m PyInstaller m3v_backend.spec --noconfirm
#   (or: npm run pack:backend)
#
# Output: dist/m3v_backend/m3v_backend(.exe) + dist/m3v_backend/_internal/

import os
import sys
from PyInstaller.utils.hooks import collect_data_files, collect_submodules

block_cipher = None
ROOT = os.path.abspath('.')

# PyInstaller 6.x: collect_data_files returns 2-tuples (src, dest); Analysis
# normalizes them to 3-tuples internally. Mixing manual 2-tuples with
# collected 2-tuples at `a.datas += ...` time breaks COLLECT's normalization.
# So we gather everything into ONE datas list up front.
_datas = [
    ('backend/plugins', 'backend/plugins'),
    ('backend/core',    'backend/core'),
    ('backend/lib',     'backend/lib'),   # includes models/unet_sem.pt
    ('backend/__init__.py', 'backend'),
]
# open3d ships bundled resource files (DLLs, configs) loaded at runtime.
# (torch is excluded — slim build.)
_datas += collect_data_files('open3d')


a = Analysis(
    ['backend/main.py'],
    pathex=[ROOT, os.path.join(ROOT, 'backend')],
    binaries=[],
    datas=_datas,
    hiddenimports=[
        # Heavy deps PyInstaller's static analysis sometimes misses.
        'open3d',
        'scipy', 'scipy.special',
        'numpy', 'paramiko', 'websockets',
        # pip module API — used by optional_deps.install() for in-app
        # "Install torch" feature. pip is pure Python so it freezes fine.
        'pip', 'pip._internal', 'pip._internal.cli.main',
        # Plugins are imported dynamically (by name) by the registry — list
        # every plugin so PyInstaller freezes them.
        'backend.plugins.source.local_replay',
        'backend.plugins.display.point_cloud',
        'backend.plugins.display.grid_map',
        'backend.plugins.service.icp_registration',
        'backend.plugins.service.explorer_service',
        'backend.plugins.service.semantics',
        'backend.plugins.service.ssh_launcher',
        'backend.plugins.service.connection_monitor',
        # ccenter-reused lib modules.
        'backend.lib.gridmap', 'backend.lib.data_utils', 'backend.lib.player',
        'backend.lib.registration', 'backend.lib.explorer',
        'backend.lib.sem_infer', 'backend.lib.room_detect',
        'backend.lib.trajectory_plot',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        # === Slim-build exclusions (drop from 7.7G → ~600M) ===
        # torch is ONLY used by sem_infer.py (UNet semantic segmentation,
        # an optional feature — SemanticsService defaults off). sem_infer now
        # imports it lazily and reports unavailable when missing. torch+CUDA
        # alone is 4.3G of the bundle.
        'torch',
        # matplotlib — used only by trajectory_plot (PNG export). Lazy-imported.
        'matplotlib',
        # === Hermes-venv cruft (not used by backend at all) ===
        'polars', '_polars_runtime_32',
        'llvmlite', 'numba',
        'PyQt5', 'PyQt6', 'PySide2', 'PySide6',
        'dash', 'plotly', 'pandas',
        'IPython', 'notebook', 'jupyter', 'jupyter_client', 'jupyter_core',
        'pytest', 'sphinx',
        # === Build-breaking / optional ===
        # onnx/onnx.reference crashes PyInstaller's import probe (segfault).
        'onnx', 'onnxruntime', 'onnx_pybind_utils',
        # pywin32 broken in venv (pywintypes DLL missing).
        'pywin32', 'pythoncom', 'pywintypes', 'win32api', 'win32com',
        'win32con', 'win32event', 'win32evtlog', 'win32file', 'win32process',
        'win32security', 'win32service', 'winerror', 'ntsecuritycon',
    ],
    cipher=block_cipher,
)

# Open3D + torch's bundled resource files are already in _datas above
# (collected at Analysis() time so they go through proper normalization).

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='m3v_backend',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,            # UPX on torch/open3d DLLs corrupts them — leave off.
    console=True,         # backend reads/writes stdout — must be console app
    disable_windowed_traceback=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name='m3v_backend',
)
