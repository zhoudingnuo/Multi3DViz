"""
sem_infer.py — load trained UNet and predict semantic classes on a live grid.

ccenter's GridMap uses int8 with 0=free / 100=obstacle.
Training data uses 125=free / 255=obstacle. predict() translates conventions,
runs inference at 256x256, then resizes predictions back to source size.

Designed to run in a background thread — caller passes the current grid and
gets back a same-shaped int8 array of class indices (0=unset,
1=wall, 2=room, 3=corridor, 4=furniture).

OPTIONAL DEPS: torch + cv2 are imported lazily so this module loads even in a
slim build that excludes them (the packaged PyInstaller build drops torch to
keep the bundle <1G). If torch/cv2 are missing, SemPredictor.available is
False and predict() returns None — the caller (SemanticsService) treats that
as "semantics unavailable".
"""
from pathlib import Path

import numpy as np

MODEL_PATH = Path(__file__).parent / "models" / "unet_sem.pt"
INFER_SIZE = 256


def _try_import_deps():
    """Lazily import torch + cv2 + UNet. Returns (torch, F, cv2, UNet) or
    (None, None, None, None) if any are missing."""
    try:
        import torch
        import torch.nn.functional as F
        import cv2
        import sys
        train_dir = str(Path(__file__).parent / "train")
        if train_dir not in sys.path:
            sys.path.insert(0, train_dir)
        from unet import UNet
        return torch, F, cv2, UNet
    except ImportError:
        return None, None, None, None


class SemPredictor:
    def __init__(self, model_path=MODEL_PATH, device=None):
        self._loaded = False
        torch, F, cv2, UNet = _try_import_deps()
        if torch is None:
            print("[sem] torch/cv2 not available; predictor disabled (slim build)")
            self._torch = self._F = self._cv2 = self._net_cls = None
            return
        self._torch = torch
        self._F = F
        self._cv2 = cv2
        self._net_cls = UNet
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.net = UNet(in_ch=1, num_classes=4).to(self.device).eval()
        if Path(model_path).exists():
            try:
                state = torch.load(model_path, map_location=self.device)
                self.net.load_state_dict(state)
                self._loaded = True
            except Exception as e:
                print(f"[sem] failed to load {model_path}: {e}")
        else:
            print(f"[sem] model not found at {model_path}; predictor disabled")

    @property
    def available(self):
        return self._loaded

    def predict(self, grid_int8):
        """grid_int8: HxW int8 (-1/0/100). Returns HxW int8 class indices
        (0 = unknown, 1-6 = sem classes). If model not loaded, returns None."""
        if not self._loaded:
            return None
        torch = self._torch; cv2 = self._cv2
        H, W = grid_int8.shape
        if H < 4 or W < 4:
            return None

        # Crop to observed region with a small margin
        known = grid_int8 != 0  # 0 = unset/free
        if not known.any():
            return None
        rows = np.any(known, axis=1)
        cols = np.any(known, axis=0)
        rmin = int(np.argmax(rows)); rmax = int(len(rows) - np.argmax(rows[::-1]))
        cmin = int(np.argmax(cols)); cmax = int(len(cols) - np.argmax(cols[::-1]))
        pad = 8
        rmin = max(0, rmin - pad); rmax = min(H, rmax + pad)
        cmin = max(0, cmin - pad); cmax = min(W, cmax + pad)
        crop_h = rmax - rmin
        crop_w = cmax - cmin
        if crop_h < 4 or crop_w < 4:
            return None

        grid_crop = grid_int8[rmin:rmax, cmin:cmax]

        # Pad to square so aspect ratio is preserved during the resize.
        side = max(crop_h, crop_w)
        ph0 = (side - crop_h) // 2; ph1 = side - crop_h - ph0
        pw0 = (side - crop_w) // 2; pw1 = side - crop_w - pw0
        grid_sq = np.pad(grid_crop, ((ph0, ph1), (pw0, pw1)),
                         mode='constant', constant_values=0)

        # Map ccenter convention -> training convention.
        img = np.full((side, side), 125, dtype=np.float32)
        img[grid_sq == 100] = 255.0

        img_in = cv2.resize(img, (INFER_SIZE, INFER_SIZE), interpolation=cv2.INTER_LINEAR)
        with torch.no_grad():
            x = torch.from_numpy(img_in / 255.0).unsqueeze(0).unsqueeze(0).to(self.device)
            logits = self.net(x)
        pred_sq = logits.argmax(1).squeeze(0).byte().cpu().numpy()  # 256x256 in 0..3

        # Resize predictions back to square size, then strip padding.
        pred_full = cv2.resize(pred_sq, (side, side), interpolation=cv2.INTER_NEAREST)
        pred_crop = pred_full[ph0:ph0 + crop_h, pw0:pw0 + crop_w]

        # Place into a full-grid output (0 elsewhere, +1 to shift to 1..4 convention).
        out = np.zeros((H, W), dtype=np.int8)
        crop_out = (pred_crop.astype(np.int8) + 1)
        crop_out[grid_crop == 0] = 0
        out[rmin:rmax, cmin:cmax] = crop_out
        return out
