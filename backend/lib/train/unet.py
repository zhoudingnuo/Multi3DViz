"""
unet.py — compact UNet for grid-map semantic segmentation + dataset class.

Input: 1-channel grid (125=free / 255=wall, normalized to [0,1]).
Output: 6-class logits (1=floor, 2=wall, 3=room, 4=corridor, 5=plaza, 6=longwall).

Model size kept small (32/64/128/256/512 channels) because we have only 100
samples — a heavy model would overfit. Trains on 256x256 patches at batch 8.
"""
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset


class DoubleConv(nn.Module):
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.block = nn.Sequential(
            nn.Conv2d(in_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_ch, out_ch, 3, padding=1, bias=False),
            nn.BatchNorm2d(out_ch),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.block(x)


class Down(nn.Module):
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.pool = nn.MaxPool2d(2)
        self.conv = DoubleConv(in_ch, out_ch)

    def forward(self, x):
        return self.conv(self.pool(x))


class Up(nn.Module):
    def __init__(self, in_ch, out_ch):
        super().__init__()
        self.up = nn.ConvTranspose2d(in_ch, in_ch // 2, 2, stride=2)
        self.conv = DoubleConv(in_ch, out_ch)

    def forward(self, x, skip):
        x = self.up(x)
        # Pad in case sizes mismatch by 1px due to rounding
        dy = skip.size(2) - x.size(2)
        dx = skip.size(3) - x.size(3)
        x = F.pad(x, [dx // 2, dx - dx // 2, dy // 2, dy - dy // 2])
        x = torch.cat([skip, x], dim=1)
        return self.conv(x)


class UNet(nn.Module):
    def __init__(self, in_ch=1, num_classes=6, base=32):
        super().__init__()
        b = base
        self.inc = DoubleConv(in_ch, b)
        self.d1 = Down(b, b * 2)
        self.d2 = Down(b * 2, b * 4)
        self.d3 = Down(b * 4, b * 8)
        self.d4 = Down(b * 8, b * 16)
        self.u1 = Up(b * 16, b * 8)
        self.u2 = Up(b * 8, b * 4)
        self.u3 = Up(b * 4, b * 2)
        self.u4 = Up(b * 2, b)
        self.outc = nn.Conv2d(b, num_classes, 1)

    def forward(self, x):
        x1 = self.inc(x)
        x2 = self.d1(x1)
        x3 = self.d2(x2)
        x4 = self.d3(x3)
        x5 = self.d4(x4)
        x = self.u1(x5, x4)
        x = self.u2(x, x3)
        x = self.u3(x, x2)
        x = self.u4(x, x1)
        return self.outc(x)


class SemMapDataset(Dataset):
    """Loads grid + synthesized sem pairs, resizes to a fixed square,
    applies random D4 symmetry augmentation (rotations/flips — maze maps
    have no canonical orientation)."""

    def __init__(self, items, size=256, augment=False):
        self.items = items  # list of (grid_path, sem_path)
        self.size = size
        self.augment = augment

    def __len__(self):
        return len(self.items)

    def __getitem__(self, idx):
        grid_path, sem_path = self.items[idx]
        grid = np.load(grid_path).astype(np.float32) / 255.0
        sem = np.load(sem_path).astype(np.int64) - 1  # 1..6 -> 0..5

        grid = self._resize(grid, order=1)
        sem = self._resize(sem, order=0)

        if self.augment:
            grid, sem = self._augment(grid, sem)

        grid_t = torch.from_numpy(grid).unsqueeze(0)  # 1xHxW
        sem_t = torch.from_numpy(sem).long()          # HxW
        return grid_t, sem_t

    def _resize(self, arr, order):
        import cv2
        return cv2.resize(arr, (self.size, self.size), interpolation={
            0: cv2.INTER_NEAREST, 1: cv2.INTER_LINEAR
        }[order])

    def _augment(self, grid, sem):
        # Random 90° rotation k times, then optional h/v flip.
        k = np.random.randint(0, 4)
        if k:
            grid = np.rot90(grid, k).copy()
            sem = np.rot90(sem, k).copy()
        if np.random.rand() < 0.5:
            grid = np.fliplr(grid).copy()
            sem = np.fliplr(sem).copy()
        if np.random.rand() < 0.5:
            grid = np.flipud(grid).copy()
            sem = np.flipud(sem).copy()
        return grid, sem


def collect_items(mapdata_dir):
    """Pair each *_npy with its *_sem_syn.npy sibling."""
    mapdata_dir = Path(mapdata_dir)
    grids = sorted(mapdata_dir.glob("*.npy"))
    items = []
    for g in grids:
        if g.name.endswith("_sem.npy") or g.name.endswith("_sem_syn.npy"):
            continue
        s = g.with_name(g.stem + "_sem_syn.npy")
        if s.exists():
            items.append((g, s))
    return items
