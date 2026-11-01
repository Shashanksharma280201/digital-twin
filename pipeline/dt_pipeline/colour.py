"""GPU colouring of map points and mesh vertices from the camera, with exposure compensation.

* Keyframes only: a frame is used when the camera moved >= ``min_move`` m or turned >= ``min_turn``
  degrees since the last used frame (thousands of near-identical frames add time, not colour).
* Blurred frames (sharpness below ``blur_skip`` x running median) are skipped.
* Exposure: the ZED's auto-exposure changes as the rig turns between bright and dark views, and
  averaging those frames washes colours out. Each frame's gain is estimated against the colours
  already accumulated for the points it sees (median brightness ratio) and divided out.
* Best views win: weight = (sharpness x centrality / depth^2) ** sharpen.
"""

from __future__ import annotations

import logging
from collections import deque

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from . import gpu
from .colmap import sharpness
from .geometry import Trajectory

log = logging.getLogger("dtp")

_LUM = (0.299, 0.587, 0.114)  # on RGB


class GpuColourer:
    def __init__(self, xyz: np.ndarray, K: np.ndarray, max_depth: float = 15.0, sharpen: float = 3.0,
                 exposure: bool = True, dev: torch.device | None = None):
        self.dev = dev or gpu.device()
        self.xyz = torch.as_tensor(xyz, dtype=torch.float32, device=self.dev)
        self.K, self.max_depth, self.sharpen, self.exposure = K, max_depth, sharpen, exposure
        self.acc = torch.zeros((len(xyz), 3), dtype=torch.float32, device=self.dev)
        self.weight = torch.zeros(len(xyz), dtype=torch.float32, device=self.dev)
        self.lum = torch.tensor(_LUM, device=self.dev)
        self.gains: list[float] = []

    def add(self, bgr: np.ndarray, T_world_cam: np.ndarray, sharp: float = 1.0) -> int:
        h, w = bgr.shape[:2]
        img = torch.as_tensor(bgr, device=self.dev)
        u, v, z = gpu.project(self.xyz, T_world_cam, self.K)
        idx, ui, vi, d = gpu.visible(u, v, z, w, h, self.max_depth)
        if len(idx) == 0:
            return 0
        rgb = img[vi, ui].flip(-1).float()
        g = 1.0
        if self.exposure:
            seen = self.weight[idx] > 0
            if int(seen.sum()) > 500:
                ref = (self.acc[idx[seen]] / self.weight[idx[seen], None]) @ self.lum
                obs = rgb[seen] @ self.lum
                ok = (obs > 12) & (obs < 243) & (ref > 12)
                if int(ok.sum()) > 300:
                    g = float(torch.median(obs[ok] / ref[ok]).clamp(0.33, 3.0))
            rgb = rgb / g
        self.gains.append(g)
        r2 = ((ui - w / 2) / (w / 2)) ** 2 + ((vi - h / 2) / (h / 2)) ** 2
        centre = (1.0 - 0.4 * r2).clamp(0.2, 1.0)
        wgt = (sharp * centre / d.clamp_min(0.5) ** 2) ** self.sharpen
        self.acc.index_add_(0, idx, rgb * wgt[:, None])
        self.weight.index_add_(0, idx, wgt)
        return len(idx)

    def result(self, unseen=(128, 128, 128)) -> tuple[np.ndarray, np.ndarray]:
        seen = self.weight > 0
        rgb = torch.tensor(unseen, dtype=torch.float32, device=self.dev).repeat(len(self.xyz), 1)
        rgb[seen] = self.acc[seen] / self.weight[seen, None]
        return rgb.clamp(0, 255).to(torch.uint8).cpu().numpy(), seen.cpu().numpy()


def colour_frames(colourer: GpuColourer, frames, tr: Trajectory, T_lidar_optical: np.ndarray,
                  time_offset: float = 0.0, min_move: float = 0.05, min_turn: float = 4.0,
                  blur_skip: float = 0.6, holdout: int = 0) -> dict:
    """Feed ``(stamp, bgr)`` frames through keyframe selection and blur rejection.

    ``bgr`` may be a zero-argument callable, so frames that are skipped are never copied out.
    ``holdout``: frames whose stream index divides by this are never used, to score against them.
    """
    last = None
    recent = deque(maxlen=31)
    stats = {"frames": 0, "used": 0, "blurred": 0, "still": 0}
    for i, (stamp, bgr) in enumerate(frames):
        stats["frames"] += 1
        if holdout and i % holdout == 0:
            continue
        t = stamp + time_offset
        if not tr.covers(np.array([t]))[0]:
            continue
        T_wc = tr.interpolate(t)[0] @ T_lidar_optical
        if last is not None:
            moved = np.linalg.norm(T_wc[:3, 3] - last[:3, 3])
            turned = np.degrees(Rotation.from_matrix(last[:3, :3].T @ T_wc[:3, :3]).magnitude())
            if moved < min_move and turned < min_turn:
                stats["still"] += 1
                continue
        if callable(bgr):
            bgr = bgr()
        sh = sharpness(bgr)
        recent.append(sh)
        med = float(np.median(recent))
        if len(recent) > 5 and sh < blur_skip * med:
            stats["blurred"] += 1
            continue
        colourer.add(bgr, T_wc, float(np.clip(sh / max(med, 1e-6), 0.3, 2.0)))
        last = T_wc
        stats["used"] += 1
    g = np.asarray(colourer.gains) if colourer.gains else np.ones(1)
    stats["exposure_gain_p5_p95"] = [round(float(np.percentile(g, 5)), 2), round(float(np.percentile(g, 95)), 2)]
    return stats
