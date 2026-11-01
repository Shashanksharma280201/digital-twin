"""Rendered ZED left images (camera/frames.csv + camera/left/%06d.png) -> JPEG + CameraInfo."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path

import cv2
import numpy as np

FRAME_ID = "zed_left_camera_optical_frame"


def load_frames(camera_dir: Path) -> list[tuple[int, float, Path]]:
    """[(index, t, png_path)] for frames whose left image exists; empty if no camera data."""
    csv = Path(camera_dir) / "frames.csv"
    if not csv.exists():
        return []
    data = np.loadtxt(csv, delimiter=",", skiprows=1, ndmin=2)
    frames = []
    for idx, t in data:
        png = Path(camera_dir) / "left" / f"{int(idx):06d}.png"
        if png.exists():
            frames.append((int(idx), float(t), png))
    return frames


def iter_jpeg(frames, quality: int = 90) -> Iterator[tuple[float, bytes, tuple[int, int]]]:
    for _, t, png in frames:
        img = cv2.imread(str(png), cv2.IMREAD_COLOR)
        if img is None:
            continue
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            yield t, buf.tobytes(), (img.shape[1], img.shape[0])
