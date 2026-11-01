"""LAS/LAZ read-write (laspy) and optional COPC conversion (PDAL)."""

from __future__ import annotations

import json
from pathlib import Path

import laspy
import numpy as np


def write_laz(path: str | Path, xyz: np.ndarray, intensity: np.ndarray | None = None,
              rgb: np.ndarray | None = None, extra: dict[str, np.ndarray] | None = None) -> Path:
    """Write points as LAS 1.4 (point format 7 with RGB, else 6). ``rgb`` is uint8 Nx3."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = laspy.LasHeader(point_format=7 if rgb is not None else 6, version="1.4")
    header.scales = np.array([0.001, 0.001, 0.001])
    header.offsets = np.floor(xyz.min(axis=0)) if len(xyz) else np.zeros(3)
    for name, values in (extra or {}).items():
        header.add_extra_dim(laspy.ExtraBytesParams(name=name, type=np.asarray(values).dtype))
    las = laspy.LasData(header)
    las.x, las.y, las.z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    if intensity is not None:
        las.intensity = np.clip(intensity, 0, 65535).astype(np.uint16)
    if rgb is not None:
        rgb16 = rgb.astype(np.uint16) * 257
        las.red, las.green, las.blue = rgb16[:, 0], rgb16[:, 1], rgb16[:, 2]
    for name, values in (extra or {}).items():
        las[name] = values
    las.write(path)
    return path


def read_laz(path: str | Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    las = laspy.read(path)
    xyz = np.column_stack([las.x, las.y, las.z]).astype(np.float64)
    attrs = {"intensity": np.asarray(las.intensity, dtype=np.float32)}
    if hasattr(las, "red"):
        attrs["rgb"] = (np.column_stack([las.red, las.green, las.blue]) // 257).astype(np.uint8)
    for dim in las.point_format.extra_dimension_names:
        attrs[dim] = np.asarray(las[dim])
    return xyz, attrs


def write_copc(src: str | Path, dst: str | Path) -> bool:
    """Convert a LAS/LAZ file to COPC with PDAL. Returns False when PDAL is unavailable."""
    try:
        import pdal
    except ImportError:
        return False
    pipeline = [str(src), {"type": "writers.copc", "filename": str(dst), "forward": "all", "extra_dims": "all"}]
    pdal.Pipeline(json.dumps(pipeline)).execute()
    return True
