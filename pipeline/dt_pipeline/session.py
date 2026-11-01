"""Capture session folder layout (design doc §8)::

    <session>/bag/          rosbag2 (MCAP)
    <session>/rig.yaml      extrinsics at capture time
    <session>/meta.yaml
    <session>/products/     everything dt_pipeline writes
"""

from __future__ import annotations

from pathlib import Path

from .rig import Rig


class Session:
    def __init__(self, path: str | Path):
        self.path = Path(path).expanduser().resolve()
        if not self.path.exists():
            raise FileNotFoundError(self.path)

    @property
    def bag(self) -> Path:
        if (self.path / "bag").exists():
            return self.path / "bag"
        if (self.path / "metadata.yaml").exists() or self.path.suffix == ".mcap":
            return self.path
        mcaps = sorted(self.path.glob("*.mcap"))
        if mcaps:
            return mcaps[0]
        raise FileNotFoundError(f"no bag found in {self.path}")

    @property
    def rig(self) -> Rig:
        p = self.path / "rig.yaml"
        return Rig.load(p) if p.exists() else Rig()

    @property
    def products(self) -> Path:
        p = self.path / "products"
        p.mkdir(exist_ok=True)
        return p

    @property
    def traj_path(self) -> Path:
        return self.products / "traj_lidar.tum"

    @property
    def map_path(self) -> Path:
        return self.products / "map.laz"

    @property
    def static_map_path(self) -> Path:
        return self.products / "map_static.laz"

    def best_map(self) -> Path:
        return self.static_map_path if self.static_map_path.exists() else self.map_path
