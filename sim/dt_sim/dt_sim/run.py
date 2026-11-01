"""Run directory layout and rig_sim.yaml (design: docs/design/2026-10-18-blender-sim-design.md §3).

``rig_sim.yaml`` uses the rig.yaml schema (``dt_pipeline.rig``) plus::

    profile: mid360 | airy
    camera: {width, height, fx, fy, cx, cy, baseline}
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from dt_pipeline.rig import Rig

EPOCH0 = 1760000000.0

# Bridge rotation for the Airy IMU (ros2_ws/src/dt_bringup/config/profiles/airy/bridge.yaml):
# v_lidar = R @ v_raw. The simulator emits raw axes, a_raw = R.T @ a_lidar.
AIRY_IMU_R = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])

# IMU position in lidar_link expected by the stack profiles (rig.yaml / fast_lio.yaml).
PROFILE_IMU_XYZ = {
    "mid360": np.array([0.011, 0.02329, -0.04412]),
    "airy": np.zeros(3),
}


@dataclass
class CameraModel:
    width: int = 1280
    height: int = 720
    fx: float = 700.0
    fy: float = 700.0
    cx: float = 640.0
    cy: float = 360.0
    baseline: float = 0.12

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])


@dataclass
class SimRig:
    rig: Rig
    camera: CameraModel = field(default_factory=CameraModel)
    profile: str = "mid360"

    @classmethod
    def load(cls, path: str | Path) -> "SimRig":
        data = yaml.safe_load(Path(path).read_text()) or {}
        cam = data.get("camera") or {}
        camera = CameraModel(**{k: type(getattr(CameraModel, k))(v) for k, v in cam.items()
                                if hasattr(CameraModel, k)})
        return cls(rig=Rig.load(path), camera=camera, profile=str(data.get("profile", "mid360")))

    @property
    def T_base_imu(self) -> np.ndarray:
        return self.rig.T_base_lidar @ self.rig.T_lidar_imu

    @property
    def T_base_optical(self) -> np.ndarray:
        return self.rig.T_base_lidar @ self.rig.lidar_to_optical


class RunDir:
    def __init__(self, root: str | Path):
        self.root = Path(root).expanduser()

    def mesh(self, variant: str = "day1") -> Path:
        return self.root / "world" / f"mesh_{variant}.npz"

    @property
    def rig_path(self) -> Path:
        return self.root / "rig_sim.yaml"

    @property
    def trajectory_path(self) -> Path:
        return self.root / "trajectory.csv"

    @property
    def camera_dir(self) -> Path:
        return self.root / "camera"

    @property
    def products(self) -> Path:
        return self.root / "products"

    def load_rig(self) -> SimRig:
        return SimRig.load(self.rig_path)
