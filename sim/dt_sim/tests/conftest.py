"""Synthetic run directory: box room with pillars, static start, walk, turn."""

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from dt_sim.run import RunDir
from dt_sim.trajectory import save_csv
from dt_sim.world import World

ROOM = (np.array([-5.0, -4.0, 0.0]), np.array([6.0, 4.0, 3.0]))
PILLARS = [((1.0, 1.5, 0.0), (1.5, 2.0, 3.0)), ((3.0, -2.5, 0.0), (3.4, -2.1, 3.0)),
           ((-2.0, -1.0, 0.0), (-1.2, -0.4, 1.0))]


def box_mesh(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    v = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return v, np.array(faces)


def make_world() -> World:
    verts, tris, cls = [], [], []
    for k, (lo, hi) in enumerate([ROOM, *PILLARS]):
        v, f = box_mesh(lo, hi)
        tris.append(f + sum(len(x) for x in verts))
        verts.append(v)
        cls.append(np.full(len(f), 1 if k == 0 else 2, np.uint8))
    tris = np.concatenate(tris)
    return World(np.concatenate(verts), tris, np.concatenate(cls), np.full(len(tris), 0.6, np.float32),
                 {"1": "room", "2": "pillar"})


def smooth(u):
    u = np.clip(u, 0, 1)
    return u ** 3 * (10 - 15 * u + 6 * u ** 2)  # quintic ease, zero vel/acc at ends


def make_trajectory(duration=7.0, rate=1000.0, spin=0.0):
    t = np.arange(0, duration, 1.0 / rate)
    x = -2.0 + 3.0 * smooth((t - 2.0) / 3.0)
    y = np.zeros_like(t)
    z = np.full_like(t, 1.2) + 0.02 * np.sin(2 * np.pi * 1.8 * t) * (t > 2.0)
    yaw = np.radians(90) * smooth((t - 5.0) / 2.0) + spin * t
    q = Rotation.from_euler("z", yaw).as_quat()
    return t, np.column_stack([x, y, z]), q


def write_rig(path: Path, profile: str, pitch: float = 0.0):
    imu = [0.011, 0.02329, -0.04412] if profile == "mid360" else [0.0, 0.0, 0.0]
    R_opt = Rotation.from_matrix(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]))  # link -> optical
    path.write_text(yaml.safe_dump({
        "profile": profile,
        "T_base_lidar": {"xyz": [0.05, 0.0, 0.25], "rpy": [0.0, pitch, 0.0]},
        "T_lidar_imu": {"xyz": imu, "rpy": [0.0, 0.0, 0.0]},
        "T_lidar_camera_optical": {"xyz": [0.08, 0.06, -0.06], "quat_xyzw": R_opt.as_quat().tolist()},
        "camera_time_offset": 0.0,
        "camera": {"width": 64, "height": 48, "fx": 40.0, "fy": 40.0, "cx": 32.0, "cy": 24.0,
                   "baseline": 0.12},
    }))


def make_run(root: Path, profile: str = "mid360", spin: float = 0.0, pitch: float = 0.0,
             duration: float = 7.0) -> RunDir:
    run = RunDir(root)
    (root / "world").mkdir(parents=True)
    make_world().save(run.mesh("day1"))
    save_csv(run.trajectory_path, *make_trajectory(duration, spin=spin))
    write_rig(run.rig_path, profile, pitch)
    cam = root / "camera"
    (cam / "left").mkdir(parents=True)
    rows = []
    for i, t in enumerate((2.5, 4.5)):
        img = np.full((48, 64, 3), 60 + 80 * i, np.uint8)
        cv2.imwrite(str(cam / "left" / f"{i:06d}.png"), img)
        rows.append((i, t))
    np.savetxt(cam / "frames.csv", np.array(rows), delimiter=",", header="index,t", comments="",
               fmt=["%d", "%.6f"])
    return run


@pytest.fixture(scope="session")
def mid360_run(tmp_path_factory) -> RunDir:
    return make_run(tmp_path_factory.mktemp("mid360"), "mid360")


@pytest.fixture(scope="session")
def airy_run(tmp_path_factory) -> RunDir:
    return make_run(tmp_path_factory.mktemp("airy"), "airy")


def write_json(path: Path, data):
    path.write_text(json.dumps(data))
