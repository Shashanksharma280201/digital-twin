"""Synthetic capture sessions: a ray-cast box room observed by a moving spinning LiDAR."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.rosbag2 import StoragePlugin, Writer
from scipy.spatial.transform import Rotation

from dt_pipeline.geometry import inv, make_T
from dt_pipeline.io import typestore
from dt_pipeline.rig import R_LINK_OPTICAL

T0 = 1_760_000_000.0
ROOM = (np.array([-5.0, -4.0, 0.0]), np.array([5.0, 4.0, 3.0]))
PILLAR = (np.array([3.0, 2.0, 0.0]), np.array([3.6, 2.6, 3.0]))
ADDED_BOX = (np.array([-1.0, 1.5, 0.0]), np.array([0.0, 2.5, 1.0]))
T_LIDAR_IMU = make_T(t=[0.011, 0.02329, -0.04412])
T_LIDAR_OPTICAL = make_T(R_LINK_OPTICAL, [0.05, 0.0, -0.03])
K = np.array([[100.0, 0, 80.0], [0, 100.0, 60.0], [0, 0, 1]])
W, H = 160, 120


def gt_pose(t: np.ndarray) -> np.ndarray:
    """Ground-truth T_world_lidar at absolute times ``t``."""
    s = np.atleast_1d(t) - T0
    T = np.tile(np.eye(4), (len(s), 1, 1))
    T[:, :3, :3] = Rotation.from_euler("zyx", np.column_stack([0.3 * s, 0.05 * np.sin(s), 0 * s])).as_matrix()
    T[:, :3, 3] = np.column_stack([-2.0 + 0.8 * s, 0.5 * np.sin(0.5 * s), np.full_like(s, 1.0)])
    return T


def _ray_room(o, d):
    lo, hi = ROOM
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(d > 0, (hi - o) / d, np.where(d < 0, (lo - o) / d, np.inf))
    return t.min(axis=1)


def _ray_box(o, d, lo, hi):
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (lo - o) / d, (hi - o) / d
    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
    return np.where((tmax >= tmin) & (tmin > 0), tmin, np.inf)


def person_box(t: np.ndarray):
    y = -3.0 + 1.5 * (np.atleast_1d(t) - T0)
    lo = np.column_stack([np.full_like(y, 1.8), y - 0.2, np.zeros_like(y)])
    return lo, lo + [0.4, 0.4, 1.7]


@dataclass
class Scene:
    added_box: bool = False
    moving_person: bool = False
    T_frame_world: np.ndarray = field(default_factory=lambda: np.eye(4))  # session odometry frame


def lidar_dirs(n_az: int = 360, elev_deg=np.linspace(-30, 50, 32)):
    az = np.linspace(0, 2 * np.pi, n_az, endpoint=False)
    A, E = np.meshgrid(az, np.radians(elev_deg))
    d = np.column_stack([(np.cos(E) * np.cos(A)).ravel(), (np.cos(E) * np.sin(A)).ravel(), np.sin(E).ravel()])
    frac = (A / (2 * np.pi)).ravel()
    ring = np.repeat(np.arange(len(elev_deg)), n_az)
    return d, frac, ring


def simulate_scan(stamp: float, scene: Scene, period: float = 0.1):
    d, frac, ring = lidar_dirs()
    t = stamp + frac * period
    P = gt_pose(t)
    o = P[:, :3, 3]
    dw = np.einsum("nij,nj->ni", P[:, :3, :3], d)
    hit = np.minimum(_ray_room(o, dw), _ray_box(o, dw, *PILLAR))
    if scene.added_box:
        hit = np.minimum(hit, _ray_box(o, dw, *ADDED_BOX))
    if scene.moving_person:
        lo, hi = person_box(t)
        hit = np.minimum(hit, _ray_box(o, dw, lo, hi))
    return d * hit[:, None], frac * period, ring


def _header(ts, stamp: float, frame: str):
    Time = ts.types["builtin_interfaces/msg/Time"]
    Header = ts.types["std_msgs/msg/Header"]
    sec = int(np.floor(stamp))
    return Header(stamp=Time(sec=sec, nanosec=int(round((stamp - sec) * 1e9)) % 1_000_000_000), frame_id=frame)


def canonical_cloud(ts, stamp, xyz, rel, ring):
    PF = ts.types["sensor_msgs/msg/PointField"]
    dtype = np.dtype({"names": ["x", "y", "z", "intensity", "time", "ring"],
                      "formats": ["<f4", "<f4", "<f4", "<f4", "<f4", "<u2"],
                      "offsets": [0, 4, 8, 12, 16, 20], "itemsize": 24})
    arr = np.zeros(len(xyz), dtype)
    arr["x"], arr["y"], arr["z"] = xyz.T
    arr["intensity"] = 100.0
    arr["time"] = rel
    arr["ring"] = ring
    fields = [PF(name=n, offset=o, datatype=dt, count=1)
              for n, o, dt in [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("time", 16, 7), ("ring", 20, 4)]]
    return ts.types["sensor_msgs/msg/PointCloud2"](
        header=_header(ts, stamp, "lidar_link"), height=1, width=len(arr), fields=fields, is_bigendian=False,
        point_step=24, row_step=24 * len(arr), data=np.frombuffer(arr.tobytes(), np.uint8), is_dense=True)


def odometry(ts, stamp, T):
    t = ts.types
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    pose = t["geometry_msgs/msg/Pose"](
        position=t["geometry_msgs/msg/Point"](x=T[0, 3], y=T[1, 3], z=T[2, 3]),
        orientation=t["geometry_msgs/msg/Quaternion"](x=q[0], y=q[1], z=q[2], w=q[3]))
    v0 = t["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0)
    return t["nav_msgs/msg/Odometry"](
        header=_header(ts, stamp, "camera_init"), child_frame_id="body",
        pose=t["geometry_msgs/msg/PoseWithCovariance"](pose=pose, covariance=np.zeros(36)),
        twist=t["geometry_msgs/msg/TwistWithCovariance"](
            twist=t["geometry_msgs/msg/Twist"](linear=v0, angular=v0), covariance=np.zeros(36)))


def camera_info(ts, stamp):
    t = ts.types
    return t["sensor_msgs/msg/CameraInfo"](
        header=_header(ts, stamp, "zed_left_camera_optical_frame"), height=H, width=W,
        distortion_model="plumb_bob", d=np.zeros(5), k=K.ravel(), r=np.eye(3).ravel(),
        p=np.column_stack([K, np.zeros(3)]).ravel(), binning_x=0, binning_y=0,
        roi=t["sensor_msgs/msg/RegionOfInterest"](x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))


def compressed_image(ts, stamp, value: int):
    img = np.full((H, W, 3), value, np.uint8)
    img[:, : W // 2, 2] = 255 - value
    ok, buf = cv2.imencode(".jpg", img)
    return ts.types["sensor_msgs/msg/CompressedImage"](
        header=_header(ts, stamp, "zed_left_camera_optical_frame"), format="jpeg", data=buf.ravel())


def write_rig(path: Path):
    path.write_text(yaml.safe_dump({
        "profile": "synthetic",
        "T_lidar_imu": {"xyz": T_LIDAR_IMU[:3, 3].tolist(), "rpy": [0.0, 0.0, 0.0]},
        "T_lidar_camera_optical": {"xyz": T_LIDAR_OPTICAL[:3, 3].tolist(),
                                   "quat_xyzw": Rotation.from_matrix(T_LIDAR_OPTICAL[:3, :3]).as_quat().tolist()},
        "camera_time_offset": 0.0,
    }))


def make_session(root: Path, scene: Scene, duration: float = 4.0) -> Path:
    ts = typestore()
    root.mkdir(parents=True)
    write_rig(root / "rig.yaml")
    with Writer(root / "bag", version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c_pts = w.add_connection("/dt/lidar/points", "sensor_msgs/msg/PointCloud2", typestore=ts)
        c_odom = w.add_connection("/Odometry", "nav_msgs/msg/Odometry", typestore=ts)
        c_img = w.add_connection("/zed/zed_node/rgb/color/rect/image/compressed",
                                 "sensor_msgs/msg/CompressedImage", typestore=ts)
        c_info = w.add_connection("/zed/zed_node/rgb/color/rect/camera_info",
                                  "sensor_msgs/msg/CameraInfo", typestore=ts)
        msgs = []
        for stamp in np.arange(T0 - 0.04, T0 + duration + 0.2, 0.02):
            T_frame_body = scene.T_frame_world @ gt_pose(stamp)[0] @ T_LIDAR_IMU
            msgs.append((stamp, c_odom, odometry(ts, stamp, T_frame_body), "nav_msgs/msg/Odometry"))
        for stamp in np.arange(T0, T0 + duration, 0.1):
            xyz, rel, ring = simulate_scan(stamp, scene)
            msgs.append((stamp + 0.1, c_pts, canonical_cloud(ts, stamp, xyz, rel, ring), "sensor_msgs/msg/PointCloud2"))
        for k, stamp in enumerate(np.arange(T0 + 0.25, T0 + duration - 0.25, 0.5)):
            msgs.append((stamp, c_img, compressed_image(ts, stamp, 40 + 20 * k), "sensor_msgs/msg/CompressedImage"))
            msgs.append((stamp, c_info, camera_info(ts, stamp), "sensor_msgs/msg/CameraInfo"))
        for stamp, conn, msg, mt in sorted(msgs, key=lambda m: m[0]):
            w.write(conn, int(stamp * 1e9), ts.serialize_cdr(msg, mt))
    return root


def surface_distance(p: np.ndarray, with_box: bool = False) -> np.ndarray:
    """Distance of points to the nearest static scene surface."""
    lo, hi = ROOM
    d = np.minimum(np.abs(p - lo), np.abs(hi - p)).min(axis=1)
    boxes = [PILLAR] + ([ADDED_BOX] if with_box else [])
    for blo, bhi in boxes:
        outside = np.linalg.norm(np.maximum(np.maximum(blo - p, p - bhi), 0), axis=1)
        inside = np.minimum(np.abs(p - blo), np.abs(bhi - p)).min(axis=1)
        is_in = np.all((p > blo) & (p < bhi), axis=1)
        d = np.minimum(d, np.where(is_in, inside, outside))
    return d


def world_offset() -> np.ndarray:
    """T_frame_world for a second session whose odometry frame differs from the first."""
    return inv(make_T(Rotation.from_euler("z", 4.0, degrees=True).as_matrix(), [0.25, -0.15, 0.05]))
