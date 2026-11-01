"""Write a driver-native MCAP bag for a simulated run (design §4).

Streams are generated lazily and merged by log time, so long runs do not need to fit in
memory. LiDAR frames are logged at the end of their sweep, like the real drivers.
"""

from __future__ import annotations

import json
import heapq
import logging
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from dt_pipeline.io.bag import typestore
from rosbags.rosbag2 import StoragePlugin, Writer
from scipy.spatial.transform import Rotation

from . import camera as cam
from .imu import ImuNoise, simulate_imu
from .lidar import LidarNoise
from .lidar_airy import FIELDS as AIRY_FIELDS
from .lidar_airy import Airy
from .lidar_airy import FRAME_ID as AIRY_FRAME
from .lidar_airy import pack_points as pack_airy
from .lidar_mid360 import FRAME_ID as LIVOX_FRAME
from .lidar_mid360 import Mid360, custom_msg_cdr
from .lidar_mid360 import pack_points as pack_mid360
from .run import AIRY_IMU_R, EPOCH0, PROFILE_IMU_XYZ, RunDir
from .trajectory import Trajectory
from .world import World

log = logging.getLogger(__name__)

TOPICS = {
    "mid360": {"points": ("/livox/lidar", "livox_ros_driver2/msg/CustomMsg"),
               "imu": ("/livox/imu", "sensor_msgs/msg/Imu")},
    "airy": {"points": ("/rslidar_points", "sensor_msgs/msg/PointCloud2"),
             "imu": ("/rslidar_imu_data", "sensor_msgs/msg/Imu")},
}
IMAGE_TOPIC = ("/zed/zed_node/rgb/color/rect/image/compressed", "sensor_msgs/msg/CompressedImage")
INFO_TOPIC = ("/zed/zed_node/rgb/color/rect/camera_info", "sensor_msgs/msg/CameraInfo")
GT_TOPIC = ("/sim/ground_truth/odometry", "nav_msgs/msg/Odometry")


@dataclass
class BagOptions:
    profile: str = "mid360"
    variant: str = "day1"
    camera: bool = True
    duration: float | None = None
    seed: int = 0
    epoch0: float = EPOCH0
    imu_rate: float = 200.0
    gt_rate: float = 100.0
    lidar_noise: LidarNoise | None = None
    ring_bias_mm: float = 0.0
    imu_noise: ImuNoise | None = None
    jpeg_quality: int = 90


class _Msgs:
    def __init__(self, ts):
        self.ts = ts
        self.t = ts.types

    def header(self, stamp_ns: int, frame: str):
        return self.t["std_msgs/msg/Header"](
            stamp=self.t["builtin_interfaces/msg/Time"](sec=stamp_ns // 1_000_000_000,
                                                        nanosec=stamp_ns % 1_000_000_000),
            frame_id=frame)

    def vec3(self, v):
        return self.t["geometry_msgs/msg/Vector3"](x=float(v[0]), y=float(v[1]), z=float(v[2]))

    def imu(self, stamp_ns, frame, acc, gyro):
        cov = np.zeros(9)
        return self.t["sensor_msgs/msg/Imu"](
            header=self.header(stamp_ns, frame),
            orientation=self.t["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0),
            orientation_covariance=cov, angular_velocity=self.vec3(gyro),
            angular_velocity_covariance=cov, linear_acceleration=self.vec3(acc),
            linear_acceleration_covariance=cov)

    def pointcloud2(self, stamp_ns, frame, arr: np.ndarray, fields):
        PF = self.t["sensor_msgs/msg/PointField"]
        return self.t["sensor_msgs/msg/PointCloud2"](
            header=self.header(stamp_ns, frame), height=1, width=len(arr),
            fields=[PF(name=n, offset=o, datatype=d, count=1) for n, o, d in fields],
            is_bigendian=False, point_step=arr.dtype.itemsize, row_step=arr.dtype.itemsize * len(arr),
            data=np.frombuffer(arr.tobytes(), np.uint8), is_dense=False)

    def odometry(self, stamp_ns, frame, child, T, lin_vel=None):
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        t = self.t
        pose = t["geometry_msgs/msg/Pose"](
            position=t["geometry_msgs/msg/Point"](x=float(T[0, 3]), y=float(T[1, 3]), z=float(T[2, 3])),
            orientation=t["geometry_msgs/msg/Quaternion"](x=q[0], y=q[1], z=q[2], w=q[3]))
        zero = self.vec3((0, 0, 0))
        return t["nav_msgs/msg/Odometry"](
            header=self.header(stamp_ns, frame), child_frame_id=child,
            pose=t["geometry_msgs/msg/PoseWithCovariance"](pose=pose, covariance=np.zeros(36)),
            twist=t["geometry_msgs/msg/TwistWithCovariance"](
                twist=t["geometry_msgs/msg/Twist"](
                    linear=self.vec3(lin_vel) if lin_vel is not None else zero, angular=zero),
                covariance=np.zeros(36)))

    def camera_info(self, stamp_ns, frame, model, size):
        K = model.K
        t = self.t
        return t["sensor_msgs/msg/CameraInfo"](
            header=self.header(stamp_ns, frame), height=int(size[1]), width=int(size[0]),
            distortion_model="plumb_bob", d=np.zeros(5), k=K.ravel(), r=np.eye(3).ravel(),
            p=np.column_stack([K, np.zeros(3)]).ravel(), binning_x=0, binning_y=0,
            roi=t["sensor_msgs/msg/RegionOfInterest"](x_offset=0, y_offset=0, height=0, width=0,
                                                       do_rectify=False))

    def compressed(self, stamp_ns, frame, data: bytes):
        return self.t["sensor_msgs/msg/CompressedImage"](
            header=self.header(stamp_ns, frame), format="bgr8; jpeg compressed bgr8",
            data=np.frombuffer(data, np.uint8))


def _ns(epoch0: float, t: float) -> int:
    return int(round(epoch0 * 1e9)) + int(round(t * 1e9))


def write_bag(run: RunDir, out: Path, opts: BagOptions) -> dict:
    sim = run.load_rig()
    profile = opts.profile or sim.profile
    if profile not in TOPICS:
        raise ValueError(f"unknown profile {profile!r}")
    expected = PROFILE_IMU_XYZ[profile]
    if not np.allclose(sim.rig.T_lidar_imu[:3, 3], expected, atol=1e-3):
        log.warning("rig_sim T_lidar_imu %s differs from the %s stack profile %s; FAST-LIO will "
                    "use the profile value", sim.rig.T_lidar_imu[:3, 3], profile, expected)

    world = World.load(run.mesh(opts.variant))
    t_csv = run.trajectory_path
    traj_lidar = Trajectory.from_csv(t_csv, sim.rig.T_base_lidar)
    traj_imu = Trajectory.from_csv(t_csv, sim.T_base_imu)
    traj_base = Trajectory.from_csv(t_csv)
    t_end = traj_base.end if opts.duration is None else min(traj_base.end, traj_base.start + opts.duration)
    t0 = traj_base.start

    ts = typestore()
    m = _Msgs(ts)
    topics = TOPICS[profile]

    def lidar_stream() -> Iterator:
        period = 0.1
        if profile == "mid360":
            sensor = Mid360(world, traj_lidar, opts.lidar_noise, seed=opts.seed)
            k = 0
            while t0 + (k + 1) * period <= t_end:
                ts_start = t0 + k * period
                idx, ret, offs = sensor.frame(k, ts_start)
                stamp = _ns(opts.epoch0, ts_start)
                raw = custom_msg_cdr(stamp, stamp, pack_mid360(idx, ret, offs))
                yield _ns(opts.epoch0, ts_start + period), "points", raw
                k += 1
        else:
            sensor = Airy(world, traj_lidar, opts.lidar_noise, seed=opts.seed, ring_bias_mm=opts.ring_bias_mm)
            if opts.ring_bias_mm:
                (run.root / "ring_bias_truth.json").write_text(json.dumps({"range_m": sensor.ring_bias.tolist()}))
            k = 0
            while t0 + (k + 1) * period <= t_end:
                ts_start = t0 + k * period
                ret, times = sensor.frame(ts_start)
                arr = pack_airy(ret, sensor.ring, opts.epoch0 + times)
                msg = m.pointcloud2(_ns(opts.epoch0, ts_start), AIRY_FRAME, arr, AIRY_FIELDS)
                yield (_ns(opts.epoch0, ts_start + period), "points",
                       ts.serialize_cdr(msg, topics["points"][1]))
                k += 1

    def imu_stream() -> Iterator:
        R_raw = AIRY_IMU_R if profile == "airy" else None
        frame = LIVOX_FRAME if profile == "mid360" else AIRY_FRAME
        s = simulate_imu(traj_imu, opts.imu_rate, R_raw, opts.imu_noise, opts.seed + 1, t0, t_end)
        for t, a, w in zip(s.t, s.acc_g, s.gyro):
            stamp = _ns(opts.epoch0, t)
            yield stamp, "imu", ts.serialize_cdr(m.imu(stamp, frame, a, w), "sensor_msgs/msg/Imu")

    def gt_stream() -> Iterator:
        times = np.arange(t0, t_end, 1.0 / opts.gt_rate)
        T = traj_base.matrices(times)
        vel = traj_base.velocity(times)
        for t, Ti, v in zip(times, T, vel):
            stamp = _ns(opts.epoch0, t)
            v = Ti[:3, :3].T @ v  # nav_msgs twist is expressed in the child frame
            yield stamp, "gt", ts.serialize_cdr(m.odometry(stamp, "world", "base_link", Ti, v),
                                                GT_TOPIC[1])

    def camera_stream() -> Iterator:
        frames = [f for f in cam.load_frames(run.camera_dir) if t0 <= f[1] <= t_end]
        for t, data, size in cam.iter_jpeg(frames, opts.jpeg_quality):
            stamp = _ns(opts.epoch0, t)
            yield stamp, "image", ts.serialize_cdr(m.compressed(stamp, cam.FRAME_ID, data), IMAGE_TOPIC[1])
            yield stamp, "info", ts.serialize_cdr(
                m.camera_info(stamp, cam.FRAME_ID, sim.camera, size), INFO_TOPIC[1])

    streams = [lidar_stream(), imu_stream(), gt_stream()]
    has_camera = opts.camera and bool(cam.load_frames(run.camera_dir))
    if opts.camera and not has_camera:
        log.warning("no rendered camera frames in %s; writing LiDAR/IMU only", run.camera_dir)
    if has_camera:
        streams.append(camera_stream())

    out = Path(out)
    counts: dict[str, int] = {}
    with Writer(out, version=8, storage_plugin=StoragePlugin.MCAP) as w:
        conns = {
            "points": w.add_connection(*topics["points"], typestore=ts),
            "imu": w.add_connection(*topics["imu"], typestore=ts),
            "gt": w.add_connection(*GT_TOPIC, typestore=ts),
        }
        if has_camera:
            conns["image"] = w.add_connection(*IMAGE_TOPIC, typestore=ts)
            conns["info"] = w.add_connection(*INFO_TOPIC, typestore=ts)
        for stamp, key, raw in heapq.merge(*streams, key=lambda x: x[0]):
            w.write(conns[key], stamp, raw)
            counts[key] = counts.get(key, 0) + 1
            if key == "points" and counts[key] % 50 == 0:
                log.info("%d LiDAR frames (t=%.1f s)", counts[key], (stamp * 1e-9 - opts.epoch0))
    return {"profile": profile, "variant": opts.variant, "duration": t_end - t0,
            "epoch0": opts.epoch0, "messages": counts, "bag": str(out)}
