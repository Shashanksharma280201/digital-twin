"""ROS-free bag access (rosbags) with fast numpy decoding of LiDAR and image messages.

Any LiDAR topic is converted to a :class:`Scan` that follows the canonical contract
(design doc §4.1): absolute ``stamp`` of the earliest point plus per-point ``time``
relative to it. Supported inputs are the canonical ``/dt/lidar/points`` cloud, generic
PointCloud2 clouds with relative or absolute per-point times (RoboSense XYZIRT,
Livox PointXYZRTLT, Ouster) and ``livox_ros_driver2/msg/CustomMsg``.
"""

from __future__ import annotations

import struct
from collections.abc import Iterable, Iterator
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

POINTS_TOPIC = "/dt/lidar/points"
IMU_TOPIC = "/dt/lidar/imu"
ODOM_TOPIC = "/Odometry"
# ZED wrapper "rgb" = rectified left camera; v5.2 publishes only rgb/..., v5.5 publishes both.
IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image/compressed"
CAMERA_INFO_TOPIC = "/zed/zed_node/rgb/color/rect/camera_info"
# Older recordings (and early sim bags) used the left/... names for the same stream.
_TOPIC_ALIASES = {
    IMAGE_TOPIC: "/zed/zed_node/left/color/rect/image/compressed",
    CAMERA_INFO_TOPIC: "/zed/zed_node/left/color/rect/camera_info",
}

LIVOX_CUSTOM_MSG = "livox_ros_driver2/msg/CustomMsg"
LIVOX_CUSTOM_POINT = "livox_ros_driver2/msg/CustomPoint"
_LIVOX_POINT_DEF = """
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""
_LIVOX_MSG_DEF = """
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
"""

_PF_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
_TIME_FIELDS = ("time", "t", "timestamp", "time_stamp", "offset_time")
_INTENSITY_FIELDS = ("intensity", "reflectivity")

_LIVOX_DTYPE = np.dtype({
    "names": ["t", "x", "y", "z", "refl", "tag", "line"],
    "formats": ["<u4", "<f4", "<f4", "<f4", "u1", "u1", "u1"],
    "offsets": [0, 4, 8, 12, 16, 17, 18],
    "itemsize": 20,
})


def typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    ts.register({
        **get_types_from_msg(_LIVOX_POINT_DEF, LIVOX_CUSTOM_POINT),
        **get_types_from_msg(_LIVOX_MSG_DEF, LIVOX_CUSTOM_MSG),
    })
    return ts


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


@dataclass
class Scan:
    stamp: float  # absolute time of the earliest point [s]
    xyz: np.ndarray  # Nx3 float64, sensor frame
    intensity: np.ndarray  # N float32
    time: np.ndarray  # N float64, seconds relative to ``stamp``
    ring: np.ndarray | None = None
    frame_id: str = ""

    def __len__(self) -> int:
        return len(self.xyz)

    @property
    def abs_time(self) -> np.ndarray:
        return self.stamp + self.time


@dataclass
class CameraInfo:
    width: int
    height: int
    K: np.ndarray  # 3x3
    D: np.ndarray
    distortion_model: str
    frame_id: str


class Bag:
    """Thin wrapper over :class:`AnyReader` for a rosbag2 directory or ``.mcap`` file."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self._reader = AnyReader([self.path], default_typestore=typestore())

    def __enter__(self) -> "Bag":
        self._reader.open()
        return self

    def __exit__(self, *exc) -> None:
        self._reader.close()

    @property
    def topics(self) -> dict[str, str]:
        return {c.topic: c.msgtype for c in self._reader.connections}

    def msgtype(self, topic: str) -> str:
        try:
            return self.topics[topic]
        except KeyError:
            raise KeyError(f"topic {topic!r} not in bag {self.path} (have: {sorted(self.topics)})") from None

    def raw(self, topics: Iterable[str]) -> Iterator[tuple[str, str, int, bytes]]:
        """Yield ``(topic, msgtype, receive_time_ns, rawdata)`` in log order."""
        conns = [c for c in self._reader.connections if c.topic in set(topics)]
        for conn, t_ns, data in self._reader.messages(connections=conns):
            yield conn.topic, conn.msgtype, t_ns, data

    def messages(self, topics: Iterable[str]) -> Iterator[tuple[str, int, object]]:
        for topic, msgtype, t_ns, data in self.raw(topics):
            yield topic, t_ns, self._reader.deserialize(data, msgtype)

    def info(self) -> dict:
        r = self._reader
        counts: dict[str, int] = {}
        for c in r.connections:
            counts[c.topic] = counts.get(c.topic, 0) + c.msgcount
        duration = (r.end_time - r.start_time) * 1e-9 if r.message_count else 0.0
        return {
            "path": str(self.path),
            "start": r.start_time * 1e-9,
            "end": r.end_time * 1e-9,
            "duration": duration,
            "topics": {
                t: {"type": self.topics[t], "count": n, "rate_hz": n / duration if duration > 0 else 0.0}
                for t, n in sorted(counts.items())
            },
        }

    def scans(self, topic: str = POINTS_TOPIC, livox_tag_filter: bool = True) -> Iterator[Scan]:
        msgtype = self.msgtype(topic)
        for _, _, _, data in self.raw([topic]):
            if msgtype == LIVOX_CUSTOM_MSG:
                scan = livox_custom_to_scan(data, tag_filter=livox_tag_filter)
            elif msgtype == "sensor_msgs/msg/PointCloud2":
                scan = pointcloud2_to_scan(self._reader.deserialize(data, msgtype))
            else:
                raise TypeError(f"{topic}: unsupported LiDAR message type {msgtype}")
            if len(scan):
                yield scan

    def resolve(self, topic: str) -> str:
        """The topic itself, or its legacy alias when only that is in the bag."""
        alias = _TOPIC_ALIASES.get(topic)
        if alias and topic not in self.topics and alias in self.topics:
            return alias
        return topic

    def images(self, topic: str = IMAGE_TOPIC, lazy: bool = False) -> Iterator[tuple[float, np.ndarray]]:
        """Yield ``(header_stamp_s, bgr_image)`` for CompressedImage topics.

        ``lazy``: yield a callable that decodes the JPEG, so skipped frames cost nothing.
        """
        for _, _, msg in self.messages([self.resolve(topic)]):
            stamp = stamp_to_sec(msg.header.stamp)
            yield stamp, ((lambda m=msg: decode_compressed_image(m)) if lazy else decode_compressed_image(msg))

    def image_stamps(self, topic: str = IMAGE_TOPIC) -> np.ndarray:
        return np.array([stamp_to_sec(m.header.stamp) for _, _, m in self.messages([self.resolve(topic)])])

    def camera_info(self, topic: str = CAMERA_INFO_TOPIC) -> CameraInfo:
        for _, _, msg in self.messages([self.resolve(topic)]):
            return CameraInfo(
                width=int(msg.width),
                height=int(msg.height),
                K=np.asarray(msg.k, dtype=float).reshape(3, 3),
                D=np.asarray(msg.d, dtype=float),
                distortion_model=msg.distortion_model,
                frame_id=msg.header.frame_id,
            )
        raise LookupError(f"no CameraInfo on {topic}")


def pointcloud2_array(msg) -> np.ndarray:
    """Structured numpy view of a ``sensor_msgs/PointCloud2`` (no per-point copies)."""
    endian = ">" if msg.is_bigendian else "<"
    names, formats, offsets = [], [], []
    for f in msg.fields:
        base = endian + _PF_DTYPES[f.datatype]
        names.append(f.name)
        formats.append(base if f.count <= 1 else (base, f.count))
        offsets.append(f.offset)
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    n = msg.width * msg.height
    data = np.asarray(msg.data, dtype=np.uint8)
    if msg.row_step != msg.width * msg.point_step and msg.height > 1:
        data = data.reshape(msg.height, msg.row_step)[:, : msg.width * msg.point_step].reshape(-1)
    return np.frombuffer(data.tobytes() if not data.flags.c_contiguous else data, dtype=dtype, count=n)


def _relative_times(values: np.ndarray, header_stamp: float) -> np.ndarray:
    """Convert a per-point time field to seconds relative to ``header_stamp``."""
    v = values.astype(np.float64)
    vmax = np.nanmax(np.abs(v)) if len(v) else 0.0
    if vmax > 1e16:  # absolute nanoseconds (Livox PointXYZRTLT)
        return v * 1e-9 - header_stamp
    if vmax > 1e8:  # absolute seconds (RoboSense XYZIRT)
        return v - header_stamp
    if np.issubdtype(values.dtype, np.integer):  # relative nanoseconds (Ouster ``t``)
        return v * 1e-9
    return v  # relative seconds (canonical, Velodyne)


def pointcloud2_to_scan(msg) -> Scan:
    arr = pointcloud2_array(msg)
    names = arr.dtype.names
    xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)
    header = stamp_to_sec(msg.header.stamp)
    tf = next((n for n in _TIME_FIELDS if n in names), None)
    rel = _relative_times(arr[tf], header) if tf else np.zeros(len(arr))
    inf = next((n for n in _INTENSITY_FIELDS if n in names), None)
    intensity = arr[inf].astype(np.float32) if inf else np.zeros(len(arr), np.float32)
    ring = arr["ring"].astype(np.uint16) if "ring" in names else (
        arr["line"].astype(np.uint16) if "line" in names else None)
    keep = np.isfinite(xyz).all(axis=1) & np.isfinite(rel)
    return _make_scan(header, xyz[keep], intensity[keep], rel[keep],
                      ring[keep] if ring is not None else None, msg.header.frame_id)


def _make_scan(header: float, xyz, intensity, rel, ring, frame_id) -> Scan:
    if len(rel) == 0:
        return Scan(header, xyz, intensity, rel, ring, frame_id)
    t0 = float(rel.min())
    order = np.argsort(rel, kind="stable")
    return Scan(
        stamp=header + t0,
        xyz=xyz[order],
        intensity=intensity[order],
        time=rel[order] - t0,
        ring=ring[order] if ring is not None else None,
        frame_id=frame_id,
    )


def _align(pos: int, n: int) -> int:
    return (pos + n - 1) // n * n


def parse_livox_custom(raw: bytes | memoryview) -> tuple[float, str, int, np.ndarray]:
    """Decode a CDR ``livox_ros_driver2/msg/CustomMsg`` without per-point Python objects.

    Returns ``(header_stamp_s, frame_id, timebase_ns, points)``; ``points`` has fields
    t (offset ns), x, y, z, refl, tag, line.
    """
    buf = bytes(raw)
    if buf[1] not in (0, 1):
        raise ValueError("unknown CDR encapsulation")
    little = buf[1] == 1
    e = "<" if little else ">"
    body = memoryview(buf)[4:]
    sec, nsec, slen = struct.unpack_from(e + "iII", body, 0)
    frame_id = bytes(body[12 : 12 + slen - 1]).decode()
    pos = _align(12 + slen, 8)
    (timebase,) = struct.unpack_from(e + "Q", body, pos)
    pos += 8
    (point_num,) = struct.unpack_from(e + "I", body, pos)
    pos += 4 + 1 + 3
    pos = _align(pos, 4)
    (count,) = struct.unpack_from(e + "I", body, pos)
    pos += 4
    if count != point_num:
        point_num = count
    dtype = _LIVOX_DTYPE if little else _LIVOX_DTYPE.newbyteorder(">")
    start = 4 + pos
    chunk = buf[start : start + max(point_num * 20 - 1, 0)] + b"\x00"
    pts = np.frombuffer(chunk, dtype=dtype, count=point_num)
    return sec + nsec * 1e-9, frame_id, timebase, pts


def livox_custom_to_scan(raw, tag_filter: bool = True) -> Scan:
    header, frame_id, timebase, pts = parse_livox_custom(raw)
    if tag_filter:
        pts = pts[np.isin(pts["tag"] & 0x30, (0x00, 0x10))]
    xyz = np.column_stack([pts["x"], pts["y"], pts["z"]]).astype(np.float64)
    base = timebase * 1e-9 if timebase else header
    rel = pts["t"].astype(np.float64) * 1e-9
    keep = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0)
    return _make_scan(base, xyz[keep], pts["refl"][keep].astype(np.float32), rel[keep],
                      pts["line"][keep].astype(np.uint16), frame_id)


def decode_compressed_image(msg) -> np.ndarray:
    import cv2

    img = cv2.imdecode(np.asarray(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError(f"cannot decode CompressedImage with format {msg.format!r}")
    return img
