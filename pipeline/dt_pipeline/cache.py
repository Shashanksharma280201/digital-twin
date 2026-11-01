"""Scan cache: the recording's LiDAR scans decoded once into flat arrays (``products/cache/``).

Reading a 12 GB bag and decoding PointCloud2 costs ~25 ms per scan; refine, map, mesh and the
quality report each used to repeat it. ``ingest`` reads the bag once (scans decoded in worker
processes, the odometry on the way), filters each scan and writes::

    cache/points.f32    (N, 5) float32: x, y, z (sensor frame), t (s since scan stamp), intensity
    cache/index.npz     stamps (S,) float64, offsets (S+1,) int64

Filters: range limits, and "mixed pixels": at a depth edge the beam partly hits the front and
the back surface and the sensor reports a point floating between them. Such a point differs
in range from both of its neighbours on the same laser ring; it is dropped.

A per-ring calibration (``ringcal``), when given, is applied here, so every later step sees corrected
points.
"""

from __future__ import annotations

import logging
from collections import deque
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from .geometry import Trajectory, inv
from .io import ODOM_TOPIC, POINTS_TOPIC, Bag
from .io.bag import pointcloud2_to_scan, stamp_to_sec, typestore
from .ringcal import apply as apply_ring_calib

log = logging.getLogger("dtp")

_TS = None


def mixed_pixels(r: np.ndarray, ring: np.ndarray, t: np.ndarray, rel: float = 0.03, floor: float = 0.05) -> np.ndarray:
    """Mask of points whose range jumps against both ring neighbours (time order within a ring)."""
    order = np.lexsort((t, ring))
    rs, rg = r[order], ring[order]
    jump = np.maximum(rel * rs, floor)
    prev = np.abs(np.diff(rs, prepend=np.nan))
    nxt = np.abs(np.diff(rs, append=np.nan))
    same_prev = np.concatenate([[False], rg[1:] == rg[:-1]])
    same_next = np.concatenate([rg[:-1] == rg[1:], [False]])
    bad_sorted = same_prev & same_next & (prev > jump) & (nxt > jump)
    bad = np.zeros(len(r), bool)
    bad[order] = bad_sorted
    return bad


def _decode(args):
    """Worker: CDR PointCloud2 -> (stamp, float32 (n, 5), dropped mixed pixels)."""
    global _TS
    data, msgtype, min_range, max_range, drop_mixed, calib = args
    if _TS is None:
        _TS = typestore()
    scan = pointcloud2_to_scan(_TS.deserialize_cdr(data, msgtype))
    xyz = scan.xyz
    if calib is not None and scan.ring is not None and len(xyz):
        xyz = apply_ring_calib(xyz, scan.ring, *calib)
    r = np.linalg.norm(xyz, axis=1)
    keep = (r >= min_range) & (r <= max_range)
    n_mixed = 0
    if drop_mixed and scan.ring is not None and len(r):
        mixed = mixed_pixels(r, scan.ring, scan.time) & keep
        n_mixed = int(mixed.sum())
        keep &= ~mixed
    arr = np.empty((int(keep.sum()), 5), np.float32)
    arr[:, :3] = xyz[keep]
    arr[:, 3] = scan.time[keep]
    arr[:, 4] = scan.intensity[keep]
    return scan.stamp, arr, n_mixed


class ScanCache:
    def __init__(self, directory: str | Path):
        self.dir = Path(directory)
        idx = np.load(self.dir / "index.npz")
        self.stamps, self.offsets = idx["stamps"], idx["offsets"]
        self.points = np.memmap(self.dir / "points.f32", dtype=np.float32, mode="r",
                                shape=(int(self.offsets[-1]), 5))

    @staticmethod
    def exists(directory: str | Path) -> bool:
        return (Path(directory) / "index.npz").exists()

    def __len__(self) -> int:
        return len(self.stamps)

    def scan(self, i: int) -> np.ndarray:
        return self.points[self.offsets[i]:self.offsets[i + 1]]

    def batches(self, max_points: int = 4_000_000, every: int = 1, max_scans: int | None = None):
        """Yield ``(scan_indices, stamps_per_point, points)`` in time order, ~max_points at a time."""
        idx = np.arange(0, len(self), every)
        start = 0
        while start < len(idx):
            end, total = start, 0
            while end < len(idx) and (total == 0 or total + self.offsets[idx[end] + 1] - self.offsets[idx[end]] <= max_points) \
                    and (max_scans is None or end - start < max_scans):
                total += self.offsets[idx[end] + 1] - self.offsets[idx[end]]
                end += 1
            sel = idx[start:end]
            if every == 1:
                pts = np.asarray(self.points[self.offsets[sel[0]]:self.offsets[sel[-1] + 1]])
            else:
                pts = np.concatenate([self.scan(i) for i in sel])
            counts = self.offsets[sel + 1] - self.offsets[sel]
            yield sel, np.repeat(self.stamps[sel], counts), pts
            start = end


def ingest(bag_path: str | Path, out_dir: str | Path, T_lidar_imu: np.ndarray, topic: str = POINTS_TOPIC,
           odom_topic: str = ODOM_TOPIC, workers: int = 8, min_range: float = 0.3, max_range: float = 60.0,
           drop_mixed: bool = True, ring_calib: tuple[np.ndarray, np.ndarray] | None = None
           ) -> tuple[ScanCache, Trajectory | None, dict]:
    """One pass over the bag: scan cache + FAST-LIO trajectory (``T_world_lidar``)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    stamps, offsets = [], [0]
    o_stamps, o_xyz, o_quat = [], [], []
    mixed_total = 0
    with Bag(bag_path) as bag, open(out / "points.f32", "wb") as f, ProcessPoolExecutor(workers) as pool:
        msgtype = bag.msgtype(topic)
        if msgtype != "sensor_msgs/msg/PointCloud2":
            raise TypeError(f"{topic}: cache needs the canonical PointCloud2 topic, got {msgtype}")
        topics = [topic] + ([odom_topic] if odom_topic in bag.topics else [])
        pending: deque = deque()

        def drain(block: bool):
            nonlocal mixed_total
            while pending and (block or pending[0].done() or len(pending) > 3 * workers):
                stamp, arr, n_mixed = pending.popleft().result()
                if len(arr):
                    f.write(arr.tobytes())
                    stamps.append(stamp)
                    offsets.append(offsets[-1] + len(arr))
                    mixed_total += n_mixed

        for t, mt, _, data in bag.raw(topics):
            if t == topic:
                pending.append(pool.submit(_decode, (bytes(data), mt, min_range, max_range, drop_mixed, ring_calib)))
                drain(False)
            else:
                msg = bag._reader.deserialize(data, mt)
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                o_stamps.append(stamp_to_sec(msg.header.stamp))
                o_xyz.append((p.x, p.y, p.z))
                o_quat.append((q.x, q.y, q.z, q.w))
        drain(True)
    np.savez(out / "index.npz", stamps=np.asarray(stamps), offsets=np.asarray(offsets, np.int64))
    traj = None
    if len(o_stamps) >= 2:
        from scipy.spatial.transform import Rotation

        T = np.tile(np.eye(4), (len(o_stamps), 1, 1))
        T[:, :3, :3] = Rotation.from_quat(np.asarray(o_quat)).as_matrix()
        T[:, :3, 3] = o_xyz
        traj = Trajectory(np.asarray(o_stamps), T).compose(inv(T_lidar_imu))
    stats = {"scans": len(stamps), "points": int(offsets[-1]), "mixed_pixels_dropped": mixed_total,
             "ring_calibrated": ring_calib is not None}
    return ScanCache(out), traj, stats
