"""dt_blender_bridge: ROS 2 -> Blender live view (design: docs/design/2026-10-18-blender-sim-design.md §6).

Subscribes to the SLAM odometry and registered scans, accumulates a voxel map and serves
pose / cloud / ground-truth frames to the Blender add-on (sim/blender/addon/dt_live) over TCP.

ros2 run dt_blender_bridge dt_blender_bridge --ros-args -p rig_path:=$RUN/rig_sim.yaml -p use_sim_time:=true
"""
import collections
import queue
import socket
import threading

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from dt_blender_bridge.protocol import encode
from dt_tools.common import load_yaml, matrix_to_quat, pose_to_matrix, quat_to_matrix

_NP_TYPES = {PointField.FLOAT32: '<f4', PointField.FLOAT64: '<f8', PointField.UINT16: '<u2',
             PointField.UINT8: 'u1', PointField.INT32: '<i4', PointField.UINT32: '<u4'}


def cloud_to_xyzi(msg):
    """PointCloud2 -> (N, 4) float32 x, y, z, intensity (intensity 0 if absent)."""
    fields = {f.name: f for f in msg.fields if f.datatype in _NP_TYPES}
    names = [n for n in ('x', 'y', 'z', 'intensity') if n in fields]
    dtype = np.dtype({'names': names, 'formats': [_NP_TYPES[fields[n].datatype] for n in names],
                      'offsets': [fields[n].offset for n in names], 'itemsize': msg.point_step})
    raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)
    out = np.zeros((len(raw), 4), np.float32)
    for i, n in enumerate(names):
        out[:, i] = raw[n]
    return out[np.isfinite(out[:, :3]).all(axis=1)]


def odom_to_matrix(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix([q.x, q.y, q.z, q.w])
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def matrix_to_pose7(T):
    return np.concatenate([T[:3, 3], matrix_to_quat(T[:3, :3])]).astype(np.float32)


class VoxelMap:
    """Keeps one point per voxel; first point wins."""

    def __init__(self, voxel, max_points):
        self.voxel = voxel
        self.max_points = max_points
        self.keys = np.empty(0, np.int64)
        self.points = np.empty((0, 4), np.float32)
        self.version = 0

    def _key(self, xyz):
        k = np.floor(xyz / self.voxel).astype(np.int64) + (1 << 20)
        return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]

    def add(self, pts):
        if len(pts) == 0 or len(self.points) >= self.max_points:
            return
        keys = self._key(pts[:, :3])
        keys, idx = np.unique(keys, return_index=True)
        new = ~np.isin(keys, self.keys, assume_unique=True)
        if not new.any():
            return
        take = idx[new][: self.max_points - len(self.points)]
        self.keys = np.sort(np.concatenate([self.keys, self._key(pts[take, :3])]))
        self.points = np.concatenate([self.points, pts[take]])
        self.version += 1


class Server:
    """Accepts any number of Blender clients; each gets its own bounded send queue."""

    def __init__(self, host, port, logger, on_connect=None):
        self.logger = logger
        self.on_connect = on_connect
        self.clients = []
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.listen(4)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            conn, addr = self.sock.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            q = queue.Queue(maxsize=64)
            with self.lock:
                self.clients.append(q)
            self.logger.info(f'Blender connected from {addr[0]}:{addr[1]}')
            if self.on_connect:
                self.on_connect()
            threading.Thread(target=self._writer, args=(conn, q), daemon=True).start()

    def _writer(self, conn, q):
        try:
            while True:
                conn.sendall(q.get())
        except OSError:
            self.logger.info('Blender disconnected')
        finally:
            with self.lock:
                self.clients.remove(q)
            conn.close()

    def send(self, frame):
        # Never block ROS callbacks on a slow viewer: drop frames for that client instead.
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(frame)
            except queue.Full:
                pass

    @property
    def connected(self):
        return bool(self.clients)


class BlenderBridge(Node):
    def __init__(self):
        super().__init__('dt_blender_bridge')
        host = self.declare_parameter('host', '127.0.0.1').value
        port = self.declare_parameter('port', 5577).value
        voxel = self.declare_parameter('voxel', 0.05).value
        max_points = self.declare_parameter('max_points', 300000).value
        cloud_rate = self.declare_parameter('cloud_rate', 2.0).value
        rig_path = self.declare_parameter('rig_path', '').value
        odom_topic = self.declare_parameter('odom_topic', '/Odometry').value
        cloud_topic = self.declare_parameter('cloud_topic', '/cloud_registered').value
        gt_topic = self.declare_parameter('gt_topic', '/sim/ground_truth/odometry').value

        # World <- SLAM frame. Identity unless a GT stream and rig are available (sim only).
        self.T_world_slam = None if rig_path else np.eye(4)
        self.T_base_body = None
        if rig_path:
            rig = load_yaml(rig_path)
            self.T_base_body = pose_to_matrix(rig['T_base_lidar']) @ pose_to_matrix(rig['T_lidar_imu'])
        self.gt = collections.deque(maxlen=2000)
        self.pending_scans = collections.deque(maxlen=20)
        self.map = VoxelMap(voxel, max_points)
        self.sent_version = -1
        self.gt_count = 0

        self.server = Server(host, port, self.get_logger(), on_connect=self._resend_map)
        self.create_subscription(Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, gt_topic, self._on_gt, qos_profile_sensor_data)
        self.create_timer(1.0 / cloud_rate, self._send_cloud)
        self.get_logger().info(f'serving Blender on {host}:{port} (odom {odom_topic}, cloud {cloud_topic})')

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_gt(self, msg):
        T = odom_to_matrix(msg)
        self.gt.append((self._stamp(msg), T))
        self.gt_count += 1
        if self.gt_count % 5 == 0:  # GT arrives at 100 Hz; 20 Hz is plenty for display
            self.server.send(encode('gt_pose', matrix_to_pose7(T), self._stamp(msg)))

    def _gt_at(self, t):
        if not self.gt:
            return None
        times = np.fromiter((g[0] for g in self.gt), float)
        i = int(np.argmin(np.abs(times - t)))
        return self.gt[i][1] if abs(times[i] - t) < 0.05 else None

    def _on_odom(self, msg):
        T_slam_body = odom_to_matrix(msg)
        t = self._stamp(msg)
        if self.T_world_slam is None:
            T_world_base = self._gt_at(t)
            if T_world_base is None:
                return
            self.T_world_slam = T_world_base @ self.T_base_body @ np.linalg.inv(T_slam_body)
            self.get_logger().info('aligned SLAM frame to sim world using ground truth')
            while self.pending_scans:
                self.map.add(self._to_world(self.pending_scans.popleft()))
        T_world_base = self.T_world_slam @ T_slam_body
        if self.T_base_body is not None:
            T_world_base = T_world_base @ np.linalg.inv(self.T_base_body)
        self.server.send(encode('pose', matrix_to_pose7(T_world_base), t))

    def _to_world(self, pts):
        out = pts.copy()
        out[:, :3] = pts[:, :3] @ self.T_world_slam[:3, :3].T + self.T_world_slam[:3, 3]
        return out

    def _on_cloud(self, msg):
        pts = cloud_to_xyzi(msg)
        if self.T_world_slam is None:
            self.pending_scans.append(pts)
            return
        self.map.add(self._to_world(pts))

    def _resend_map(self):
        self.sent_version = -1

    def _send_cloud(self):
        if self.map.version == self.sent_version or not self.server.connected:
            return
        self.sent_version = self.map.version
        self.server.send(encode('cloud', self.map.points))


def main():
    rclpy.init()
    node = BlenderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
