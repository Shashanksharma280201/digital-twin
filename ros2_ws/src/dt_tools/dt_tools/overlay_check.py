"""Snapshot LiDAR-over-camera check: projects one LiDAR scan into one camera frame using TF.

ros2 run dt_tools dt_overlay_check --out overlay.png [--frames 5]

Points are coloured by distance (near = red, far = blue). If the LiDAR->camera geometry is right,
the coloured points follow the edges in the image (columns, door frames, people).
"""
import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2
from tf2_ros import Buffer, TransformListener

from dt_tools.common import quat_to_matrix

IMAGE = '/zed/zed_node/rgb/color/rect/image/compressed'
INFO = '/zed/zed_node/rgb/color/rect/camera_info'
POINTS = '/dt/lidar/points'


def cloud_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    return np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)


def project(xyz_lidar, T_cam_lidar, K, w, h):
    pc = xyz_lidar @ T_cam_lidar[:3, :3].T + T_cam_lidar[:3, 3]
    z = pc[:, 2]
    ok = z > 0.3
    uv = (pc[ok, :2] / z[ok, None]) * [K[0], K[4]] + [K[2], K[5]]
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    return uv[inside], z[ok][inside]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default='overlay.png')
    ap.add_argument('--frames', type=int, default=5, help='LiDAR scans to accumulate (rig must be still)')
    ap.add_argument('--timeout', type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('dt_overlay_check')
    tf = Buffer()
    TransformListener(tf, node)
    got = {'img': None, 'info': None, 'clouds': []}
    node.create_subscription(CompressedImage, IMAGE, lambda m: got.__setitem__('img', m), 2)
    node.create_subscription(CameraInfo, INFO, lambda m: got.__setitem__('info', m), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, POINTS, lambda m: got['clouds'].append(m), qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < args.timeout and (
            got['img'] is None or got['info'] is None or len(got['clouds']) < args.frames):
        rclpy.spin_once(node, timeout_sec=0.1)
    if got['img'] is None or got['info'] is None or not got['clouds']:
        raise SystemExit(f"missing data: image={got['img'] is not None} info={got['info'] is not None} "
                         f"clouds={len(got['clouds'])}")
    optical = got['info'].header.frame_id
    lidar = got['clouds'][0].header.frame_id
    while not tf.can_transform(optical, lidar, rclpy.time.Time()) and time.time() - t0 < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    t = tf.lookup_transform(optical, lidar, rclpy.time.Time()).transform
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
    T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]

    img = cv2.imdecode(np.frombuffer(bytes(got['img'].data), np.uint8), cv2.IMREAD_COLOR)
    info = got['info']
    sx, sy = img.shape[1] / info.width, img.shape[0] / info.height
    K = np.array(info.k, float)
    K[[0, 2]] *= sx
    K[[4, 5]] *= sy
    xyz = np.vstack([cloud_xyz(c) for c in got['clouds'][: args.frames]])
    uv, z = project(xyz, T, K, img.shape[1], img.shape[0])
    hue = np.clip((z - 0.5) / 8.0, 0, 1) * 120  # OpenCV hue: 0 red (near) .. 120 blue (far)
    colors = cv2.cvtColor(np.stack([hue, np.full_like(hue, 255), np.full_like(hue, 255)], 1)
                          .astype(np.uint8)[None], cv2.COLOR_HSV2BGR)[0]
    out = img.copy()
    for (u, v), c in zip(uv.astype(int), colors):
        cv2.circle(out, (u, v), 1, tuple(int(x) for x in c), -1)
    cv2.putText(out, f'{lidar} -> {optical}  t=[{T[0,3]:+.3f} {T[1,3]:+.3f} {T[2,3]:+.3f}] m  '
                f'{len(uv)} pts in view', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(args.out, np.hstack([img, out]))
    print(f'wrote {args.out}: {len(uv)} of {len(xyz)} LiDAR points fall inside the image')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
