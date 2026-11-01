"""Import a direct_visual_lidar_calibration result into the profile's rig.yaml.

ros2 run dt_tools dt_apply_camera_calib calib.json --profile airy

calib.json["results"]["T_lidar_camera"] is [x, y, z, qx, qy, qz, qw] for the camera
optical frame of the calibrated image (the rectified left image). With the ZED wrapper
running, this tool reads that frame from camera_info, looks up zed_camera_link -> optical
in TF, and writes both T_lidar_camera_optical and T_lidar_camera (to zed_camera_link).
"""
import argparse
import os

import numpy as np
import yaml

from dt_tools.common import (load_yaml, matrix_to_pose, profile_share_dir, profile_source_dir,
                             quat_to_matrix, tum_to_matrix)


def load_calib(path):
    with open(path) as f:
        data = yaml.safe_load(f)  # JSON is valid YAML
    try:
        values = data['results']['T_lidar_camera']
    except (KeyError, TypeError):
        raise ValueError(f'{path}: expected results.T_lidar_camera') from None
    if len(values) != 7:
        raise ValueError(f'{path}: T_lidar_camera must be [x y z qx qy qz qw]')
    return tum_to_matrix(values)


def camera_link_pose(T_lidar_optical, T_link_optical):
    """T_lidar_link = T_lidar_optical * inv(T_link_optical)."""
    return T_lidar_optical @ np.linalg.inv(T_link_optical)


def lookup_link_to_optical(camera_frame, info_topic, timeout=10.0):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('dt_apply_camera_calib')
    buffer = Buffer()
    TransformListener(buffer, node)
    frame = {}
    node.create_subscription(CameraInfo, info_topic,
                             lambda m: frame.setdefault('id', m.header.frame_id), qos_profile_sensor_data)
    try:
        deadline = node.get_clock().now().nanoseconds + int(timeout * 1e9)
        while 'id' not in frame and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if 'id' not in frame:
            raise RuntimeError(f'no CameraInfo on {info_topic}; is the ZED running?')
        optical = frame['id']
        while node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if buffer.can_transform(camera_frame, optical, rclpy.time.Time()):
                tf = buffer.lookup_transform(camera_frame, optical, rclpy.time.Time()).transform
                T = np.eye(4)
                T[:3, 3] = [tf.translation.x, tf.translation.y, tf.translation.z]
                T[:3, :3] = quat_to_matrix([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
                return optical, T
        raise RuntimeError(f'no TF {camera_frame} -> {optical} (publish_urdf must be true)')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('calib_json')
    parser.add_argument('--profile', required=True, choices=['mid360', 'airy'])
    parser.add_argument('--rig', default=None, help='rig.yaml to update (default: source tree via DT_ROOT)')
    parser.add_argument('--camera-frame', default='zed_camera_link')
    parser.add_argument('--camera-info', default='/zed/zed_node/rgb/color/rect/camera_info')
    args = parser.parse_args()

    rig_path = args.rig
    if rig_path is None:
        src = profile_source_dir(args.profile)
        rig_path = os.path.join(src or profile_share_dir(args.profile), 'rig.yaml')

    T_lidar_optical = load_calib(args.calib_json)
    optical, T_link_optical = lookup_link_to_optical(args.camera_frame, args.camera_info)
    T_lidar_link = camera_link_pose(T_lidar_optical, T_link_optical)

    rig = load_yaml(rig_path)
    rig['T_lidar_camera_optical'] = matrix_to_pose(T_lidar_optical)
    rig['T_lidar_camera'] = matrix_to_pose(T_lidar_link)
    rig['camera_optical_frame'] = optical
    with open(rig_path, 'w') as f:
        f.write('# Updated by dt_apply_camera_calib from %s\n' % os.path.abspath(args.calib_json))
        yaml.safe_dump(rig, f, sort_keys=False, default_flow_style=None)
    print(f'updated {rig_path}')
    print(f'  T_lidar_camera_optical ({optical}): {rig["T_lidar_camera_optical"]}')
    print(f'  T_lidar_camera ({args.camera_frame}): {rig["T_lidar_camera"]}')
    print('Rebuild is not needed with --symlink-install; restart the launch to apply.')


if __name__ == '__main__':
    main()
