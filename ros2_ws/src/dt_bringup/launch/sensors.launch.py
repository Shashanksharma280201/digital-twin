"""LiDAR driver (Mid-360 or Airy) + canonical bridge + ZED 2i + rig static TFs.

ros2 launch dt_bringup sensors.launch.py profile:=airy
ros2 launch dt_bringup sensors.launch.py profile:=mid360 mid360_ip:=192.168.1.142
"""
import json
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from dt_bringup.rig import load_rig, profile_dir, rig_static_transforms, static_tf_args


def _bridge_params(profile):
    with open(os.path.join(profile_dir(profile), 'bridge.yaml')) as f:
        params = yaml.safe_load(f)
    return params['points_bridge']['ros__parameters'], params['imu_bridge']['ros__parameters']


def _livox_driver(context):
    template = os.path.join(profile_dir('mid360'), 'MID360_config.json')
    with open(template) as f:
        cfg = json.load(f)
    cfg['lidar_configs'][0]['ip'] = LaunchConfiguration('mid360_ip').perform(context)
    out = os.path.join(tempfile.gettempdir(), 'dt_MID360_config.json')
    with open(out, 'w') as f:
        json.dump(cfg, f, indent=2)
    return Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,  # CustomMsg: keeps per-point offset_time
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': out,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )


def _rslidar_driver():
    return Node(
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        name='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_path': os.path.join(profile_dir('airy'), 'rslidar.yaml')}],
    )


def _setup(context):
    profile = LaunchConfiguration('profile').perform(context)
    actions = []

    if LaunchConfiguration('driver').perform(context) == 'true':
        actions.append(_livox_driver(context) if profile == 'mid360' else _rslidar_driver())

    if LaunchConfiguration('bridge').perform(context) == 'true':
        points_params, imu_params = _bridge_params(profile)
        actions.append(ComposableNodeContainer(
            name='dt_lidar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='dt_lidar_bridge', plugin='dt_lidar_bridge::PointsBridge',
                    name='points_bridge', parameters=[points_params],
                    extra_arguments=[{'use_intra_process_comms': True}]),
                ComposableNode(
                    package='dt_lidar_bridge', plugin='dt_lidar_bridge::ImuBridge',
                    name='imu_bridge', parameters=[imu_params],
                    extra_arguments=[{'use_intra_process_comms': True}]),
            ],
        ))

    for parent, child, xyz, q in rig_static_transforms(load_rig(profile)):
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'tf_{parent}_to_{child}', arguments=static_tf_args(parent, child, xyz, q),
            output='log'))

    if LaunchConfiguration('camera').perform(context) == 'true':
        zed_cfg = os.path.join(
            get_package_share_directory('dt_bringup'), 'config', 'zed',
            f"zed2i_{LaunchConfiguration('zed_config').perform(context)}.yaml")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')),
            launch_arguments={
                'camera_model': 'zed2i',
                'camera_name': 'zed',
                'publish_urdf': 'true',
                'publish_tf': 'false',
                'publish_map_tf': 'false',
                'publish_imu_tf': 'false',
                'ros_params_override_path': zed_cfg,
            }.items(),
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when replaying a bag with --clock (e.g. the Blender sim)'),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        DeclareLaunchArgument('profile', default_value='mid360', choices=['mid360', 'airy']),
        DeclareLaunchArgument('mid360_ip', default_value=os.environ.get('DT_MID360_IP', '192.168.1.12'),
                              description='Mid-360 IP: 192.168.1.1XX, XX = last two serial digits'),
        DeclareLaunchArgument('driver', default_value='true',
                              description='false when replaying raw driver topics from a bag'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('zed_config', default_value='capture', choices=['capture', 'depth']),
        OpaqueFunction(function=_setup),
    ])
