"""Full live stack: sensors + bridge + SLAM + health monitor (+ RViz).

ros2 launch dt_bringup capture.launch.py profile:=airy
ros2 launch dt_bringup capture.launch.py profile:=mid360 mid360_ip:=192.168.1.142 rviz:=true
Recording is started separately with `ros2 run dt_tools dt_session record ...`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter, Node


def _include(name, args):
    path = os.path.join(get_package_share_directory('dt_bringup'), 'launch', name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments={k: LaunchConfiguration(k) for k in args}.items())


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution([get_package_share_directory('dt_bringup'), 'rviz',
                                     [LaunchConfiguration('rviz_config'), '.rviz']])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when replaying a bag with --clock (e.g. the Blender sim)'),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        DeclareLaunchArgument('profile', default_value='mid360', choices=['mid360', 'airy']),
        DeclareLaunchArgument('mid360_ip', default_value=os.environ.get('DT_MID360_IP', '192.168.1.12')),
        DeclareLaunchArgument('driver', default_value='true'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('zed_config', default_value='capture', choices=['capture', 'depth']),
        DeclareLaunchArgument('slam', default_value='fastlio', choices=['fastlio', 'glim', 'none']),
        DeclareLaunchArgument('glim_config', default_value=''),
        DeclareLaunchArgument('monitor', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value='dt', choices=['dt', 'calib', 'live'],
                              description='calib: sensor frames + LiDAR-over-camera overlay; '
                                          'live: map growing during the walk'),
        _include('sensors.launch.py',
                 ['profile', 'mid360_ip', 'driver', 'bridge', 'camera', 'zed_config', 'use_sim_time']),
        _include('slam.launch.py', ['profile', 'slam', 'glim_config', 'use_sim_time']),
        Node(package='dt_tools', executable='dt_monitor', name='dt_monitor', output='screen',
             parameters=[{'camera': LaunchConfiguration('camera'),
                          'profile': LaunchConfiguration('profile')}],
             condition=IfCondition(LaunchConfiguration('monitor'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
