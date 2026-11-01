"""Live odometry on the canonical topics.

slam:=fastlio  FAST-LIO2 (default). TF: map -> odom -> camera_init -> body.
slam:=glim     GLIM online; needs a config made by `dt_glim_config --profile <p> --out <dir>`.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter, Node

from dt_bringup.rig import profile_dir, static_tf_args

IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def _setup(context):
    profile = LaunchConfiguration('profile').perform(context)
    slam = LaunchConfiguration('slam').perform(context)
    if slam == 'none':
        return []
    if slam == 'fastlio':
        return [
            Node(
                package='fast_lio', executable='fastlio_mapping', name='fast_lio',
                output='screen',
                parameters=[os.path.join(profile_dir(profile), 'fast_lio.yaml')]),
            # FAST-LIO hard-codes camera_init -> body; hang it under map/odom.
            Node(package='tf2_ros', executable='static_transform_publisher', name='tf_map_to_odom',
                 arguments=static_tf_args('map', 'odom', *IDENTITY), output='log'),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='tf_odom_to_camera_init',
                 arguments=static_tf_args('odom', 'camera_init', *IDENTITY), output='log'),
        ]
    if slam == 'glim':
        config_dir = LaunchConfiguration('glim_config').perform(context)
        if not config_dir:
            config_dir = os.path.join(os.environ.get('DT_DATA', os.path.expanduser('~/dt_data')),
                                      'glim_config', profile)
        if not os.path.isfile(os.path.join(config_dir, 'config.json')):
            raise RuntimeError(f'GLIM config not found in {config_dir}; run '
                               f'`ros2 run dt_tools dt_glim_config --profile {profile} --out {config_dir}`')
        return [Node(package='glim_ros', executable='glim_rosnode', name='glim', output='screen',
                     parameters=[{'config_path': config_dir}])]
    raise RuntimeError(f'unknown slam {slam!r}')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when replaying a bag with --clock (e.g. the Blender sim)'),
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        DeclareLaunchArgument('profile', default_value='mid360', choices=['mid360', 'airy']),
        DeclareLaunchArgument('slam', default_value='fastlio', choices=['fastlio', 'glim', 'none']),
        DeclareLaunchArgument('glim_config', default_value=''),
        OpaqueFunction(function=_setup),
    ])
