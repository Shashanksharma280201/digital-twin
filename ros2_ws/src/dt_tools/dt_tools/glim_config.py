"""Generate a GLIM config directory for a profile.

ros2 run dt_tools dt_glim_config --profile airy --out ~/dt_data/glim_config/airy [--cpu] [--viewer]

Copies GLIM's stock config (share/glim/config), then applies the profile's
glim_overrides.json and T_lidar_imu from rig.yaml. Comments in patched files are lost.
"""
import argparse
import json
import os
import shutil

from dt_tools.common import (deep_merge, load_jsonc, load_yaml, matrix_to_tum, pose_to_matrix,
                             profile_share_dir)


def glim_stock_config():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('glim'), 'config')


def build_config(stock_dir, profile_dir, out_dir, cpu=False, viewer=False):
    os.makedirs(out_dir, exist_ok=True)
    for name in os.listdir(stock_dir):
        src = os.path.join(stock_dir, name)
        if os.path.isfile(src):
            shutil.copy(src, os.path.join(out_dir, name))

    patches = load_jsonc(os.path.join(profile_dir, 'glim_overrides.json'))
    rig = load_yaml(os.path.join(profile_dir, 'rig.yaml'))
    patches.setdefault('config_sensors.json', {}).setdefault('sensors', {})['T_lidar_imu'] = [
        round(v, 8) for v in matrix_to_tum(pose_to_matrix(rig['T_lidar_imu']))]

    if cpu:
        def pick(*candidates):
            for c in candidates:
                if os.path.isfile(os.path.join(stock_dir, c)):
                    return c
            raise FileNotFoundError(f'none of {candidates} in {stock_dir}')
        patches.setdefault('config.json', {})['global'] = {
            'config_odometry': pick('config_odometry_cpu.json'),
            'config_sub_mapping': pick('config_sub_mapping_cpu.json',
                                       'config_sub_mapping_passthrough.json'),
            'config_global_mapping': pick('config_global_mapping_cpu.json',
                                          'config_global_mapping_pose_graph.json'),
        }

    for name, patch in patches.items():
        path = os.path.join(out_dir, name)
        cfg = deep_merge(load_jsonc(path), patch)
        if name == 'config_ros.json':
            modules = cfg['glim_ros'].get('extension_modules', [])
            if not viewer:
                modules = [m for m in modules if m != 'libstandard_viewer.so']
            cfg['glim_ros']['extension_modules'] = modules
        with open(path, 'w') as f:
            json.dump(cfg, f, indent=2)
    return out_dir


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--profile', required=True, choices=['mid360', 'airy'])
    parser.add_argument('--out', required=True)
    parser.add_argument('--cpu', action='store_true', help='use CPU odometry/mapping configs')
    parser.add_argument('--viewer', action='store_true', help='keep the GLIM standard viewer (needs a display)')
    parser.add_argument('--stock', default=None, help='GLIM stock config dir (default: share/glim/config)')
    args = parser.parse_args()
    out = build_config(args.stock or glim_stock_config(), profile_share_dir(args.profile),
                       os.path.expanduser(args.out), cpu=args.cpu, viewer=args.viewer)
    print(f'GLIM config written to {out}')
    print(f'  offline: ros2 run glim_ros glim_rosbag <session>/bag --ros-args -p config_path:={out}')


if __name__ == '__main__':
    main()
