import json
import math

import numpy as np
import pytest
from diagnostic_msgs.msg import DiagnosticStatus

from dt_tools.apply_camera_calib import camera_link_pose, load_calib
from dt_tools.common import (matrix_to_quat, pose_to_matrix, quat_to_matrix, strip_json_comments,
                             tum_to_matrix)
from dt_tools.glim_config import build_config
from dt_tools.monitor import check_gravity, classify_latency, coach
from dt_tools.session import session_topics

GLIM_ROS = """{
  /*** ROS-related config ***/
  // url-like strings must survive: "http://x"
  "glim_ros": {
    "acc_scale": 0.0, // Auto-detect if zero
    "extension_modules": ["libmemory_monitor.so", "libstandard_viewer.so", "librviz_viewer.so"],
    "imu_topic": "/os_cloud_node/imu",
    "points_topic": "/os_cloud_node/points",
    "note": "a // b"
  }
}"""
GLIM_SENSORS = '{ "sensors": { "T_lidar_imu": [0,0,0,0,0,0,1], "ring_field": "" } }'
GLIM_GLOBAL = '{ "global": { "config_odometry": "config_odometry_gpu.json" } }'


def test_strip_json_comments_keeps_strings():
    data = json.loads(strip_json_comments(GLIM_ROS))
    assert data['glim_ros']['note'] == 'a // b'
    assert data['glim_ros']['acc_scale'] == 0.0


def test_quaternion_roundtrip():
    q = np.array([0.1, -0.3, 0.2, 0.9])
    q /= np.linalg.norm(q)
    back = matrix_to_quat(quat_to_matrix(q))
    assert np.allclose(back, q) or np.allclose(back, -q)


def test_glim_config_patch(tmp_path):
    stock = tmp_path / 'stock'
    stock.mkdir()
    (stock / 'config_ros.json').write_text(GLIM_ROS)
    (stock / 'config_sensors.json').write_text(GLIM_SENSORS)
    (stock / 'config.json').write_text(GLIM_GLOBAL)
    (stock / 'config_odometry_cpu.json').write_text('{}')
    (stock / 'config_sub_mapping_passthrough.json').write_text('{}')
    (stock / 'config_global_mapping_pose_graph.json').write_text('{}')
    profile = tmp_path / 'profile'
    profile.mkdir()
    (profile / 'glim_overrides.json').write_text(json.dumps({
        'config_ros.json': {'glim_ros': {'points_topic': '/dt/lidar/points', 'acc_scale': 1.0}}}))
    (profile / 'rig.yaml').write_text(
        'T_lidar_imu: {xyz: [0.011, 0.02329, -0.04412], rpy: [0, 0, 0]}\n')

    out = build_config(str(stock), str(profile), str(tmp_path / 'out'), cpu=True)
    ros = json.loads((tmp_path / 'out' / 'config_ros.json').read_text())['glim_ros']
    assert ros['points_topic'] == '/dt/lidar/points'
    assert ros['acc_scale'] == 1.0
    assert 'libstandard_viewer.so' not in ros['extension_modules']
    sensors = json.loads((tmp_path / 'out' / 'config_sensors.json').read_text())['sensors']
    assert sensors['T_lidar_imu'] == pytest.approx([0.011, 0.02329, -0.04412, 0, 0, 0, 1])
    glob = json.loads((tmp_path / 'out' / 'config.json').read_text())['global']
    assert glob['config_odometry'] == 'config_odometry_cpu.json'
    assert glob['config_sub_mapping'] == 'config_sub_mapping_passthrough.json'
    assert out == str(tmp_path / 'out')


def test_camera_link_from_optical(tmp_path):
    # ROS link -> optical: z forward, x right, y down.
    T_link_optical = np.eye(4)
    T_link_optical[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    T_link_optical[:3, 3] = [0.0, 0.06, 0.0]  # left lens offset
    T_lidar_link = pose_to_matrix({'xyz': [0.1, 0.0, -0.05], 'rpy': [0.0, 0.05, 0.0]})
    T_lidar_optical = T_lidar_link @ T_link_optical
    calib = tmp_path / 'calib.json'
    q = matrix_to_quat(T_lidar_optical[:3, :3])
    calib.write_text(json.dumps({'results': {'T_lidar_camera': list(T_lidar_optical[:3, 3]) + list(q)}}))
    assert np.allclose(load_calib(str(calib)), T_lidar_optical, atol=1e-9)
    assert np.allclose(camera_link_pose(load_calib(str(calib)), T_link_optical), T_lidar_link, atol=1e-9)


def test_tum_matrix():
    T = tum_to_matrix([1, 2, 3, 0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    assert np.allclose(T[:3, :3] @ [1, 0, 0], [0, 1, 0])
    assert np.allclose(T[:3, 3], [1, 2, 3])


@pytest.mark.parametrize('latency,level', [
    (0.02, DiagnosticStatus.OK),
    (0.5, DiagnosticStatus.WARN),
    (-0.2, DiagnosticStatus.WARN),
    (37.01, DiagnosticStatus.ERROR),
    (1.7e9, DiagnosticStatus.ERROR),
    (None, DiagnosticStatus.ERROR),
])
def test_classify_latency(latency, level):
    assert classify_latency(latency)[0] == level


def test_check_gravity():
    assert check_gravity((0.0, 0.1, 9.8))[0] == DiagnosticStatus.OK
    assert check_gravity((0.0, 0.0, 1.0))[0] == DiagnosticStatus.ERROR  # still in g
    assert check_gravity((0.0, 0.0, -9.8))[0] == DiagnosticStatus.WARN  # flipped


def test_session_topics():
    assert '/rslidar_points' in session_topics('airy')
    assert '/rslidar_points' not in session_topics('airy', lean=True)
    assert not any('zed' in t for t in session_topics('mid360', camera=False))


def _samples(seconds, speed_mps, turn_dps, hz=10):
    return [(i / hz, speed_mps * i / hz, 0.0, 0.0, math.radians(turn_dps * i / hz)) for i in range(int(seconds * hz))]


def test_coach_good_pace_walking():
    level, msg, turn, speed = coach(_samples(12, 0.4, 10))
    assert level == DiagnosticStatus.OK and abs(speed - 0.4) < 0.01 and abs(turn - 10) < 0.5


def test_coach_warns_fast_turn_and_spin():
    assert 'turning' in coach(_samples(3, 0.0, 60))[1]
    level, msg, _, _ = coach(_samples(12, 0.02, 25))  # 300 deg in 12 s, barely moving
    assert level == DiagnosticStatus.WARN and 'walk while turning' in msg
    assert 'walking' in coach(_samples(3, 1.0, 0))[1]
