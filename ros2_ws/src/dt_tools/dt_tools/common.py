"""Small shared helpers: JSON-with-comments, quaternion math, profile paths."""
import json
import math
import os

import numpy as np
import yaml


def strip_json_comments(text):
    """Removes // and /* */ comments (GLIM configs use them) outside of strings."""
    out = []
    i, n = 0, len(text)
    in_string = False
    while i < n:
        c = text[i]
        if in_string:
            out.append(c)
            if c == '\\' and i + 1 < n:
                out.append(text[i + 1])
                i += 2
                continue
            if c == '"':
                in_string = False
            i += 1
        elif c == '"':
            in_string = True
            out.append(c)
            i += 1
        elif text.startswith('//', i):
            j = text.find('\n', i)
            i = n if j < 0 else j
        elif text.startswith('/*', i):
            j = text.find('*/', i + 2)
            i = n if j < 0 else j + 2
        else:
            out.append(c)
            i += 1
    return ''.join(out)


def load_jsonc(path):
    with open(path) as f:
        return json.loads(strip_json_comments(f.read()))


def deep_merge(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m):
    t = np.trace(m)
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        q = [(m[2, 1] - m[1, 2]) * s, (m[0, 2] - m[2, 0]) * s, (m[1, 0] - m[0, 1]) * s, 0.25 / s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def pose_to_matrix(entry):
    """rig.yaml pose entry ({xyz, rpy} or {xyz, quat_xyzw}) -> 4x4."""
    T = np.eye(4)
    T[:3, 3] = entry.get('xyz', [0.0, 0.0, 0.0])
    if 'quat_xyzw' in entry:
        T[:3, :3] = quat_to_matrix(entry['quat_xyzw'])
    else:
        T[:3, :3] = rpy_to_matrix(*entry.get('rpy', [0.0, 0.0, 0.0]))
    return T


def matrix_to_pose(T):
    return {'xyz': [round(float(v), 6) for v in T[:3, 3]],
            'quat_xyzw': [round(float(v), 8) for v in matrix_to_quat(T[:3, :3])]}


def tum_to_matrix(v):
    """[x, y, z, qx, qy, qz, qw] -> 4x4."""
    T = np.eye(4)
    T[:3, 3] = v[:3]
    T[:3, :3] = quat_to_matrix(v[3:7])
    return T


def matrix_to_tum(T):
    return [float(v) for v in T[:3, 3]] + [float(v) for v in matrix_to_quat(T[:3, :3])]


def profile_share_dir(profile):
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('dt_bringup'), 'config', 'profiles', profile)


def profile_source_dir(profile):
    """Profile dir in the source tree (for writing calibration results), if DT_ROOT is set."""
    root = os.environ.get('DT_ROOT')
    if root:
        path = os.path.join(root, 'ros2_ws', 'src', 'dt_bringup', 'config', 'profiles', profile)
        if os.path.isdir(path):
            return path
    return None


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)
