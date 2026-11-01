"""Load the DT Live add-on without installing it, optionally auto-connecting.

blender $RUN/world/walk_preview_day1.blend -P sim/blender/addon/run_live.py -- --connect 127.0.0.1:5577
"""
import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dt_live  # noqa: E402


def _args():
    argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    target = None
    if '--connect' in argv:
        target = argv[argv.index('--connect') + 1]
    return target


if not hasattr(bpy.types.Scene, 'dt_host'):
    dt_live.register()
target = _args()
if target:
    host, _, port = target.partition(':')
    try:
        dt_live.connect(host, int(port or 5577))
        print(f'[dt_live] connected to {target}')
    except OSError as exc:
        print(f'[dt_live] could not connect to {target}: {exc}; use the DT Live sidebar to retry')
