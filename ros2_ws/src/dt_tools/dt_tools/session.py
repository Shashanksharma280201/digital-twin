"""dt_session: record capture sessions and calibration bags.

ros2 run dt_tools dt_session record --site office --profile airy [--raw] [--no-svo] [--duration 300]
ros2 run dt_tools dt_session calib-bags --profile mid360 [--count 6] [--duration 15]

Session layout (design §8):
  $DT_DATA/sessions/<site>/<YYYYmmdd_HHMMSS>_<profile>/{bag/, zed.svo2, meta.yaml, rig.yaml}
"""
import argparse
import datetime
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

import yaml

from dt_tools.common import profile_share_dir

CANONICAL = ['/dt/lidar/points', '/dt/lidar/imu']
RAW = {
    'mid360': ['/livox/lidar', '/livox/imu'],
    'airy': ['/rslidar_points', '/rslidar_imu_data'],
}
CAMERA = [
    '/zed/zed_node/rgb/color/rect/image/compressed',
    '/zed/zed_node/rgb/color/rect/camera_info',
    '/zed/zed_node/imu/data',
]
COMMON = ['/Odometry', '/tf', '/tf_static', '/diagnostics']
SVO_SERVICE = '/zed/zed_node/start_svo_rec'
SVO_STOP_SERVICE = '/zed/zed_node/stop_svo_rec'


def session_topics(profile, lean=False, camera=True):
    topics = list(CANONICAL)
    if not lean:
        topics += RAW[profile]
    if camera:
        topics += CAMERA
    return topics + COMMON


def data_root():
    return os.path.expanduser(os.environ.get('DT_DATA', '~/dt_data'))


def git_describe():
    root = os.environ.get('DT_ROOT')
    if not root:
        return None
    try:
        return subprocess.run(['git', '-C', root, 'describe', '--always', '--dirty'],
                              capture_output=True, text=True, timeout=5).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def ptp_snapshot():
    # ptp4l's socket is root-only; use passwordless sudo when available, else say so plainly.
    cmd = ['pmc', '-u', '-b', '0', 'GET TIME_STATUS_NP']
    try:
        for prefix in ([], ['sudo', '-n']):
            out = subprocess.run(prefix + cmd, capture_output=True, text=True, timeout=3)
            if out.stdout.strip():
                return out.stdout.strip()
        return 'unavailable (pmc needs root); IMU latency in dt_monitor is the lock check'
    except (OSError, subprocess.SubprocessError):
        return None


def bag_command(out_dir, topics):
    return ['ros2', 'bag', 'record', '-s', 'mcap', '--storage-preset-profile', 'zstd_fast',
            '--max-cache-size', str(1024 ** 3), '-o', out_dir] + topics


class SvoRecorder:
    """Starts/stops ZED SVO2 recording through the wrapper's services (H.265, NVENC)."""

    def __init__(self):
        import rclpy
        from zed_msgs.srv import StartSvoRec
        from std_srvs.srv import Trigger
        from rclpy.signals import SignalHandlerOptions
        # Keep Ctrl-C for record(): rclpy's own handler would shut the context down before
        # stop_svo_rec can be called, leaving the ZED still writing the SVO.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._rclpy = rclpy
        self.node = rclpy.create_node('dt_session_svo')
        self.start_cli = self.node.create_client(StartSvoRec, SVO_SERVICE)
        self.stop_cli = self.node.create_client(Trigger, SVO_STOP_SERVICE)
        self._start_type = StartSvoRec
        self._trigger_type = Trigger

    def _call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {client.srv_name} not available (is the ZED running?)')
        future = client.call_async(request)
        self._rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f'{client.srv_name} timed out')
        return future.result()

    def start(self, path):
        req = self._start_type.Request()
        req.svo_filename = path
        req.compression_mode = 0  # H.265
        res = self._call(self.start_cli, req)
        if not res.success:
            raise RuntimeError(f'start_svo_rec failed: {res.message}')

    def stop(self):
        res = self._call(self.stop_cli, self._trigger_type.Request())
        return res.success

    def close(self):
        self.node.destroy_node()
        self._rclpy.try_shutdown()


def record(args):
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    session = os.path.join(data_root(), 'sessions', args.site, f'{stamp}_{args.profile}')
    os.makedirs(session)
    shutil.copy(args.rig or os.path.join(profile_share_dir(args.profile), 'rig.yaml'),
                os.path.join(session, 'rig.yaml'))
    topics = session_topics(args.profile, lean=not args.raw, camera=not args.no_camera)
    meta = {
        'site': args.site,
        'profile': args.profile,
        'note': args.note,
        'host': socket.gethostname(),
        'git': git_describe(),
        'start': datetime.datetime.now().astimezone().isoformat(),
        'topics': topics,
        'ptp_status_start': ptp_snapshot(),
    }
    if args.rehearsal:
        meta['rehearsal'] = True  # replayed bag: the sensor's profile calibration does not apply

    svo = None
    if not args.no_svo and not args.no_camera:
        try:
            svo = SvoRecorder()
            svo.start(os.path.join(session, 'zed.svo2'))
            meta['svo'] = 'zed.svo2'
        except Exception as exc:  # recording LiDAR must not depend on the camera
            print(f'[dt_session] WARNING: SVO recording disabled: {exc}', file=sys.stderr)
            if svo:
                svo.close()
            svo = None

    with open(os.path.join(session, 'meta.yaml'), 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)

    print(f'[dt_session] recording to {session}')
    print('[dt_session] keep the rig STILL for ~3 s so LIO can initialise; Ctrl-C to stop')
    proc = subprocess.Popen(bag_command(os.path.join(session, 'bag'), topics))
    if svo:
        # ZED wrapper v5.2.2 (JetPack 6.0) accepts the request but writes no frames; catch it early.
        time.sleep(3.0)
        svo_file = os.path.join(session, 'zed.svo2')
        if not os.path.exists(svo_file) or os.path.getsize(svo_file) < 100_000:
            print('[dt_session] WARNING: zed.svo2 is not growing; colour will come from the bag images',
                  file=sys.stderr)
            try:
                svo.stop()
            except Exception:
                pass
            svo.close()
            svo = None
            meta['svo'] = None
            meta['svo_note'] = 'wrapper accepted start_svo_rec but wrote no frames'
            with open(os.path.join(session, 'meta.yaml'), 'w') as f:
                yaml.safe_dump(meta, f, sort_keys=False)
    try:
        if args.duration:
            proc.wait(timeout=args.duration)
        else:
            proc.wait()
    except subprocess.TimeoutExpired:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                proc.kill()
        if svo:
            try:
                svo.stop()
            except Exception as exc:
                print(f'[dt_session] WARNING: stopping SVO failed: {exc}', file=sys.stderr)
            svo.close()

    meta['end'] = datetime.datetime.now().astimezone().isoformat()
    meta['ptp_status_end'] = ptp_snapshot()
    with open(os.path.join(session, 'meta.yaml'), 'w') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    print(f'[dt_session] done: {session}')
    return session


def calib_bags(args):
    """Short bags for direct_visual_lidar_calibration (design §11)."""
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = os.path.join(data_root(), 'calib', f'{stamp}_{args.profile}')
    os.makedirs(out)
    topics = ['/dt/lidar/points', '/zed/zed_node/rgb/color/rect/image',
              '/zed/zed_node/rgb/color/rect/camera_info']
    hint = ('hold the rig completely still' if args.profile == 'mid360'
            else 'sweep the rig slowly up and down')
    for i in range(args.count):
        input(f'[dt_session] bag {i + 1}/{args.count}: point at a feature-rich scene, '
              f'press Enter, then {hint} for {args.duration:.0f} s ')
        proc = subprocess.Popen(['ros2', 'bag', 'record', '-o', os.path.join(out, f'calib_{i:02d}')]
                                + topics)
        time.sleep(args.duration)
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=30)
    print(f'[dt_session] calibration bags in {out}; copy to the x86 PC and run '
          'direct_visual_lidar_calibration (see docs/design §11)')
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='cmd', required=True)

    rec = sub.add_parser('record', help='record a capture session')
    rec.add_argument('--site', required=True)
    rec.add_argument('--profile', required=True, choices=sorted(RAW))
    rec.add_argument('--note', default='')
    # Raw driver clouds duplicate /dt/lidar/points and double the bag (12 GB for 6 min with the Airy),
    # so they are only recorded on request (bridge debugging).
    rec.add_argument('--raw', action='store_true', help='also record the raw driver topics')
    rec.add_argument('--lean', action='store_true', help=argparse.SUPPRESS)  # default since 2026-10-22
    rec.add_argument('--no-svo', action='store_true', help='do not record the ZED SVO2')
    rec.add_argument('--no-camera', action='store_true', help='LiDAR-only session')
    rec.add_argument('--duration', type=float, default=None, help='seconds (default: until Ctrl-C)')
    rec.add_argument('--rig', default=None, help='rig.yaml to store instead of the profile one')
    rec.add_argument('--rehearsal', action='store_true', help='mark a session recorded from a replayed bag')
    rec.set_defaults(func=record)

    cal = sub.add_parser('calib-bags', help='record LiDAR-camera calibration bags')
    cal.add_argument('--profile', required=True, choices=sorted(RAW))
    cal.add_argument('--count', type=int, default=6)
    cal.add_argument('--duration', type=float, default=15.0)
    cal.set_defaults(func=calib_bags)

    args = parser.parse_args()
    args.func(args)


if __name__ == '__main__':
    main()
