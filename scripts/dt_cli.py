#!/usr/bin/env python3
"""dt: record, process and view digital twins from one terminal.

  dt record   start the rig stack and record, with a running clock and live health
  dt process  turn a recording into the twin, with a clock on every step
  dt run      record, then process
  dt view     open the finished 3D model
  dt live     open the live map on a stack that is already running
  dt list     recordings and how long each one took
  dt doctor   check this machine
  dt stop     stop anything left running
  dt install  put `dt` on your PATH
"""
import argparse
import datetime
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
DATA = Path(os.environ.get('DT_DATA', Path.home() / 'dt_data'))
VENV = Path(os.environ.get('DT_VENV', DATA / 'venvs' / 'dt'))
VERSION = (ROOT / 'VERSION').read_text().strip()

STREAMS = (('dt/lidar', 'LiDAR'), ('dt/imu', 'IMU'), ('dt/camera', 'Camera'), ('dt/odom', 'Odometry'))
HOLD_STILL_S = 5
READY_TIMEOUT_S = 60
REPLAY_SETTLE_S = 5  # replay has no streams to wait for until the bag plays
STACK_SETTLE_S = 2
LOW_DISK_GB = 20
TTY = sys.stdout.isatty()


def paint(text, code):
    return f'\033[{code}m{text}\033[0m' if TTY else text


def red(text):
    return paint(text, '1;31')


def green(text):
    return paint(text, '32')


def yellow(text):
    return paint(text, '33')


def bold(text):
    return paint(text, '1')


def clock(seconds):
    seconds = int(round(seconds))
    hours, rest = divmod(seconds, 3600)
    minutes, secs = divmod(rest, 60)
    return f'{hours}:{minutes:02d}:{secs:02d}' if hours else f'{minutes:02d}:{secs:02d}'


def size_text(num_bytes):
    for unit, scale in (('GB', 1e9), ('MB', 1e6)):
        if num_bytes >= scale:
            return f'{num_bytes / scale:.1f} {unit}'
    return f'{num_bytes / 1e3:.0f} kB'


def dir_size(path):
    total = 0
    for base, _, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(base, name))
            except OSError:  # the recorder renames files while it runs
                pass
    return total


def sessions(root=None):
    """Every recording, oldest first."""
    base = Path(root or DATA) / 'sessions'
    found = [p.parent for p in base.glob('*/*/bag')]
    return sorted(found, key=lambda p: p.name)


def resolve_session(name, root=None):
    """A path, a folder name (or its end), or the newest recording when name is empty or 'latest'."""
    known = sessions(root)
    if not name or name == 'latest':
        if not known:
            raise SystemExit('No recordings yet. Start one with: dt record')
        return known[-1]
    if Path(name).is_dir():
        return Path(name).resolve()
    matches = [p for p in known if p.name.endswith(name) or name in p.name]
    if len(matches) != 1:
        raise SystemExit(f'{len(matches)} recordings match "{name}". See: dt list')
    return matches[0]


def read_timing(session):
    try:
        return json.loads((Path(session) / 'timing.json').read_text())
    except (OSError, ValueError):
        return {}


def write_timing(session, key, value):
    timing = read_timing(session)
    timing[key] = value
    (Path(session) / 'timing.json').write_text(json.dumps(timing, indent=2) + '\n')


def bag_summary(session):
    """Length and message counts from the bag's metadata; None until the recorder has closed it."""
    try:
        info = yaml.safe_load((Path(session) / 'bag' / 'metadata.yaml').read_text())
        info = info['rosbag2_bagfile_information']
        counts = {t['topic_metadata']['name']: t['message_count'] for t in info['topics_with_message_count']}
        return {'seconds': info['duration']['nanoseconds'] * 1e-9, 'counts': counts}
    except (OSError, yaml.YAMLError, KeyError, TypeError):
        return None


def parse_step_line(line, steps):
    """Feeds one line of process_session.sh into steps [{'name', 'seconds'}]; True when a step started."""
    if line.startswith('== '):
        steps.append({'name': line[3:].strip(), 'seconds': None})
        return True
    done = re.fullmatch(r'\s+(\d+) s\s*', line)
    if done and steps and steps[-1]['seconds'] is None:
        steps[-1]['seconds'] = int(done.group(1))
    return False


def bar(value, largest, width=24):
    return '█' * max(1, round(width * value / largest)) if largest else ''


def rate_cell(label, status):
    if status is None or status['rate'] <= 0:
        return f'{label} {red("no data")}'
    text = f'{label} {status["rate"]:.0f} Hz'
    return green(text) if status['level'] == 0 else yellow(text)


def panel(elapsed, session, bag_bytes, statuses, motion, duration=None, replay=False):
    """The lines shown while recording."""
    limit = f' / {clock(duration)}' if duration else ''
    head = f'{red("● REC")}  {bold(clock(elapsed))}{limit}   {size_text(bag_bytes)}   {session}'
    cells = [rate_cell(label, statuses.get(key)) for key, label in STREAMS if key != 'dt/camera'
             or 'dt/camera' in statuses]
    if motion:
        pace = f'{motion["speed"]:.2f} m/s, {motion["turn"]:.0f} deg/s: {motion["message"]}'
        pace = green(pace) if motion['level'] == 0 else yellow(pace)
    else:
        pace = 'waiting for odometry'
    if replay:
        hint = 'Replaying a recording through the real stack'
    elif elapsed < HOLD_STILL_S:
        hint = yellow(f'HOLD STILL {HOLD_STILL_S - int(elapsed)} s')
    else:
        hint = 'Walk slowly, finish where you started, hold still 5 s, then Ctrl-C'
    return [head, '   '.join(cells), f'Motion  {pace}', hint]


class Telemetry:
    """Reads dt_monitor's /diagnostics in the background."""

    def __init__(self):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._rclpy = rclpy
        self.node = rclpy.create_node('dt_cli')
        self.statuses = {}
        self.motion = None
        self.node.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # context shut down by close()
            pass

    def _on_diagnostics(self, msg):
        for status in msg.status:
            values = {v.key: v.value for v in status.values}
            level = int.from_bytes(status.level, 'little') if isinstance(status.level, bytes) else status.level
            if status.name == 'dt/motion':
                self.motion = {'speed': float(values['speed']), 'turn': float(values['turn_dps']),
                               'message': status.message, 'level': level}
            elif status.name in dict(STREAMS):
                self.statuses[status.name] = {'rate': float(values.get('rate', 0)), 'level': level,
                                              'message': status.message}

    def close(self):
        # The spin thread must end before the node goes, or the process aborts on exit.
        self._executor.shutdown(timeout_sec=2)
        self._thread.join(timeout=2)
        self.node.destroy_node()
        self._rclpy.try_shutdown()


class Screen:
    """Rewrites the same block of lines on a terminal; prints a line now and then otherwise."""

    def __init__(self):
        self.height = 0
        self.last_plain = 0.0

    def draw(self, lines):
        if TTY:
            if self.height:
                sys.stdout.write(f'\033[{self.height}F')
            sys.stdout.write(''.join(f'\033[2K{line}\n' for line in lines))
            sys.stdout.flush()
            self.height = len(lines)
        elif time.monotonic() - self.last_plain >= 10:
            self.last_plain = time.monotonic()
            print(lines[0], flush=True)


def spawn(cmd, log_path, **kwargs):
    log = open(log_path, 'ab')
    # Own process group: Ctrl-C reaches dt only, which then stops the recorder before the stack.
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            start_new_session=True, **kwargs)


def stop_group(proc, timeout):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def tail(path, lines=12):
    try:
        return '\n'.join(Path(path).read_text(errors='replace').splitlines()[-lines:])
    except OSError:
        return ''


def stack_running():
    return subprocess.run(['pgrep', '-f', '[c]apture.launch.py'], capture_output=True).returncode == 0


def stop_stack():
    subprocess.run(['bash', str(ROOT / 'scripts/jetson/stop_stack.sh')], capture_output=True)


def wait_ready(telemetry, proc, want, stop_flag):
    """Waits for every stream to flow. Returns the names still missing."""
    started = time.monotonic()
    screen = Screen()
    while time.monotonic() - started < READY_TIMEOUT_S and not stop_flag.is_set():
        if proc.poll() is not None:
            break
        flowing = [k for k, _ in want if telemetry.statuses.get(k, {}).get('rate', 0) > 0]
        missing = [label for key, label in want if key not in flowing]
        if not missing:
            return []
        screen.draw([f'Starting sensors {clock(time.monotonic() - started)}   waiting for: {", ".join(missing)}'])
        time.sleep(0.5)
    return [label for key, label in want if telemetry.statuses.get(key, {}).get('rate', 0) <= 0]


def report_recording(session, seconds):
    summary = bag_summary(session)
    size = dir_size(session)
    print(f'\n{bold("Recording finished")}')
    print(f'  Time     {bold(clock(seconds))}')
    print(f'  Size     {size_text(size)}')
    print(f'  Saved    {session}')
    record = {'seconds': round(seconds, 1), 'bytes': size}
    if summary:
        counts = summary['counts']
        record['bag_seconds'] = round(summary['seconds'], 1)
        print(f'  LiDAR    {counts.get("/dt/lidar/points", 0)} scans'
              f'   Odometry {counts.get("/Odometry", 0)}'
              f'   Images {counts.get("/zed/zed_node/rgb/color/rect/image/compressed", 0)}')
        for topic in ('/dt/lidar/points', '/dt/lidar/imu', '/Odometry'):
            if not counts.get(topic):
                print(red(f'  WARNING  nothing recorded on {topic}: this recording cannot be processed'))
    else:
        print(red('  WARNING  the recorder did not close the bag: this recording may be damaged'))
    write_timing(session, 'record', record)


def capture(args, replay, live, run, stop_flag, telemetry, logs):
    """Starts the stack and recorder and waits for the end of the walk. Returns an error text or None."""
    launch = ['ros2', 'launch', 'dt_bringup', 'capture.launch.py', f'profile:={args.profile}',
              'rviz_config:=live', f'rviz:={"true" if live else "false"}']
    if replay:
        launch += ['driver:=false', 'camera:=false', 'use_sim_time:=true']
    elif args.no_camera:
        launch += ['camera:=false']

    site = args.site or ('rehearsal' if replay else 'site')
    rec = ['ros2', 'run', 'dt_tools', 'dt_session', 'record', '--site', site, '--profile', args.profile,
           '--note', args.note]
    if replay:
        rec += ['--no-svo', '--rehearsal']
        rig = replay.parent / 'rig_sim.yaml'
        if rig.exists():
            rec += ['--rig', str(rig)]
    if args.no_camera:
        rec += ['--no-camera']
    if args.raw:
        rec += ['--raw']
    env = dict(os.environ, PYTHONUNBUFFERED='1')

    print(f'{bold("Digital twin")} v{VERSION}   profile {args.profile}'
          f'{"   live map on" if live else ""}{"   replay " + replay.name if replay else ""}')
    run['stack'] = stack = spawn(launch, logs['stack'], env=env)
    if replay:
        stop_flag.wait(REPLAY_SETTLE_S)
    else:
        want = [s for s in STREAMS if not (s[0] == 'dt/camera' and args.no_camera)]
        missing = wait_ready(telemetry, stack, want, stop_flag)
        if missing and not stop_flag.is_set():
            return (f'No data from: {", ".join(missing)}\n'
                    'Check that the rig is powered and its cables are in, then run dt doctor.\n'
                    f'Full log: {logs["stack"]}')
    if stop_flag.is_set():
        return None

    run['recorder'] = recorder = spawn(rec, logs['record'], env=env)
    session = None
    while session is None and recorder.poll() is None and not stop_flag.is_set():
        found = re.search(r'recording to (\S+)', Path(logs['record']).read_text(errors='replace'))
        session = Path(found.group(1)) if found else None
        time.sleep(0.2)
    if session is None:
        return None if stop_flag.is_set() else f'The recorder did not start.\n{tail(logs["record"])}'
    run['session'] = session
    player = None
    if replay:
        stop_flag.wait(STACK_SETTLE_S)
        run['player'] = player = spawn(['ros2', 'bag', 'play', str(replay), '--clock', '-r', str(args.rate)],
                                       logs['replay'], env=env)

    started = time.monotonic()
    screen = Screen()
    label = f'{session.parent.name}/{session.name}'
    while not stop_flag.is_set():
        run['seconds'] = time.monotonic() - started
        screen.draw(panel(run['seconds'], label, dir_size(session / 'bag'), telemetry.statuses, telemetry.motion,
                          args.duration, bool(replay)))
        if args.duration and run['seconds'] >= args.duration:
            break
        if player is not None and player.poll() is not None:
            stop_flag.wait(STACK_SETTLE_S + 1)  # let the last scans reach the recorder
            break
        if recorder.poll() is not None:
            return f'The recorder stopped on its own.\n{tail(logs["record"])}'
        if stack.poll() is not None:
            return f'The sensor stack stopped.\n{tail(logs["stack"])}\nFull log: {logs["stack"]}'
        time.sleep(0.5)
    run['seconds'] = time.monotonic() - started
    return None


def record_session(args):
    """Runs one recording. Returns its folder, or None if it failed or nothing was recorded."""
    replay = Path(args.replay).resolve() if args.replay else None
    if stack_running():
        raise SystemExit('The stack is already running. Stop it first with: dt stop')
    if replay and not (replay / 'metadata.yaml').exists():
        raise SystemExit(f'{replay} is not a bag folder')
    free_gb = shutil.disk_usage(DATA).free / 1e9
    if free_gb < LOW_DISK_GB:
        print(yellow(f'Only {free_gb:.0f} GB free; a capture uses about 1 GB a minute.'))
    live = args.live
    if live and not os.environ.get('DISPLAY'):
        print(yellow('No display here, so no live map. Recording continues without it.'))
        live = False

    (DATA / 'logs').mkdir(exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    logs = {name: DATA / 'logs' / f'{stamp}_{name}.log' for name in ('stack', 'record', 'replay')}
    run = {'stack': None, 'recorder': None, 'player': None, 'session': None, 'seconds': 0.0}
    stop_flag = threading.Event()
    sigint = signal.signal(signal.SIGINT, lambda *_: stop_flag.set())
    sigterm = signal.signal(signal.SIGTERM, lambda *_: stop_flag.set())
    telemetry = Telemetry()
    try:
        failed = capture(args, replay, live, run, stop_flag, telemetry, logs)
    finally:
        print('Stopping, please wait…', flush=True)
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # the bag must close cleanly
        stop_group(run['player'], 5)
        stop_group(run['recorder'], 40)
        session = run['session']
        if session is not None:
            report_recording(session, run['seconds'])
            for name, path in logs.items():
                if path.exists():
                    shutil.copy(path, session / f'{name}.log')
        stop_group(run['stack'], 10)
        stop_stack()
        telemetry.close()
        signal.signal(signal.SIGINT, sigint)
        signal.signal(signal.SIGTERM, sigterm)
    if failed:
        print(red(f'\n{failed}'), file=sys.stderr)
        return None
    return session


def cmd_record(args):
    session = record_session(args)
    if session is None:
        return 1
    print(f'  Next     dt process {session.name}')
    return 0


def process_session(session, args):
    """Runs the pipeline on one recording with a live clock. Returns the exit code."""
    cmd = ['bash', str(ROOT / 'scripts/process_session.sh'), str(session)] + args.passthrough
    env = dict(os.environ, PYTHONUNBUFFERED='1')
    summary = bag_summary(session)
    print(f'{bold("Processing")} {session.parent.name}/{session.name}'
          + (f'   recording length {clock(summary["seconds"])}' if summary else ''))
    steps = []
    output = []
    lock = threading.Lock()
    stop_flag = threading.Event()
    previous = signal.signal(signal.SIGINT, lambda *_: stop_flag.set())
    started = time.monotonic()
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, env=env,
                            stdin=subprocess.DEVNULL, start_new_session=True)

    def read():
        for line in proc.stdout:
            with lock:
                output.append(line.rstrip())
                parse_step_line(line, steps)

    reader = threading.Thread(target=read, daemon=True)
    reader.start()
    shown = 0
    step_started = {}
    while proc.poll() is None or reader.is_alive():
        if stop_flag.is_set():
            stop_group(proc, 5)
            break
        with lock:
            snapshot = [dict(s) for s in steps]
        for index, step in enumerate(snapshot[shown:], shown):
            step_started.setdefault(index, time.monotonic())
            if step['seconds'] is None:
                break
            print(f'\r\033[2K  {green("✓")} {step["name"]:<38}{clock(step["seconds"]):>7}'
                  if TTY else f'  ok {step["name"]:<38}{clock(step["seconds"]):>7}', flush=True)
            shown = index + 1
        if TTY and shown < len(snapshot):
            now = time.monotonic()
            sys.stdout.write(f'\r\033[2K  ▸ {bold(snapshot[shown]["name"].ljust(38))}'
                             f'{clock(now - step_started[shown]):>7}   total {clock(now - started)}')
            sys.stdout.flush()
        time.sleep(0.25)
    code = proc.wait()
    reader.join(timeout=2)
    signal.signal(signal.SIGINT, previous)
    total = time.monotonic() - started
    if TTY:
        print('\r\033[2K', end='')
    if stop_flag.is_set():
        print(yellow(f'Stopped after {clock(total)}. Nothing is lost: run dt process again.'))
        return 130
    if code != 0:
        print(red(f'Processing failed after {clock(total)}.'), file=sys.stderr)
        print('\n'.join(output[-12:]), file=sys.stderr)
        print(f'Full log: {session}/products/logs/process.log', file=sys.stderr)
        return code

    finished = [s for s in steps if s['seconds'] is not None]
    write_timing(session, 'process', {'seconds': round(total, 1), 'steps': finished})
    print(f'\n{bold("Twin finished")}')
    largest = max((s['seconds'] for s in finished), default=0)
    for step in finished:
        print(f'  {step["name"]:<38}{clock(step["seconds"]):>7}  {paint(bar(step["seconds"], largest), "36")}')
    ratio = f'  ({total / summary["seconds"]:.1f} × the recording)' if summary and summary['seconds'] else ''
    print(f'  {bold("Total".ljust(38))}{clock(total):>7}{ratio}')
    quality = session / 'products' / 'quality' / 'quality.json'
    if quality.exists():
        sharp = json.loads(quality.read_text()).get('sharpness_mm', {})
        if sharp.get('full_map_median') is not None:
            print(f'  Surface thickness  {sharp["full_map_median"]} mm median   '
                  f'one scan {sharp.get("one_scan_median")} mm')
    print(f'  Saved    {session}/products')
    print(f'  Next     dt view {session.name}')
    return 0


def cmd_process(args):
    return process_session(resolve_session(args.session), args)


def cmd_run(args):
    session = record_session(args)
    if session is None:
        return 1
    print()
    return process_session(session, args)


def view_mode(session):
    products = Path(session) / 'products'
    if (products / 'textured' / 'mesh.obj').exists():
        return ['--textured']
    if (products / 'mesh_rgb.ply').exists() or (products / 'mesh.ply').exists():
        return ['--mesh']
    return []


def cmd_view(args):
    session = resolve_session(args.session)
    if not (session / 'products').is_dir():
        raise SystemExit(f'{session.name} has not been processed yet. Run: dt process {session.name}')
    mode = {'textured': ['--textured'], 'mesh': ['--mesh'], 'points': []}.get(args.mode, view_mode(session))
    env = dict(os.environ, PYTHONPATH=str(ROOT / 'pipeline'))
    print(f'Opening {session.name}. Drag to look around, scroll to move, Q to close.')
    return subprocess.call([str(VENV / 'bin/python'), str(ROOT / 'scripts/view_map.py'), str(session)] + mode, env=env)


def cmd_live(_):
    if not stack_running():
        raise SystemExit('Nothing is recording. Start with: dt record --live')
    config = ROOT / 'ros2_ws/src/dt_bringup/rviz/live.rviz'
    return subprocess.call(['rviz2', '-d', str(config)], stderr=subprocess.DEVNULL)


def cmd_list(_):
    known = sessions()
    if not known:
        print('No recordings yet. Start one with: dt record')
        return 0
    print(f'{"recording":<44}{"length":>8}{"size":>10}{"processing":>12}')
    for session in known:
        timing = read_timing(session)
        summary = bag_summary(session)
        length = clock(summary['seconds']) if summary else '-'
        processing = timing.get('process', {}).get('seconds')
        if processing is not None:
            state = clock(processing)
        else:
            state = 'done' if (session / 'products' / 'map.laz').exists() else 'not yet'
        print(f'{session.parent.name + "/" + session.name:<44}{length:>8}'
              f'{size_text(dir_size(session / "bag")):>10}{state:>12}')
    return 0


def doctor_checks():
    def has_package(name):
        return subprocess.run(['ros2', 'pkg', 'prefix', name], capture_output=True).returncode == 0

    gpu = subprocess.run(['nvidia-smi', '-L'], capture_output=True, text=True) if shutil.which('nvidia-smi') else None
    free_gb = shutil.disk_usage(DATA).free / 1e9
    yield 'ROS 2 Humble', shutil.which('ros2') is not None, 'run scripts/jetson/01_install_ros_humble.sh'
    for name, why in (('dt_bringup', 'capture stack'), ('fast_lio', 'odometry'), ('rslidar_sdk', 'Airy driver'),
                      ('zed_wrapper', 'camera driver')):
        yield f'{name} ({why})', has_package(name), 'run scripts/jetson/07_build_workspace.sh'
    yield 'dtp pipeline', (VENV / 'bin/dtp').exists(), 'run scripts/jetson/08_install_python_tools.sh'
    yield 'GPU', bool(gpu and gpu.returncode == 0) or Path('/dev/nvhost-gpu').exists(), 'processing will be slower'
    yield f'Disk ({free_gb:.0f} GB free)', free_gb >= LOW_DISK_GB, f'keep {LOW_DISK_GB} GB or more free'
    yield 'Display for live map', bool(os.environ.get('DISPLAY')), 'dt record --live needs a screen'


def cmd_doctor(_):
    healthy = True
    print(f'{bold("Digital twin")} v{VERSION}   data in {DATA}')
    for name, ok, advice in doctor_checks():
        healthy &= ok
        print(f'  {green("ok ") if ok else red("no ")} {name}' + ('' if ok else f'   {yellow(advice)}'))
    return 0 if healthy else 1


def cmd_stop(_):
    stop_stack()
    print('Stopped.')
    return 0


def cmd_install(_):
    target = Path.home() / '.local/bin/dt'
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_symlink() or target.exists():
        target.unlink()
    target.symlink_to(ROOT / 'dt')
    print(f'Linked {target}')
    if str(target.parent) not in os.environ.get('PATH', '').split(':'):
        print(yellow(f'Add {target.parent} to your PATH.'))
    return 0


def add_record_options(parser):
    parser.add_argument('--site', default=None, help='folder for this recording (default: site)')
    parser.add_argument('--note', default='', help='a note saved with the recording')
    parser.add_argument('--profile', default='airy', choices=['airy', 'mid360'])
    parser.add_argument('--live', action='store_true', help='show the map growing in RViz')
    parser.add_argument('--duration', type=float, default=None, help='stop after this many seconds')
    parser.add_argument('--no-camera', action='store_true', help='LiDAR only')
    parser.add_argument('--raw', action='store_true', help='also keep the raw driver topics')
    parser.add_argument('--replay', default=None, metavar='BAG',
                        help='no rig: play this driver bag through the real stack')
    parser.add_argument('--rate', type=float, default=1.0, help='replay speed')


def add_process_options(parser):
    group = parser.add_argument_group('processing')
    group.add_argument('--self-zone', default=None, metavar='M', help='drop returns closer than this (default 0.8)')
    group.add_argument('--no-refine', action='store_true', help='skip loop closures')
    group.add_argument('--no-texture', action='store_true', help='skip the image-textured mesh')
    group.add_argument('--no-mesh', action='store_true', help='points only')
    group.add_argument('--polish', action='store_true', help='extra scan alignment pass')
    group.add_argument('--calibrate-rings', action='store_true', help='learn the LiDAR beam calibration here')
    group.add_argument('--ring-calib', default=None, metavar='FILE', help='use this beam calibration')


def passthrough(args):
    out = []
    for flag in ('no_refine', 'no_texture', 'no_mesh', 'polish', 'calibrate_rings'):
        if getattr(args, flag, False):
            out.append('--' + flag.replace('_', '-'))
    if getattr(args, 'self_zone', None):
        out += ['--self-zone', args.self_zone]
    if getattr(args, 'ring_calib', None):
        out += ['--ring-calib', args.ring_calib]
    return out


def main():
    parser = argparse.ArgumentParser(prog='dt', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--version', action='version', version=f'dt {VERSION}')
    sub = parser.add_subparsers(dest='command')

    record = sub.add_parser('record', help='record, with a running clock')
    add_record_options(record)
    record.set_defaults(func=cmd_record)

    process = sub.add_parser('process', help='build the twin, with a clock on every step')
    process.add_argument('session', nargs='?', default='latest')
    add_process_options(process)
    process.set_defaults(func=cmd_process)

    run = sub.add_parser('run', help='record, then process')
    add_record_options(run)
    add_process_options(run)
    run.set_defaults(func=cmd_run)

    view = sub.add_parser('view', help='open the finished 3D model')
    view.add_argument('session', nargs='?', default='latest')
    view.add_argument('--mode', choices=['textured', 'mesh', 'points'], default=None)
    view.set_defaults(func=cmd_view)

    for name, func, text in (('live', cmd_live, 'open the live map on a running stack'),
                             ('list', cmd_list, 'recordings and their times'),
                             ('doctor', cmd_doctor, 'check this machine'),
                             ('stop', cmd_stop, 'stop anything left running'),
                             ('install', cmd_install, 'put dt on your PATH')):
        sub.add_parser(name, help=text).set_defaults(func=func)
    sub.add_parser('help', help='show this help').set_defaults(func=lambda _: parser.print_help() or 0)

    args = parser.parse_args()
    if not args.command:
        parser.print_help()
        return 0
    args.passthrough = passthrough(args)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
