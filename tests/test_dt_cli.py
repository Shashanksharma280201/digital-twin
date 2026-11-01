"""Terminal front end: clocks, step timing, recording lookup. The recording path itself is exercised by
`dt record --replay`, which needs ROS."""
import importlib.util
import json
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('dt_cli', Path(__file__).parents[1] / 'scripts' / 'dt_cli.py')
dt_cli = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dt_cli)


@pytest.mark.parametrize('seconds,text', [(0, '00:00'), (59.6, '01:00'), (291, '04:51'), (3725, '1:02:05')])
def test_clock(seconds, text):
    assert dt_cli.clock(seconds) == text


def test_size_text():
    assert dt_cli.size_text(1_900_000_000) == '1.9 GB'
    assert dt_cli.size_text(42_000_000) == '42.0 MB'
    assert dt_cli.size_text(900) == '1 kB'


def test_steps_follow_the_pipeline_output():
    steps = []
    lines = ['== ingest (bag read once)', 'INFO ingest: {"scans": 3}', '   48 s', '== refine', '   26 s',
             '== fuse', 'INFO fuse']
    started = [dt_cli.parse_step_line(line, steps) for line in lines]
    assert sum(started) == 3
    assert steps == [{'name': 'ingest (bag read once)', 'seconds': 48}, {'name': 'refine', 'seconds': 26},
                     {'name': 'fuse', 'seconds': None}]


def test_step_time_needs_an_open_step():
    steps = []
    dt_cli.parse_step_line('   5 s', steps)
    assert steps == []


def make_session(root, site, name):
    path = root / 'sessions' / site / name
    (path / 'bag').mkdir(parents=True)
    return path


def test_latest_is_the_newest_across_sites(tmp_path):
    make_session(tmp_path, 'zeta', '20261021_142710_airy')
    newest = make_session(tmp_path, 'home', '20261022_001306_airy')
    assert dt_cli.resolve_session(None, tmp_path) == newest
    assert dt_cli.resolve_session('latest', tmp_path) == newest


def test_session_by_part_of_the_name(tmp_path):
    wanted = make_session(tmp_path, 'home', '20261022_001306_airy')
    make_session(tmp_path, 'home', '20261021_222553_airy')
    assert dt_cli.resolve_session('001306', tmp_path) == wanted


def test_session_by_path(tmp_path):
    assert dt_cli.resolve_session(str(tmp_path)) == tmp_path.resolve()


@pytest.mark.parametrize('name', ['2026', 'nothing'])
def test_ambiguous_or_unknown_session_stops(tmp_path, name):
    make_session(tmp_path, 'home', '20261022_001306_airy')
    make_session(tmp_path, 'home', '20261021_222553_airy')
    with pytest.raises(SystemExit):
        dt_cli.resolve_session(name, tmp_path)


def test_no_recordings_stops(tmp_path):
    with pytest.raises(SystemExit):
        dt_cli.resolve_session(None, tmp_path)


def test_timing_keeps_both_halves(tmp_path):
    dt_cli.write_timing(tmp_path, 'record', {'seconds': 360})
    dt_cli.write_timing(tmp_path, 'process', {'seconds': 291})
    saved = json.loads((tmp_path / 'timing.json').read_text())
    assert saved == {'record': {'seconds': 360}, 'process': {'seconds': 291}}


def test_bag_summary_reads_the_metadata(tmp_path):
    (tmp_path / 'bag').mkdir()
    (tmp_path / 'bag' / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n  duration:\n    nanoseconds: 82000000000\n'
        '  topics_with_message_count:\n  - message_count: 820\n    topic_metadata:\n      name: /dt/lidar/points\n')
    assert dt_cli.bag_summary(tmp_path) == {'seconds': 82.0, 'counts': {'/dt/lidar/points': 820}}


def test_unclosed_bag_has_no_summary(tmp_path):
    assert dt_cli.bag_summary(tmp_path) is None


def test_damaged_bag_has_no_summary(tmp_path):
    (tmp_path / 'bag').mkdir()
    (tmp_path / 'bag' / 'metadata.yaml').write_text('x')
    assert dt_cli.bag_summary(tmp_path) is None


def test_panel_tells_the_operator_to_hold_still_then_walk():
    streams = {'dt/lidar': {'rate': 10.0, 'level': 0}, 'dt/imu': {'rate': 200.0, 'level': 0},
               'dt/odom': {'rate': 0.0, 'level': 1}}
    first = dt_cli.panel(2, 'home/x', 0, streams, None)
    later = dt_cli.panel(60, 'home/x', 0, streams, None)
    assert 'HOLD STILL 3' in first[-1]
    assert 'Walk slowly' in later[-1]
    assert '01:00' in later[0]
    assert 'LiDAR 10 Hz' in later[1] and 'Odometry no data' in later[1]
    assert 'Camera' not in later[1]


def test_panel_shows_the_time_limit():
    assert '00:30 / 05:00' in dt_cli.panel(30, 'home/x', 0, {}, None, duration=300)[0]


def test_failed_step_stops_the_pipeline(tmp_path):
    """A step that fails must end the run, or the clock would report a twin that was never built."""
    import subprocess
    script = (Path(__file__).parents[1] / 'scripts' / 'process_session.sh').read_text()
    start = script.index('run() {')
    run = script[start:script.index('\n}\n', start) + 3]
    test = f'set -eo pipefail\nlog={tmp_path}/log\n{run}\nrun bad false\necho reached\n'
    done = subprocess.run(['bash', '-c', test], capture_output=True, text=True)
    assert done.returncode != 0
    assert 'reached' not in done.stdout
