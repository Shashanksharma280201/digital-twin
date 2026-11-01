"""dt_monitor: live health and time-sync check for the capture stack.

Prints a table every `report_period` seconds and publishes /diagnostics:
  rate per stream, latency (now - header.stamp), since-boot stamps (PTP not locked),
  the 37 s UTC/TAI offset, LiDAR<->camera stamp offset, and the IMU gravity vector.
Motion coach: warns when the operator turns too fast, walks too fast, or spins on the spot
(the capture habit that limits the twin most); also shown as text above the rig in RViz
(/dt/coach, a visualization_msgs/Marker).
"""
import collections
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Imu, PointCloud2
from visualization_msgs.msg import Marker

GRAVITY = 9.80665
EPOCH_2020 = 1577836800.0
TAI_UTC_OFFSET = 37.0


class Stream:
    def __init__(self, name, min_rate, max_late=0.1):
        self.name = name
        self.min_rate = min_rate
        self.max_late = max_late
        self.arrivals = collections.deque()
        self.last_stamp = None
        self.last_latency = None

    def add(self, stamp_sec, now_sec):
        self.arrivals.append(now_sec)
        self.last_stamp = stamp_sec
        self.last_latency = now_sec - stamp_sec

    def rate(self, now_sec, window):
        while self.arrivals and self.arrivals[0] < now_sec - window:
            self.arrivals.popleft()
        return len(self.arrivals) / window


def classify_latency(latency, max_late=0.1):
    """Returns (level, message) for a latency in seconds."""
    if latency is None:
        return DiagnosticStatus.ERROR, 'no data'
    if abs(abs(latency) - TAI_UTC_OFFSET) < 1.0:
        return DiagnosticStatus.ERROR, (
            f'~37 s offset ({latency:+.2f} s): UTC/TAI mismatch, adjust phc2sys -O')
    if latency > 1e8:
        return DiagnosticStatus.ERROR, 'stamps look like time-since-boot: PTP not locked'
    if latency > max_late:
        return DiagnosticStatus.WARN, f'late by {latency * 1e3:.0f} ms'
    if latency < -0.01:
        return DiagnosticStatus.WARN, f'stamp {-latency * 1e3:.0f} ms in the future (clock offset)'
    return DiagnosticStatus.OK, 'ok'


MAX_TURN_DPS = 30.0
MAX_SPEED = 0.7
SPIN_WINDOW_S = 10.0


def _yaw(q):
    return math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]))


def coach(samples):
    """Motion advice from odometry samples [(t, x, y, z, yaw_rad)], oldest first.

    Returns (level, message, turn_rate_dps, speed_mps).
    """
    if len(samples) < 3:
        return DiagnosticStatus.OK, 'waiting for odometry', 0.0, 0.0
    t_end = samples[-1][0]
    recent = [s for s in samples if s[0] >= t_end - 1.0]
    if len(recent) < 2:
        recent = samples[-2:]
    dt = max(recent[-1][0] - recent[0][0], 1e-3)
    turn = sum(abs(math.remainder(b[4] - a[4], math.tau)) for a, b in zip(recent, recent[1:]))
    turn_rate = math.degrees(turn) / dt
    speed = math.dist(recent[-1][1:4], recent[0][1:4]) / dt
    window = [s for s in samples if s[0] >= t_end - SPIN_WINDOW_S]
    path = sum(math.dist(a[1:4], b[1:4]) for a, b in zip(window, window[1:]))
    spun = math.degrees(sum(abs(math.remainder(b[4] - a[4], math.tau)) for a, b in zip(window, window[1:])))
    if turn_rate > MAX_TURN_DPS:
        return DiagnosticStatus.WARN, f'turning {turn_rate:.0f} deg/s: slow down (< {MAX_TURN_DPS:.0f})', turn_rate, speed
    if speed > MAX_SPEED:
        return DiagnosticStatus.WARN, f'walking {speed:.1f} m/s: slow down (< {MAX_SPEED})', turn_rate, speed
    if window[-1][0] - window[0][0] > 0.8 * SPIN_WINDOW_S and spun > 180 and path < 1.0:
        return (DiagnosticStatus.WARN, f'turned {spun:.0f} deg but moved {path:.1f} m: walk while turning',
                turn_rate, speed)
    return DiagnosticStatus.OK, 'good pace', turn_rate, speed


def check_gravity(mean_acc):
    """Checks a mean at-rest accelerometer vector (m/s^2) in lidar axes."""
    norm = math.sqrt(sum(a * a for a in mean_acc))
    if abs(norm - GRAVITY) > 1.0:
        if abs(norm - 1.0) < 0.2:
            return DiagnosticStatus.ERROR, f'|a|={norm:.2f}: acceleration still in g (acc_scale?)'
        return DiagnosticStatus.WARN, f'|a|={norm:.2f} m/s^2, expected ~9.81 (moving?)'
    if mean_acc[2] < 0.8 * GRAVITY:
        return DiagnosticStatus.WARN, (
            f'gravity along ({mean_acc[0]:.1f}, {mean_acc[1]:.1f}, {mean_acc[2]:.1f}): '
            'expected +z if the LiDAR is upright; check bridge rotation')
    return DiagnosticStatus.OK, 'gravity +z'


class Monitor(Node):
    def __init__(self):
        super().__init__('dt_monitor')
        camera = self.declare_parameter('camera', True).value
        self.window = self.declare_parameter('window', 3.0).value
        period = self.declare_parameter('report_period', 2.0).value
        self.declare_parameter('profile', '')

        self.streams = {
            # A scan is stamped at its first point, so it arrives one sweep (100 ms) later.
            'lidar': Stream('/dt/lidar/points', 8.0, max_late=0.25),
            'imu': Stream('/dt/lidar/imu', 100.0),
            'odom': Stream('/Odometry', 8.0),
        }
        self.create_subscription(PointCloud2, '/dt/lidar/points',
                                 lambda m: self._on('lidar', m.header.stamp), qos_profile_sensor_data)
        self.create_subscription(Imu, '/dt/lidar/imu', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/Odometry', self._on_odom, qos_profile_sensor_data)
        self.motion = collections.deque(maxlen=200)  # ~20 s of 10 Hz odometry
        self.coach_pub = self.create_publisher(Marker, '/dt/coach', 1)
        if camera:
            self.streams['camera'] = Stream('/zed/zed_node/rgb/color/rect/camera_info', 10.0)
            self.create_subscription(CameraInfo, '/zed/zed_node/rgb/color/rect/camera_info',
                                     lambda m: self._on('camera', m.header.stamp), qos_profile_sensor_data)

        self.acc = collections.deque(maxlen=400)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(period, self._report)

    def _now(self):
        # ROS clock: wall time normally, /clock when use_sim_time is set (bag replay).
        return self.get_clock().now().nanoseconds * 1e-9

    def _on(self, key, stamp):
        self.streams[key].add(stamp.sec + stamp.nanosec * 1e-9, self._now())

    def _on_odom(self, msg):
        self._on('odom', msg.header.stamp)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.motion.append((t, p.x, p.y, p.z, _yaw((q.x, q.y, q.z, q.w))))

    def _publish_coach(self, level, message):
        m = Marker()
        m.header.frame_id = 'body'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'dt_coach', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.z = 0.6
        m.pose.orientation.w = 1.0
        m.scale.z = 0.15
        ok = level == DiagnosticStatus.OK
        m.color.r, m.color.g, m.color.b, m.color.a = (0.2, 0.9, 0.4, 0.9) if ok else (1.0, 0.45, 0.1, 1.0)
        m.text = message
        self.coach_pub.publish(m)

    def _on_imu(self, msg):
        self._on('imu', msg.header.stamp)
        a = msg.linear_acceleration
        self.acc.append((a.x, a.y, a.z))

    def _report(self):
        now = self._now()
        statuses = []
        lines = [f'{"stream":<44}{"rate":>8}{"latency":>12}  status']
        for key, s in self.streams.items():
            rate = s.rate(now, self.window)
            level, msg = classify_latency(s.last_latency, s.max_late)
            if key == 'odom' and s.last_latency is None:
                level, msg = DiagnosticStatus.WARN, 'no odometry (SLAM not running?)'
            elif s.last_latency is not None and rate < s.min_rate:
                level = max(level, DiagnosticStatus.WARN)
                msg = f'rate {rate:.1f} Hz < {s.min_rate:.0f} Hz; ' + msg
            lat = '-' if s.last_latency is None else f'{s.last_latency * 1e3:9.1f} ms'
            lines.append(f'{s.name:<44}{rate:8.1f}{lat:>12}  {msg}')
            statuses.append(self._status(f'dt/{key}', level, msg,
                                         rate=f'{rate:.2f}', latency_ms=lat.strip()))

        lidar, cam = self.streams['lidar'], self.streams.get('camera')
        if cam and lidar.last_latency is not None and cam.last_latency is not None:
            offset = cam.last_latency - lidar.last_latency
            lines.append(f'camera - lidar latency difference: {offset * 1e3:+.1f} ms')

        if len(self.acc) > 50:
            mean = [sum(v[i] for v in self.acc) / len(self.acc) for i in range(3)]
            level, msg = check_gravity(mean)
            lines.append(f'IMU mean accel ({mean[0]:+.2f}, {mean[1]:+.2f}, {mean[2]:+.2f}) m/s^2: {msg}')
            statuses.append(self._status('dt/imu_gravity', level, msg))

        if self.motion:
            level, msg, turn_rate, speed = coach(list(self.motion))
            lines.append(f'motion: {turn_rate:.0f} deg/s, {speed:.2f} m/s: {msg}')
            statuses.append(self._status('dt/motion', level, msg, turn_dps=f'{turn_rate:.0f}', speed=f'{speed:.2f}'))
            self._publish_coach(level, msg)

        self.get_logger().info('\n' + '\n'.join(lines))
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = statuses
        self.diag_pub.publish(arr)

    @staticmethod
    def _status(name, level, message, **values):
        st = DiagnosticStatus(name=name, level=level, message=message)
        st.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        return st


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
