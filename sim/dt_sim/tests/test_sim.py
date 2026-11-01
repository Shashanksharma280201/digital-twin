import numpy as np
import pytest
from dt_pipeline.io.bag import Bag, parse_livox_custom, typestore
from rosbags.rosbag2 import StoragePlugin, Writer
from scipy.spatial.transform import Rotation

from conftest import make_run
from dt_sim.bag import BagOptions, write_bag
from dt_sim.eval import evaluate, export_gt_map
from dt_sim.imu import ImuNoise, simulate_imu
from dt_sim.lidar import LidarNoise
from dt_sim.lidar_airy import Airy
from dt_sim.lidar_mid360 import Mid360, custom_msg_cdr, pack_points
from dt_sim.run import AIRY_IMU_R, EPOCH0
from dt_sim.trajectory import Trajectory
from dt_sim.world import World


def lidar_traj(run):
    return Trajectory.from_csv(run.trajectory_path, run.load_rig().rig.T_base_lidar)


def to_world(traj, points, times):
    T = traj.matrices(times)
    return np.einsum("nij,nj->ni", T[:, :3, :3], points) + T[:, :3, 3]


def test_custom_msg_cdr_roundtrip(mid360_run):
    world = World.load(mid360_run.mesh())
    sensor = Mid360(world, lidar_traj(mid360_run), LidarNoise.ideal())
    idx, ret, offs = sensor.frame(3, 2.0)
    pts = pack_points(idx, ret, offs)
    stamp = int(EPOCH0 * 1e9) + 2_000_000_000
    raw = custom_msg_cdr(stamp, stamp, pts)

    ts = typestore()
    msg = ts.deserialize_cdr(raw, "livox_ros_driver2/msg/CustomMsg")
    assert msg.point_num == len(pts) == 20000
    assert msg.timebase == stamp and msg.header.frame_id == "livox_frame"
    p = msg.points[12345]
    assert p.offset_time == pts["offset_time"][12345]
    assert p.x == pytest.approx(float(pts["x"][12345]))
    assert p.line == pts["line"][12345] and p.tag == 0x10
    assert ts.serialize_cdr(msg, "livox_ros_driver2/msg/CustomMsg") == raw

    _, frame, timebase, fast = parse_livox_custom(raw)
    assert frame == "livox_frame" and timebase == stamp
    assert np.array_equal(fast["x"], pts["x"])


def test_mid360_points_on_surfaces(mid360_run):
    world = World.load(mid360_run.mesh())
    traj = lidar_traj(mid360_run)
    ideal = Mid360(world, traj, LidarNoise.ideal())
    idx, ret, offs = ideal.frame(40, 4.0)  # while walking
    xyz = to_world(traj, ret.points[ret.valid], 4.0 + offs[ret.valid] * 1e-9)
    assert ret.valid.mean() > 0.95
    assert np.percentile(world.distance(xyz), 99) < 1e-3

    r_ideal = np.linalg.norm(ret.points, axis=1)
    noisy = Mid360(world, traj, LidarNoise(), seed=1)
    _, ret_n, _ = noisy.frame(40, 4.0)
    both = ret.valid & ret_n.valid
    dr = np.linalg.norm(ret_n.points[both], axis=1) - r_ideal[both]
    assert np.std(dr) == pytest.approx(0.02, rel=0.1)  # range sigma
    assert 0.8 < ret_n.valid.mean() < 1.0


def test_motion_distortion(tmp_path):
    run = make_run(tmp_path / "spin", "mid360", spin=3.0, duration=4.0)
    world = World.load(run.mesh())
    traj = lidar_traj(run)
    idx, ret, offs = Mid360(world, traj, LidarNoise.ideal()).frame(20, 2.0)
    v, p = ret.valid, ret.points[ret.valid]
    deskewed = to_world(traj, p, 2.0 + offs[v] * 1e-9)
    naive = to_world(traj, p, np.full(v.sum(), 2.0))
    assert np.percentile(world.distance(deskewed), 99) < 1e-3
    assert np.percentile(world.distance(naive), 90) > 0.05


def test_bag_mid360(mid360_run, tmp_path):
    info = write_bag(mid360_run, tmp_path / "bag", BagOptions(profile="mid360", duration=3.0))
    assert info["messages"]["points"] == 30
    with Bag(tmp_path / "bag") as bag:
        topics = bag.info()["topics"]
        assert topics["/livox/lidar"]["type"] == "livox_ros_driver2/msg/CustomMsg"
        assert topics["/livox/imu"]["count"] == pytest.approx(600, abs=2)
        assert topics["/sim/ground_truth/odometry"]["count"] == pytest.approx(300, abs=2)
        assert topics["/zed/zed_node/rgb/color/rect/image/compressed"]["count"] == 1  # t=2.5 only
        info_msg = bag.camera_info()
        assert info_msg.K[0, 0] == 40.0 and info_msg.width == 64
        scans = list(bag.scans("/livox/lidar"))
        assert len(scans) == 30
        assert scans[0].stamp == pytest.approx(EPOCH0, abs=1e-6)


def test_bag_airy_fields(airy_run, tmp_path):
    write_bag(airy_run, tmp_path / "bag", BagOptions(profile="airy", duration=1.0, camera=False))
    with Bag(tmp_path / "bag") as bag:
        _, _, msg = next(bag.messages(["/rslidar_points"]))
        assert [(f.name, f.offset, f.datatype) for f in msg.fields] == [
            ("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("ring", 16, 4), ("timestamp", 18, 8)]
        assert msg.point_step == 26 and msg.header.frame_id == "rslidar" and msg.width == 86400
        dt = np.dtype({"names": ["x", "ring", "timestamp"], "formats": ["<f4", "<u2", "<f8"],
                       "offsets": [0, 16, 18], "itemsize": 26})
        arr = np.frombuffer(bytes(msg.data), dt)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        assert arr["ring"].min() == 0 and arr["ring"].max() == 95
        assert arr["timestamp"][0] == pytest.approx(stamp, abs=1e-6)
        assert np.all(np.diff(arr["timestamp"]) >= 0)
        assert arr["timestamp"][-1] - stamp < 0.1
        assert np.isnan(arr["x"]).any()  # dense_points: false keeps misses as NaN
        scans = list(bag.scans("/rslidar_points"))
        assert len(scans) == 10


def test_airy_points_on_surfaces(airy_run):
    world = World.load(airy_run.mesh())
    traj = lidar_traj(airy_run)
    ret, times = Airy(world, traj, LidarNoise.ideal()).frame(3.0)
    xyz = to_world(traj, ret.points[ret.valid], times[ret.valid])
    assert np.percentile(world.distance(xyz), 99) < 1e-3


@pytest.mark.parametrize("profile,expected", [("mid360", (0, 0, 1)), ("airy", (0, 0, -1))])
def test_imu_at_rest(tmp_path, profile, expected):
    run = make_run(tmp_path / profile, profile, duration=4.0)
    sim = run.load_rig()
    traj = Trajectory.from_csv(run.trajectory_path, sim.T_base_imu)
    R_raw = AIRY_IMU_R if profile == "airy" else None
    s = simulate_imu(traj, 200.0, R_raw, ImuNoise.ideal(), t_start=0.2, t_end=1.8)
    assert np.allclose(s.acc_g.mean(0), expected, atol=1e-3)
    assert np.allclose(s.gyro, 0, atol=1e-4)

    noisy = simulate_imu(traj, 200.0, R_raw, ImuNoise(), seed=3, t_start=0.2, t_end=1.8)
    assert np.allclose(noisy.acc_g.mean(0), expected, atol=0.01)


def test_imu_turn_rate(mid360_run):
    sim = mid360_run.load_rig()
    traj = Trajectory.from_csv(mid360_run.trajectory_path, sim.T_base_imu)
    s = simulate_imu(traj, 200.0, None, ImuNoise.ideal(), t_start=5.0, t_end=7.0)
    yaw = getattr(np, "trapezoid", getattr(np, "trapz", None))(s.gyro[:, 2], s.t)  # numpy 1 and 2
    assert yaw == pytest.approx(np.radians(90), abs=np.radians(1.0))


def _write_odometry_session(run, root):
    """A fake dt_session whose /Odometry is the perfect FAST-LIO output."""
    sim = run.load_rig()
    traj = Trajectory.from_csv(run.trajectory_path, sim.T_base_imu)
    times = np.arange(2.0, 6.9, 0.1)
    T = traj.matrices(times)
    T_ci = np.linalg.inv(T[0]) @ T  # camera_init = first body pose
    ts = typestore()
    t = ts.types
    with Writer(root / "bag", version=8, storage_plugin=StoragePlugin.MCAP) as w:
        conn = w.add_connection("/Odometry", "nav_msgs/msg/Odometry", typestore=ts)
        for ti, Ti in zip(times, T_ci):
            stamp = int(round((EPOCH0 + ti) * 1e9))
            q = Rotation.from_matrix(Ti[:3, :3]).as_quat()
            msg = t["nav_msgs/msg/Odometry"](
                header=t["std_msgs/msg/Header"](
                    stamp=t["builtin_interfaces/msg/Time"](sec=stamp // 10**9, nanosec=stamp % 10**9),
                    frame_id="camera_init"),
                child_frame_id="body",
                pose=t["geometry_msgs/msg/PoseWithCovariance"](
                    pose=t["geometry_msgs/msg/Pose"](
                        position=t["geometry_msgs/msg/Point"](x=Ti[0, 3], y=Ti[1, 3], z=Ti[2, 3]),
                        orientation=t["geometry_msgs/msg/Quaternion"](x=q[0], y=q[1], z=q[2], w=q[3])),
                    covariance=np.zeros(36)),
                twist=t["geometry_msgs/msg/TwistWithCovariance"](
                    twist=t["geometry_msgs/msg/Twist"](
                        linear=t["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0),
                        angular=t["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0)),
                    covariance=np.zeros(36)))
            w.write(conn, stamp, ts.serialize_cdr(msg, "nav_msgs/msg/Odometry"))


def test_eval_perfect_estimate(mid360_run, tmp_path):
    session = tmp_path / "session"
    session.mkdir()
    _write_odometry_session(mid360_run, session)
    result = evaluate(mid360_run, session)
    tr = result["trajectory"]
    assert tr["anchored"]["rmse_m"] < 1e-4 and tr["aligned"]["rmse_m"] < 1e-4
    assert tr["anchored"]["rot_max_deg"] < 0.01
    assert 3.0 < tr["path_length_m"] < 3.6  # 3 m walk + bob + lever arm swing in the turn
    assert (mid360_run.root / "eval.json").exists()


def test_eval_map_in_world_frame(mid360_run, tmp_path):
    gt_map = export_gt_map(mid360_run, "mid360", tmp_path / "gt.laz", frame_step=1.0)
    from dt_pipeline.geometry.transforms import write_tum
    sim = mid360_run.load_rig()
    traj = Trajectory.from_csv(mid360_run.trajectory_path, sim.rig.T_base_lidar)
    times = np.arange(2.0, 6.9, 0.1)
    write_tum(tmp_path / "traj.tum", EPOCH0 + times, traj.matrices(times))
    result = evaluate(mid360_run, None, map_path=gt_map, traj=tmp_path / "traj.tum")
    m = result["map"]
    assert m["mean_m"] < 0.005 and m["within_2cm_pct"] > 99
    assert m["completeness_pct"] > 50


def box_world():
    import open3d as o3d

    from dt_sim.world import World

    box = o3d.geometry.TriangleMesh.create_box(8.0, 6.0, 3.0).subdivide_midpoint(2)
    return World(np.asarray(box.vertices), np.asarray(box.triangles))


def test_surface_metrics_finds_scale_and_ignores_pose():
    from scipy.spatial.transform import Rotation

    from dt_sim.eval import surface_metrics

    world = box_world()
    rng = np.random.default_rng(0)
    truth = world.sample_surface(100_000, rng)
    R = Rotation.from_euler("xyz", [0.01, -0.015, 0.02]).as_matrix()
    m = surface_metrics(world, 1.002 * truth @ R.T + np.array([0.02, -0.01, 0.015]))
    assert abs(m["scale_error_ppm"] - 1e6 * (1 / 1.002 - 1)) < 100
    # 2000 ppm over the box: the tape reads about 2 mm per metre, on the true scale.
    assert 3 < m["tape_median_mm"] < 12


def test_surface_metrics_true_map_scores_zero():
    from dt_sim.eval import surface_metrics

    world = box_world()
    m = surface_metrics(world, world.sample_surface(100_000, np.random.default_rng(0)))
    assert m["mean_m"] < 1e-4 and abs(m["scale_error_ppm"]) < 20 and m["tape_median_mm"] < 0.5
