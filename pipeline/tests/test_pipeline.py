import json

import numpy as np
from scipy.spatial.transform import Rotation
from synth import ADDED_BOX, K, T0, T_LIDAR_OPTICAL, gt_pose, surface_distance, world_offset

from dt_pipeline import cli, steps
from dt_pipeline.colmap import project, read_images_txt
from dt_pipeline.geometry import Trajectory, inv, read_tum, write_tum
from dt_pipeline.io import Bag, parse_livox_custom, read_laz, typestore
from dt_pipeline.traj import from_glim_dump


def rot_err_deg(A, B):
    return np.degrees(Rotation.from_matrix(A[:3, :3].T @ B[:3, :3]).magnitude())


def test_livox_custom_fast_path():
    ts = typestore()
    t = ts.types
    pts = [t["livox_ros_driver2/msg/CustomPoint"](offset_time=1000 * i, x=i * 0.5, y=-1.0 * i, z=0.25,
                                                   reflectivity=i % 256, tag=0x10 if i % 2 else 0, line=i % 4)
           for i in range(37)]
    msg = t["livox_ros_driver2/msg/CustomMsg"](
        header=t["std_msgs/msg/Header"](stamp=t["builtin_interfaces/msg/Time"](sec=12, nanosec=500),
                                        frame_id="livox_frame"),
        timebase=1_760_000_000_123_456_789, point_num=37, lidar_id=0, rsvd=np.zeros(3, np.uint8), points=pts)
    raw = ts.serialize_cdr(msg, "livox_ros_driver2/msg/CustomMsg")
    stamp, frame, timebase, arr = parse_livox_custom(raw)
    assert frame == "livox_frame" and timebase == msg.timebase and len(arr) == 37
    assert abs(stamp - (12 + 500e-9)) < 1e-12
    np.testing.assert_allclose(arr["x"], [p.x for p in pts])
    np.testing.assert_array_equal(arr["t"], [p.offset_time for p in pts])
    np.testing.assert_array_equal(arr["line"], [p.line for p in pts])


def test_canonical_scans(sessions):
    a, _ = sessions
    with Bag(a.bag) as bag:
        scans = list(bag.scans())
    assert len(scans) == 40
    s = scans[3]
    assert np.all(np.diff(s.time) >= 0) and s.time[0] == 0
    assert abs(s.stamp - (T0 + 0.3)) < 1e-5
    assert s.ring is not None and s.ring.max() == 31


def test_tum_roundtrip(tmp_path):
    stamps = T0 + np.arange(5) * 0.1
    poses = gt_pose(stamps)
    write_tum(tmp_path / "t.txt", stamps, poses)
    s2, p2 = read_tum(tmp_path / "t.txt")
    np.testing.assert_allclose(s2, stamps)
    np.testing.assert_allclose(p2, poses, atol=1e-6)


def test_traj_from_odometry_matches_ground_truth(sessions):
    a, _ = sessions
    tr = Trajectory.from_tum(a.traj_path)
    t = np.linspace(T0 + 0.1, T0 + 3.9, 25)
    est, gt = tr.interpolate(t), gt_pose(t)
    assert np.abs(est[:, :3, 3] - gt[:, :3, 3]).max() < 2e-3
    assert max(rot_err_deg(e, g) for e, g in zip(est, gt)) < 0.1


def test_traj_from_glim_dump(tmp_path):
    stamps = T0 + np.arange(10) * 0.1
    (tmp_path / "dump").mkdir()
    write_tum(tmp_path / "dump" / "traj_lidar.txt", stamps, gt_pose(stamps))
    tr = from_glim_dump(tmp_path / "dump")
    np.testing.assert_allclose(tr.poses, gt_pose(stamps), atol=1e-6)


def test_map_points_lie_on_surfaces(sessions):
    a, _ = sessions
    xyz, attrs = read_laz(a.map_path)
    assert len(xyz) > 10_000 and "hits" in attrs
    err = surface_distance(xyz)
    assert np.percentile(err, 95) < 0.02, np.percentile(err, [50, 95, 99])


def test_deskew_improves_map(sessions, tmp_path):
    a, _ = sessions
    steps.build(a, voxel=0.05, deskew=False)
    raw, _ = read_laz(a.map_path)
    steps.build(a, voxel=0.05, deskew=True)
    desk, _ = read_laz(a.map_path)
    assert np.percentile(surface_distance(desk), 90) < 0.5 * np.percentile(surface_distance(raw), 90)


def test_align_recovers_known_offset(sessions):
    a, b = sessions
    res = steps.align(a, b, coarse=False)
    T_true = inv(world_offset())  # maps session-b frame points into session-a frame
    assert np.linalg.norm(res.T_ref_src[:3, 3] - T_true[:3, 3]) < 0.03
    assert rot_err_deg(res.T_ref_src, T_true) < 0.3
    assert res.fitness > 0.8


def test_align_coarse_fpfh_ransac(sessions):
    a, b = sessions
    res = steps.align(a, b, coarse=True)
    assert np.linalg.norm(res.T_ref_src[:3, 3] - inv(world_offset())[:3, 3]) < 0.03


def test_diff_detects_added_box(sessions):
    a, b = sessions
    steps.align(a, b, coarse=False)
    summary = steps.diff(a, b, voxel=0.05)
    assert (b.products / "diff" / "summary.json").exists()
    assert summary["added_clusters"], summary
    c = np.array(summary["added_clusters"][0]["centroid"])
    box_center = (ADDED_BOX[0] + ADDED_BOX[1]) / 2
    assert np.linalg.norm(c[:2] - box_center[:2]) < 0.5, c
    assert summary["added_clusters"][0]["n_points"] > 100


def test_clean_flags_moving_person(person_session):
    s = person_session
    steps.clean(s, voxel=0.1, method="visibility", every=1)
    static, _ = read_laz(s.static_map_path)
    dynamic, _ = read_laz(s.products / "map_dynamic.laz")
    on_walls = surface_distance(static) < 0.05
    assert on_walls.mean() > 0.97  # person trail removed from the static map
    assert len(dynamic) > 50
    assert np.median(surface_distance(dynamic)) > 0.2  # what was removed is not the room


def test_colmap_export_pose_convention(sessions):
    a, _ = sessions
    out = steps.colmap(a, min_translation=0.0, min_rotation_deg=0.0)
    sparse = out / "sparse" / "0"
    cams = (sparse / "cameras.txt").read_text().splitlines()[1].split()
    assert cams[1] == "PINHOLE" and cams[2:4] == ["160", "120"]
    poses = read_images_txt(sparse / "images.txt")
    assert len(poses) == 7 and len(list((out / "images").glob("*.jpg"))) == 7
    n_pts = sum(1 for ln in (sparse / "points3D.txt").read_text().splitlines() if not ln.startswith("#"))
    assert n_pts > 1000
    for name, T_wc in poses.items():
        stamp = float(name[:-4])
        T_wc_true = gt_pose(stamp)[0] @ T_LIDAR_OPTICAL
        p = (T_wc_true @ [0.3, -0.2, 3.0, 1.0])[:3]  # a point in front of the true camera
        uv, z = project(p[None], T_wc, K)
        expected = K @ np.array([0.3, -0.2, 3.0]) / 3.0
        np.testing.assert_allclose(uv[0], expected[:2], atol=0.5)
        assert z[0] > 0


def test_cli_info(sessions, capsys):
    a, _ = sessions
    assert cli.main(["info", str(a.path)]) == 0
    info = json.loads(capsys.readouterr().out)
    assert info["topics"]["/dt/lidar/points"]["count"] == 40


def test_colorize_respects_occlusion():
    from dt_pipeline.colmap import ColorAccumulator

    K = np.array([[100.0, 0, 64], [0, 100.0, 48], [0, 0, 1]])
    g = np.stack(np.meshgrid(np.linspace(-0.5, 0.5, 30), np.linspace(-0.4, 0.4, 30)), -1).reshape(-1, 2)
    near = np.column_stack([g, np.full(len(g), 2.0)])          # wall 2 m in front of the camera
    far = np.column_stack([g * 2.5, np.full(len(g), 5.0)])     # wall behind it, same pixels
    side = np.array([[2.0, 0.0, 5.0]])                          # far point outside the near wall's shadow
    xyz = np.vstack([near, far, side])
    img = np.zeros((96, 128, 3), np.uint8)
    img[..., 2] = 200                                            # red in BGR
    acc = ColorAccumulator(xyz, K, cell=4)
    acc.add(img, np.eye(4))
    rgb, seen = acc.result()
    assert seen[: len(near)].all()
    assert not seen[len(near): len(near) + len(far)].any()      # hidden behind the near wall
    assert seen[-1] and tuple(rgb[0]) == (200, 0, 0)


def test_refine_correction_interpolates_between_keyframes():
    from dt_pipeline.geometry import make_T
    from dt_pipeline.refine import apply_correction

    stamps = np.linspace(0.0, 10.0, 101)
    poses = np.tile(np.eye(4), (len(stamps), 1, 1))
    poses[:, 0, 3] = stamps * 0.1
    tr = Trajectory(stamps, poses)
    kf_t = np.array([0.0, 10.0])
    kf_odom = poses[[0, -1]]
    # The end keyframe is corrected by 4 deg of yaw and 0.2 m; the start is fixed.
    C_end = make_T(Rotation.from_euler("z", 4, degrees=True).as_matrix(), [0.0, 0.2, 0.0])
    out = apply_correction(tr, kf_t, kf_odom, np.stack([kf_odom[0], C_end @ kf_odom[1]]))
    assert np.allclose(out.poses[0], poses[0])
    assert np.allclose(out.poses[-1], C_end @ poses[-1])
    mid = out.poses[50] @ inv(poses[50])
    assert abs(rot_err_deg(np.eye(4), mid) - 2.0) < 1e-6 and abs(mid[1, 3] - 0.1) < 1e-9


def test_rig_optical_includes_zed_left_lens():
    from dt_pipeline.rig import Rig

    T = Rig().lidar_to_optical
    assert np.allclose(T[:3, 3], [-0.01, 0.06, 0.015])
    assert np.allclose(T[:3, 2], [1.0, 0.0, 0.0])  # optical z looks along lidar x


def test_mixed_pixel_between_front_and_back_is_dropped():
    from dt_pipeline.cache import mixed_pixels

    r = np.array([2.0, 2.0, 2.0, 3.1, 4.0, 4.0, 4.0])  # one return floating between two surfaces
    bad = mixed_pixels(r, np.zeros(7, np.uint16), np.arange(7.0))
    assert bad.tolist() == [False, False, False, True, False, False, False]


def test_gpu_voxel_map_matches_cpu():
    import torch

    from dt_pipeline import gpu
    from dt_pipeline.mapping import VoxelCloud

    pts = np.random.default_rng(1).uniform(-2, 2, (5000, 3))
    inten = np.random.default_rng(2).uniform(0, 100, 5000).astype(np.float32)
    vc = VoxelCloud(0.1)
    vc.add(pts, inten)
    vm = gpu.VoxelMap(0.1, torch.device("cpu"))
    vm.add(torch.as_tensor(pts, dtype=torch.float32), torch.as_tensor(inten))
    a, b = vc.result()[0], vm.result()[0]
    assert len(a) == len(b) and np.allclose(np.sort(a, 0), np.sort(b, 0), atol=1e-5)


def test_gpu_tsdf_plane_is_one_flat_surface():
    import torch

    from dt_pipeline import gpu

    rng = np.random.default_rng(3)
    T = gpu.Tsdf(np.array([-1.0, -1.0, -0.5]), np.array([1.0, 1.0, 1.5]), 0.03, 0.09, torch.device("cpu"))
    for _ in range(5):  # a wall at z = 1.0 seen from the origin, 2 cm noise
        xy = rng.uniform(-0.8, 0.8, (20000, 2))
        p = np.column_stack([xy, 1.0 + rng.normal(0, 0.02, len(xy))])
        x = torch.as_tensor(p, dtype=torch.float32)
        T.integrate(x, torch.zeros_like(x))
    verts, faces = T.mesh(3.0)
    used = verts[np.unique(faces)]
    assert len(used) > 100
    assert abs(np.median(used[:, 2]) - 1.0) < 0.01 and np.percentile(np.abs(used[:, 2] - 1.0), 95) < 0.02  # no ghost layer
