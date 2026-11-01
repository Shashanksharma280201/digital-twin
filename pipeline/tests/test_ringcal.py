import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from dt_pipeline import ringcal
from dt_pipeline.geometry import Trajectory
from dt_pipeline.io import Scan


def room_mesh():
    box = o3d.geometry.TriangleMesh.create_box(8.0, 8.0, 4.0).translate((-4.0, -4.0, -2.0))
    box = box.subdivide_midpoint(3)
    box.compute_vertex_normals()
    return box


def rig_poses(n=12):
    """A walk: the rig moves and tilts, as range and elevation errors only separate under motion."""
    rng = np.random.default_rng(1)
    poses = np.tile(np.eye(4), (n, 1, 1))
    poses[:, :3, :3] = Rotation.from_euler("xyz", rng.uniform(-0.5, 0.5, (n, 3))).as_matrix()
    poses[:, :3, 3] = rng.uniform([-2.5, -2.5, -1.0], [2.5, 2.5, 1.0], (n, 3))
    return poses


def walk(mesh, dr, de, poses):
    """Scans along ``poses`` with the injected ring errors, plus the matching trajectory."""
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    rings = len(dr)
    el = np.radians(np.linspace(-60, 60, rings))
    az = np.radians(np.arange(0, 360, 1.0))
    A, E = np.meshgrid(az, el, indexing="xy")
    ring = np.repeat(np.arange(rings), len(az))
    u = np.column_stack([(np.cos(E) * np.cos(A)).ravel(), (np.cos(E) * np.sin(A)).ravel(), np.sin(E).ravel()])
    _, _, e = ringcal.beam_axes(u)
    stamps = np.arange(len(poses), dtype=float)
    scans = []
    for s, P in zip(stamps, poses):
        rays = np.column_stack([np.tile(P[:3, 3], (len(u), 1)), u @ P[:3, :3].T])
        r = scene.cast_rays(o3d.core.Tensor(rays.astype(np.float32)))["t_hit"].numpy().astype(np.float64)
        ok = np.isfinite(r)
        # Measured beam: its elevation is off by de, its range by dr.
        xyz = u[ok] * (r[ok] - dr[ring[ok]])[:, None] - e[ok] * (r[ok] * de[ring[ok]])[:, None]
        scans.append(Scan(float(s), xyz, np.zeros(len(xyz), np.float32), np.zeros(len(xyz)), ring[ok].astype(np.uint16)))
    return scans, Trajectory(stamps, poses)


def test_apply_shifts_range_and_elevation():
    xyz = np.array([[2.0, 0.0, 0.0], [0.0, 0.0, 3.0]])
    out = ringcal.apply(xyz, np.array([0, 1]), np.array([0.01, 0.0]), np.array([0.002, 0.5]))
    np.testing.assert_allclose(out[0], [2.01, 0.0, 0.004], atol=1e-9)
    np.testing.assert_allclose(out[1], xyz[1], atol=1e-9)  # zenith: elevation undefined, untouched


def test_unknown_ring_is_left_alone():
    xyz = np.array([[2.0, 0.0, 0.0]])
    out = ringcal.apply(xyz, np.array([5]), np.array([0.01]), np.array([0.0]))
    np.testing.assert_allclose(out, xyz)


def test_recovers_injected_ring_errors():
    rng = np.random.default_rng(0)
    rings = 24
    dr = rng.normal(0, 0.006, rings)
    dr -= dr.mean()
    de = rng.normal(0, 0.001, rings)
    de -= de.mean()
    mesh = room_mesh()
    scans, tr = walk(mesh, dr, de, rig_poses())
    out = ringcal.estimate(scans, tr, mesh, rings=rings, step=1)
    assert np.abs(np.array(out["range_m"]) - dr).max() < 5e-4
    assert np.abs(np.array(out["elev_rad"]) - de).max() < 1e-4


def test_shared_offset_is_not_learned():
    rings = 8
    mesh = room_mesh()
    scans, tr = walk(mesh, np.full(rings, 0.01), np.zeros(rings), rig_poses())
    out = ringcal.estimate(scans, tr, mesh, rings=rings, step=1)
    assert min(out["points"]) >= 2000  # the rings were fitted, not skipped
    assert np.abs(out["range_m"]).max() < 2e-4
