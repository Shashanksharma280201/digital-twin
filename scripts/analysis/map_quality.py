"""Map-quality report for a capture session (read-only; writes only the report files).

python scripts/analysis/map_quality.py SESSION [--out DIR]

Measures what decides how clear the twin is (docs/analysis/2026-10-21-first-real-capture-analysis.md):
  capture      path length, extent of motion, rotation, return-to-start gap
  ranges       share of LiDAR returns by distance (operator/rig = < 1 m)
  sharpness    median thickness of flat surfaces (15 cm patches): one scan vs the full map
  colour       share of map points that received camera colour
Writes DIR/quality.json and DIR/quality.png (default SESSION/products/quality/).
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import laspy
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from dt_pipeline.cache import ScanCache
from dt_pipeline.geometry.transforms import Trajectory
from dt_pipeline.io.bag import Bag


def plane_thickness(xyz, n_core=3000, radius=0.15, seed=0):
    """Std-dev across planar neighbourhoods (walls, ceilings, desks), per sampled core point."""
    tree = cKDTree(xyz)
    core = xyz[np.random.default_rng(seed).choice(len(xyz), min(n_core, len(xyz)), replace=False)]
    out = []
    for c in core:
        idx = tree.query_ball_point(c, radius)
        if len(idx) < 20:
            continue
        q = xyz[idx] - xyz[idx].mean(0)
        ev = np.linalg.eigvalsh(q.T @ q / len(q))
        if (ev[1] - ev[0]) / max(ev[2], 1e-12) > 0.5:
            out.append(np.sqrt(ev[0]))
    return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', type=Path)
    ap.add_argument('--out', type=Path, default=None)
    args = ap.parse_args()
    S = args.session.expanduser()
    out = args.out or S / 'products' / 'quality'
    out.mkdir(parents=True, exist_ok=True)

    tr = Trajectory.from_tum(S / 'products' / 'traj_lidar.tum')
    pos, t = tr.poses[:, :3, 3], tr.stamps
    R = Rotation.from_matrix(tr.poses[:, :3, :3])
    rot_rate = np.degrees((R[:-1].inv() * R[1:]).magnitude() / np.diff(t))
    rep = {'capture': {
        'duration_s': round(float(t[-1] - t[0]), 1),
        'path_m': round(float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()), 2),
        'motion_extent_m': np.ptp(pos, 0).round(2).tolist(),
        'total_rotation_deg': round(float(np.degrees((R[:-1].inv() * R[1:]).magnitude().sum())), 0),
        'rotation_rate_p90_dps': round(float(np.percentile(rot_rate, 90)), 1),
        'return_to_start_m': round(float(np.linalg.norm(pos[-1] - pos[0])), 3),
    }}

    ranges, single = [], None
    if ScanCache.exists(S / 'products' / 'cache'):  # `dtp ingest` output: no bag decoding
        cache = ScanCache(S / 'products' / 'cache')
        ranges = [np.linalg.norm(cache.scan(k)[:, :3], axis=1) for k in range(0, len(cache), 10)]
        single = SimpleNamespace(stamp=float(cache.stamps[5]), xyz=np.asarray(cache.scan(5)[:, :3], float))
    else:
        with Bag(S / 'bag') as bag:
            for k, sc in enumerate(bag.scans('/dt/lidar/points')):
                if k % 10 == 0:
                    ranges.append(np.linalg.norm(sc.xyz, axis=1))
                if k == 5:
                    single = sc
    r = np.concatenate(ranges)
    edges = [0, 0.5, 1.0, 1.5, 3, 5, 10, 20, 1e9]
    hist = np.histogram(r, bins=edges)[0] / len(r) * 100
    rep['ranges_pct'] = {f'{edges[i]}-{edges[i + 1] if edges[i + 1] < 1e9 else "inf"} m': round(float(h), 1)
                         for i, h in enumerate(hist)}

    las = laspy.read(S / 'products' / 'map.laz')
    mp = np.column_stack([las.x, las.y, las.z])
    T0 = tr.interpolate(np.clip(np.array([single.stamp]), t[0], t[-1]))[0]
    one = single.xyz[np.linalg.norm(single.xyz, axis=1) > 0.3] @ T0[:3, :3].T + T0[:3, 3]
    th_one, th_map = plane_thickness(one), plane_thickness(mp)
    rep['sharpness_mm'] = {'one_scan_median': round(float(np.median(th_one) * 1000), 1),
                           'full_map_median': round(float(np.median(th_map) * 1000), 1),
                           'full_map_p90': round(float(np.percentile(th_map, 90) * 1000), 1),
                           'blur_added_by_combining': round(float(np.median(th_map) / np.median(th_one)), 2)}
    rep['map'] = {'points': int(len(mp)), 'extent_m': np.ptp(mp, 0).round(1).tolist()}
    rgb_path = S / 'products' / 'map_rgb.laz'
    if rgb_path.exists():
        c = laspy.read(rgb_path)
        rgb = np.column_stack([c.red, c.green, c.blue]).astype(np.int64)
        grey = (rgb[:, 0] == rgb[:, 1]) & (rgb[:, 1] == rgb[:, 2]) & (np.abs(rgb[:, 0] - 128 * 257) < 300)
        rep['colour'] = {'coloured_pct': round(float(100 * (1 - grey.mean())), 1)}

    (out / 'quality.json').write_text(json.dumps(rep, indent=2))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(20, 6))
        z0 = pos[:, 2].mean() + 0.5
        sl = mp[np.abs(mp[:, 2] - z0) < 0.03]
        ax[0].scatter(sl[:, 0], sl[:, 1], s=0.3, c='k')
        ax[0].plot(pos[:, 0], pos[:, 1], c='orange', lw=1.5)
        ax[0].set_aspect('equal')
        ax[0].set_title('map slice 0.5 m above the path (6 cm thick) + path')
        ax[1].hist(th_one * 1000, bins=60, range=(0, 60), alpha=.6, label=f'one scan ({np.median(th_one) * 1000:.1f} mm)')
        ax[1].hist(th_map * 1000, bins=60, range=(0, 60), alpha=.6, label=f'full map ({np.median(th_map) * 1000:.1f} mm)')
        ax[1].set_title('thickness of flat surfaces (mm)')
        ax[1].legend()
        ax[2].hist(r, bins=120, range=(0, 12))
        ax[2].axvspan(0, 1, color='r', alpha=.15)
        ax[2].set_title('range of LiDAR returns (m); red = operator/rig zone')
        plt.tight_layout()
        plt.savefig(out / 'quality.png', dpi=70)
    except ImportError:
        pass
    print(json.dumps(rep, indent=2))


if __name__ == '__main__':
    main()
