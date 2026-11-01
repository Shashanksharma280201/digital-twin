"""Open the digital twin (reconstructed point cloud) in an interactive 3D viewer.

python sim/view_twin.py ~/dt_data/sim/demo_mid360            # day1 twin, coloured by height
python sim/view_twin.py RUN --color rgb                       # LiDAR + camera: true colours (after `dtp colorize`)
python sim/view_twin.py RUN --variant day2 --color error      # coloured by error vs ground truth (blue 0 -> red 10 cm)
python sim/view_twin.py RUN --with-world                      # overlay the ground-truth site mesh (grey)
python sim/view_twin.py RUN --export twin_day1.ply            # write a PLY for CloudCompare/MeshLab/Blender
python sim/view_twin.py RUN --color rgb --export twin_rgb.npz # world-frame npz for blender/show_result.py --map

Mouse: drag rotate, scroll zoom, shift+drag pan; +/- point size; Q quit.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d


def colormap(v, lo, hi):
    stops = np.array([[0.15, 0.25, 0.9], [0.0, 0.8, 0.9], [0.2, 0.85, 0.2], [0.95, 0.85, 0.1], [0.9, 0.2, 0.1]])
    s = np.clip((v - lo) / max(hi - lo, 1e-6), 0, 1) * (len(stops) - 1)
    i = np.minimum(s.astype(int), len(stops) - 2)
    f = (s - i)[:, None]
    return stops[i] * (1 - f) + stops[i + 1] * f


def load_rgb_twin(run, variant):
    """session_<v>/products/map_rgb.laz (SLAM frame) -> world frame via the dt_sim eval alignment."""
    import laspy

    las = laspy.read(run / f'session_{variant}' / 'products' / 'map_rgb.laz')
    xyz = np.column_stack([las.x, las.y, las.z])
    T = np.array(json.loads((run / f'eval_{variant}.json').read_text())['T_world_estimate_aligned'])
    xyz = xyz @ T[:3, :3].T + T[:3, 3]
    rgb = np.column_stack([las.red, las.green, las.blue]) / 65535.0
    return xyz, rgb


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', type=Path)
    ap.add_argument('--variant', default='day1', choices=['day1', 'day2'])
    ap.add_argument('--color', default='height', choices=['height', 'error', 'rgb'])
    ap.add_argument('--with-world', action='store_true')
    ap.add_argument('--export', type=Path, default=None)
    args = ap.parse_args()

    run = args.run.expanduser()
    if args.color == 'rgb':
        xyz, rgb = load_rgb_twin(run, args.variant)
    else:
        data = np.load(run / 'products' / f'map_eval_{args.variant}.npz')  # world frame (written by dt_sim eval)
        xyz = np.column_stack([data['x'], data['y'], data['z']]).astype(np.float64)
        if args.color == 'error':
            rgb = colormap(data['error'], 0.0, 0.10)
        else:
            rgb = colormap(xyz[:, 2], *np.percentile(xyz[:, 2], [2, 98]))
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    cloud.colors = o3d.utility.Vector3dVector(rgb)
    print(f'{len(xyz):,} points ({args.variant}, coloured by {args.color})')

    if args.export and args.export.suffix == '.npz':
        c8 = np.round(rgb * 255).astype(np.uint8)
        np.savez_compressed(args.export, x=xyz[:, 0].astype(np.float32), y=xyz[:, 1].astype(np.float32),
                            z=xyz[:, 2].astype(np.float32), red=c8[:, 0], green=c8[:, 1], blue=c8[:, 2])
        print(f'wrote {args.export}')
        return
    if args.export:
        o3d.io.write_point_cloud(str(args.export), cloud)
        print(f'wrote {args.export}')
        return

    geoms = [cloud]
    if args.with_world:
        m = np.load(run / 'world' / f'mesh_{args.variant}.npz')
        mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(m['vertices'].astype(np.float64)),
                                         o3d.utility.Vector3iVector(m['triangles']))
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color([0.55, 0.55, 0.6])
        geoms.append(mesh)
    traj = run / 'products' / f'traj_est_world_{args.variant}.tum'
    if traj.exists():
        p = np.loadtxt(traj)[:, 1:4]
        path = o3d.geometry.LineSet(o3d.utility.Vector3dVector(p),
                                    o3d.utility.Vector2iVector(np.column_stack([np.arange(len(p) - 1), np.arange(1, len(p))])))
        path.paint_uniform_color([1.0, 0.45, 0.05])
        geoms.append(path)
    o3d.visualization.draw_geometries(geoms, window_name=f'Digital twin - {run.name} {args.variant}',
                                      width=1600, height=900)


if __name__ == '__main__':
    main()
