"""Interactive 3D viewer for a captured session's digital twin (real or simulated).

python scripts/view_map.py SESSION                 # camera colours if map_rgb.laz exists, else height
python scripts/view_map.py SESSION --color height
python scripts/view_map.py SESSION --mesh           # TSDF surface mesh (products/mesh.ply, `dtp mesh`)
python scripts/view_map.py SESSION --textured       # image-textured mesh (products/textured, `dtp texture`)
python scripts/view_map.py SESSION --export twin.ply   # PLY for CloudCompare / MeshLab / Blender

Shows SESSION/products/map_rgb.laz (or map.laz) plus the scanner path (orange).
Mouse: drag rotate, scroll zoom, shift+drag pan; +/- point size; Q quit.
"""
import argparse
from pathlib import Path

import laspy
import numpy as np
import open3d as o3d


def height_colors(z):
    stops = np.array([[0.15, 0.25, 0.9], [0.0, 0.8, 0.9], [0.2, 0.85, 0.2], [0.95, 0.85, 0.1], [0.9, 0.2, 0.1]])
    lo, hi = np.percentile(z, [2, 98])
    s = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1) * (len(stops) - 1)
    i = np.minimum(s.astype(int), len(stops) - 2)
    f = (s - i)[:, None]
    return stops[i] * (1 - f) + stops[i + 1] * f


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', type=Path)
    ap.add_argument('--color', choices=['auto', 'rgb', 'height'], default='auto')
    ap.add_argument('--export', type=Path, default=None)
    ap.add_argument('--mesh', action='store_true', help='show products/mesh.ply instead of the points')
    ap.add_argument('--textured', action='store_true', help='show products/textured/mesh.obj')
    args = ap.parse_args()

    products = args.session.expanduser() / 'products'
    if args.textured:
        from dt_pipeline.texture import viewer_uvs

        mesh = o3d.io.read_triangle_mesh(str(products / 'textured' / 'mesh.obj'), True)
        mesh.triangle_uvs = o3d.utility.Vector2dVector(viewer_uvs(np.asarray(mesh.triangle_uvs)))
        print(f'{len(mesh.triangles):,} triangles, textured')
        # Start inside the room, looking across it; the ceiling hides everything from outside.
        o3d.visualization.draw_geometries([mesh], window_name=f'Digital twin textured - {args.session.name}',
                                          width=1600, height=900, mesh_show_back_face=True,
                                          lookat=mesh.get_center(), up=[0, 0, 1], front=[-0.9, 0.2, 0.1], zoom=0.12)
        return
    if args.mesh:
        mesh = o3d.io.read_triangle_mesh(str(products / 'mesh.ply'))
        mesh.compute_vertex_normals()
        print(f'{len(mesh.triangles):,} triangles from mesh.ply')
        o3d.visualization.draw_geometries([mesh], window_name=f'Digital twin mesh - {args.session.name}',
                                          width=1600, height=900, mesh_show_back_face=True)
        return
    path = products / 'map_rgb.laz'
    if args.color == 'height' or not path.exists():
        path = products / 'map.laz'
    las = laspy.read(path)
    xyz = np.column_stack([las.x, las.y, las.z])
    use_rgb = args.color != 'height' and hasattr(las, 'red')
    rgb = (np.column_stack([las.red, las.green, las.blue]) / 65535.0) if use_rgb else height_colors(xyz[:, 2])
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    cloud.colors = o3d.utility.Vector3dVector(rgb)
    print(f'{len(xyz):,} points from {path.name} ({"camera colours" if use_rgb else "height colours"})')

    if args.export:
        o3d.io.write_point_cloud(str(args.export), cloud)
        print(f'wrote {args.export}')
        return
    geoms = [cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)]
    traj = products / 'traj_lidar.tum'
    if traj.exists():
        p = np.loadtxt(traj)[:, 1:4]
        lines = np.column_stack([np.arange(len(p) - 1), np.arange(1, len(p))])
        path_set = o3d.geometry.LineSet(o3d.utility.Vector3dVector(p), o3d.utility.Vector2iVector(lines))
        path_set.paint_uniform_color([1.0, 0.45, 0.05])
        geoms.append(path_set)
    o3d.visualization.draw_geometries(geoms, window_name=f'Digital twin - {args.session.name}', width=1600, height=900)


if __name__ == '__main__':
    main()
