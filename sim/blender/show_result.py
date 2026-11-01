"""Show the reconstructed digital twin inside the ground-truth world (design §5, show_result).

sim/blender/bl.sh show_result.py --out RUN --variant day1 \
    [--map RUN/products/map_eval.npz] [--est-traj RUN/products/traj_est_world.tum] \
    [--diff RUN/products/diff] [--render] [--no-clay]

Inputs must already be in the sim world frame (dt_sim eval writes them that way).
--map: .npz (x,y,z[,error][,red,green,blue uint8] arrays or a 'points' (N,3) array) or .ply; camera colours win over error colours.
       Coloured by `error` (green 0 cm -> yellow 5 cm -> red >=10 cm) when present, else by height.
--est-traj: TUM 't x y z qx qy qz qw' (orange); ground truth from trajectory.csv (green).
--diff: dir with added.{npz,ply} (green) and removed.{npz,ply} (red).
Saves RUN/world/result_<variant>.blend; with --render writes RUN/result_<variant>.png and
RUN/result_<variant>_inside.png.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Vector  # noqa: E402

import common as C  # noqa: E402

_PLY_TYPES = {"float": "f4", "float32": "f4", "double": "f8", "float64": "f8", "uchar": "u1", "uint8": "u1",
              "char": "i1", "int8": "i1", "ushort": "u2", "uint16": "u2", "short": "i2", "int16": "i2",
              "uint": "u4", "uint32": "u4", "int": "i4", "int32": "i4"}


def read_ply(path):
    """Minimal PLY vertex reader (ascii / binary_little_endian)."""
    with open(path, "rb") as f:
        header = []
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            header.append(line)
            if line == "end_header":
                break
        fmt = next(h.split()[1] for h in header if h.startswith("format"))
        props, nverts, in_vertex = [], 0, False
        for h in header:
            parts = h.split()
            if parts[:2] == ["element", "vertex"]:
                nverts, in_vertex = int(parts[2]), True
            elif parts and parts[0] == "element":
                in_vertex = False
            elif in_vertex and parts[:1] == ["property"] and parts[1] != "list":
                props.append((parts[2], _PLY_TYPES[parts[1]]))
        if fmt == "ascii":
            data = np.loadtxt(f, max_rows=nverts)
            return {name: data[:, i] for i, (name, _) in enumerate(props)}
        dtype = np.dtype([(n, "<" + t) for n, t in props])
        arr = np.frombuffer(f.read(nverts * dtype.itemsize), dtype=dtype, count=nverts)
        return {n: arr[n].astype(np.float64) for n, _ in props}


def load_points(path):
    path = Path(path)
    if path.suffix == ".ply":
        d = read_ply(path)
    else:
        z = np.load(path)
        d = {k: z[k] for k in z.files}
        if "points" in d:
            p = d.pop("points")
            d.update(x=p[:, 0], y=p[:, 1], z=p[:, 2])
    xyz = np.stack([d["x"], d["y"], d["z"]], 1).astype(np.float32)
    rgb = None
    if all(k in d for k in ("red", "green", "blue")):  # camera colours (sRGB 0-255) -> linear
        rgb = (np.stack([d["red"], d["green"], d["blue"]], 1).astype(np.float32) / 255.0) ** 2.2
    return xyz, d.get("error"), rgb


def ramp(values, stops):
    """Piecewise-linear colour ramp: stops = [(value, (r, g, b)), ...]."""
    vs = np.array([s[0] for s in stops])
    cs = np.array([s[1] for s in stops])
    return np.stack([np.interp(values, vs, cs[:, i]) for i in range(3)], 1)


def points_node_group(radius):
    name = f"DT_Points_{radius:.3f}"
    if name in bpy.data.node_groups:
        return bpy.data.node_groups[name]
    ng = bpy.data.node_groups.new(name, "GeometryNodeTree")
    ng.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    ng.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    gin = ng.nodes.new("NodeGroupInput")
    gout = ng.nodes.new("NodeGroupOutput")
    m2p = ng.nodes.new("GeometryNodeMeshToPoints")
    m2p.inputs["Radius"].default_value = radius
    setm = ng.nodes.new("GeometryNodeSetMaterial")
    setm.inputs["Material"].default_value = C.emission_material("DT_PointColor", (1, 1, 1), 1.0, attribute="col")
    ng.links.new(gin.outputs[0], m2p.inputs["Mesh"])
    ng.links.new(m2p.outputs["Points"], setm.inputs["Geometry"])
    ng.links.new(setm.outputs["Geometry"], gout.inputs[0])
    return ng


def point_object(name, xyz, rgb, radius, col):
    me = bpy.data.meshes.new(name)
    me.vertices.add(len(xyz))
    me.vertices.foreach_set("co", xyz.astype(np.float32).ravel())
    attr = me.color_attributes.new("col", "FLOAT_COLOR", "POINT")
    rgba = np.concatenate([rgb, np.ones((len(rgb), 1))], 1).astype(np.float32)
    attr.data.foreach_set("color", rgba.ravel())
    me.update()
    obj = bpy.data.objects.new(name, me)
    col.objects.link(obj)
    mod = obj.modifiers.new("points", "NODES")
    mod.node_group = points_node_group(radius)
    obj["dt_class"] = "overlay"
    return obj


def path_curve(name, xyz, rgb, col, width=0.06):
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = width
    spline = curve.splines.new("POLY")
    spline.points.add(len(xyz) - 1)
    co = np.concatenate([xyz, np.ones((len(xyz), 1))], 1).astype(np.float32)
    spline.points.foreach_set("co", co.ravel())
    obj = bpy.data.objects.new(name, curve)
    obj.data.materials.append(C.emission_material(f"{name}_mat", rgb, 1.2))
    col.objects.link(obj)
    return obj


def clay(scene):
    mat = bpy.data.materials.new("Clay")
    mat.diffuse_color = (0.6, 0.6, 0.6, 1)
    nt = mat.node_tree or (setattr(mat, "use_nodes", True) or mat.node_tree)
    bsdf = nt.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = (0.55, 0.56, 0.58, 1.0)
    bsdf.inputs["Roughness"].default_value = 0.8
    bsdf.inputs["Alpha"].default_value = 0.35
    if hasattr(mat, "surface_render_method"):
        mat.surface_render_method = "BLENDED"
    for ob in scene.objects:
        if ob.type == "MESH" and ob.get("dt_class") not in (None, "overlay", "device"):
            for slot in ob.material_slots:
                slot.material = mat


def look_at_camera(scene, name, loc, target, lens):
    cam = bpy.data.objects.new(name, bpy.data.cameras.new(name))
    scene.collection.objects.link(cam)
    cam.location = loc
    cam.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    cam.data.lens = lens
    cam.data.clip_end = 500
    return cam


def main():
    args = C.parse_args(lambda p: (
        p.add_argument("--out", required=True),
        p.add_argument("--variant", default="day1", choices=["day1", "day2"]),
        p.add_argument("--map", default=None),
        p.add_argument("--est-traj", default=None),
        p.add_argument("--diff", default=None),
        p.add_argument("--render", action="store_true"),
        p.add_argument("--no-clay", action="store_true"),
        p.add_argument("--max-points", type=int, default=1_500_000),
        p.add_argument("--samples", type=int, default=32),
    ))
    run = C.run_dir(args.out)
    t0 = time.time()
    bpy.ops.wm.open_mainfile(filepath=str(run / "world" / f"world_{args.variant}.blend"))
    scene = bpy.context.scene
    overlay = C.collection("DT_result")
    rig = C.load_rig_json(run) if (run / "rig_sim.json").exists() else None
    if not args.no_clay and (args.map or args.diff):
        clay(scene)

    if args.map:
        xyz, err, cam_rgb = load_points(args.map)
        if len(xyz) > args.max_points:
            idx = np.random.default_rng(0).choice(len(xyz), args.max_points, replace=False)
            xyz = xyz[idx]
            err = None if err is None else err[idx]
            cam_rgb = None if cam_rgb is None else cam_rgb[idx]
        if cam_rgb is not None:
            rgb = cam_rgb
        elif err is not None:
            rgb = ramp(np.abs(err), [(0.0, (0.05, 0.75, 0.2)), (0.05, (0.95, 0.85, 0.1)),
                                     (0.10, (0.9, 0.1, 0.05))])
        else:
            z = xyz[:, 2]
            lo, hi = np.percentile(z, [2, 98])
            rgb = ramp((z - lo) / max(hi - lo, 1e-6), [(0.0, (0.27, 0.0, 0.33)), (0.35, (0.13, 0.57, 0.55)),
                                                       (0.7, (0.37, 0.79, 0.38)), (1.0, (0.99, 0.91, 0.14))])
        point_object("DT_Map", xyz, rgb, 0.025, overlay)
        how = "camera colours" if cam_rgb is not None else ("error" if err is not None else "height")
        print(f"[show_result] map: {len(xyz)} points (coloured by {how})")

    if args.diff:
        for name, rgb in (("added", (0.1, 0.9, 0.2)), ("removed", (0.95, 0.1, 0.1))):
            for ext in (".npz", ".ply"):
                f = Path(args.diff) / f"{name}{ext}"
                if f.exists():
                    xyz = load_points(f)[0]
                    point_object(f"DT_{name}", xyz, np.tile(rgb, (len(xyz), 1)), 0.05, overlay)
                    print(f"[show_result] diff {name}: {len(xyz)} points")
                    break

    if (run / "trajectory.csv").exists():
        ts, gt, q = C.load_trajectory(run)
        if rig:
            gt = np.array([(C.interp_pose(t, ts, gt, q) @ C.pose_to_matrix(rig["T_base_lidar"])).translation
                           for t in ts[::50]])
        else:
            gt = gt[::50]
        path_curve("GT_path", gt, (0.1, 0.9, 0.25), overlay)
        if rig and (run / "device" / f"device_{rig['profile']}.blend").exists():
            with bpy.data.libraries.load(str(run / "device" / f"device_{rig['profile']}.blend")) as (src, dst):
                dst.collections = ["DT_device"]
            scene.collection.children.link(dst.collections[0])
            root = bpy.data.objects["DT1"]
            root.matrix_world = C.interp_pose(ts[-1], ts, *C.load_trajectory(run)[1:])
    if args.est_traj:
        est = np.loadtxt(args.est_traj)[:, 1:4]
        path_curve("Est_path", est, (1.0, 0.45, 0.05), overlay, width=0.045)

    bpy.ops.wm.save_as_mainfile(filepath=str(run / "world" / f"result_{args.variant}.blend"))
    if args.render:
        C.setup_eevee(scene, 1600, 900, args.samples)
        views = [("", (-36.0, -40.0, 30.0), (2.0, 3.0, 2.0), 30),
                 ("_inside", (-16.0, -13.0, 5.5), (0.0, 5.0, 1.0), 20)]
        for suffix, loc, target, lens in views:
            scene.camera = look_at_camera(scene, f"ResultCam{suffix}", loc, target, lens)
            scene.render.filepath = str(run / f"result_{args.variant}{suffix}.png")
            bpy.ops.render.render(write_still=True)
            print(f"[show_result] rendered {scene.render.filepath}")
    print(f"[show_result] done in {time.time() - t0:.1f}s -> {run / 'world' / f'result_{args.variant}.blend'}")


if __name__ == "__main__":
    main()
