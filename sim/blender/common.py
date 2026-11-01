"""Shared helpers for the Blender-side sim scripts (Blender 5.2, Python 3.13).

Run scripts as ``blender -b --factory-startup -P sim/blender/<script>.py -- <args>``.
Every script imports this module by adding its own directory to ``sys.path``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

HERE = Path(__file__).resolve().parent
ASSETS = HERE.parent / "assets"

# Semantic classes written to mesh_<variant>.npz (face_class). Order is the id.
CLASSES = [
    "ground", "concrete_structure", "rebar", "formwork", "scaffold",
    "masonry", "equipment", "props", "fence", "device",
]
CLASS_ID = {name: i for i, name in enumerate(CLASSES)}

# Site layout shared by gen_world.py and walk.py (metres, world frame, Z up).
SITE = {
    "extent_x": (-30.0, 30.0),
    "extent_y": (-20.0, 20.0),
    "col_x": [-12.0, -6.0, 0.0, 6.0, 12.0],
    "col_y": [-4.0, 2.0, 8.0, 14.0],
    "storey": 3.2,
    "levels": 3,
    "pit": (16.0, 26.0, -16.0, -6.0, 2.5),   # x0, x1, y0, y1, depth
    "office": (-25.0, 14.0),
}


# --------------------------------------------------------------------------- CLI

def parse_args(build_parser) -> argparse.Namespace:
    """Parses arguments after Blender's ``--`` separator."""
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    build_parser(parser)
    return parser.parse_args(argv)


def run_dir(path: str) -> Path:
    p = Path(os.path.expanduser(path)).resolve()
    p.mkdir(parents=True, exist_ok=True)
    return p


def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    return scene


# --------------------------------------------------------------------- materials

_MAT_SPECS = {
    # name: (base rgb, roughness, metallic, reflectivity 0..1, noise scale, noise strength)
    "concrete":      ((0.55, 0.54, 0.51), 0.85, 0.0, 0.45, 6.0, 0.25),
    "concrete_fresh": ((0.42, 0.42, 0.41), 0.75, 0.0, 0.40, 9.0, 0.15),
    "plywood":       ((0.62, 0.42, 0.22), 0.70, 0.0, 0.50, 20.0, 0.30),
    "formwork_red":  ((0.45, 0.16, 0.10), 0.60, 0.0, 0.45, 15.0, 0.20),
    "steel":         ((0.55, 0.56, 0.58), 0.40, 1.0, 0.35, 30.0, 0.10),
    "galvanised":    ((0.70, 0.71, 0.72), 0.35, 1.0, 0.40, 25.0, 0.10),
    "rebar":         ((0.32, 0.17, 0.10), 0.80, 0.6, 0.25, 40.0, 0.35),
    "gravel":        ((0.45, 0.43, 0.40), 0.95, 0.0, 0.30, 60.0, 0.45),
    "soil":          ((0.30, 0.22, 0.15), 0.95, 0.0, 0.20, 8.0, 0.35),
    "asphalt":       ((0.10, 0.10, 0.10), 0.90, 0.0, 0.12, 50.0, 0.20),
    "blocks":        ((0.62, 0.61, 0.58), 0.90, 0.0, 0.42, 4.0, 0.15),
    "safety_orange": ((0.95, 0.32, 0.03), 0.50, 0.0, 0.70, 5.0, 0.05),
    "safety_yellow": ((0.95, 0.70, 0.05), 0.45, 0.0, 0.65, 5.0, 0.05),
    "white_plastic": ((0.85, 0.85, 0.82), 0.40, 0.0, 0.70, 5.0, 0.05),
    "container_blue": ((0.08, 0.22, 0.45), 0.55, 0.3, 0.35, 5.0, 0.10),
    "glass":         ((0.01, 0.012, 0.015), 0.05, 0.0, 0.05, 1.0, 0.00),
    "anodised_dark": ((0.035, 0.037, 0.04), 0.35, 0.8, 0.10, 80.0, 0.05),
    "anodised_accent": ((0.95, 0.42, 0.05), 0.35, 0.6, 0.30, 80.0, 0.03),
    "rubber":        ((0.02, 0.02, 0.02), 0.90, 0.0, 0.05, 60.0, 0.10),
    "sensor_white":  ((0.80, 0.80, 0.78), 0.35, 0.0, 0.60, 30.0, 0.03),
    "screen":        ((0.01, 0.02, 0.03), 0.10, 0.0, 0.05, 1.0, 0.00),
    "fence_mesh":    ((0.55, 0.57, 0.55), 0.50, 0.9, 0.30, 10.0, 0.10),
    "tarp_green":    ((0.10, 0.30, 0.15), 0.80, 0.0, 0.25, 10.0, 0.10),
}

# Optional CC0 image textures from fetch_assets.py (Poly Haven ids).
_ASSET_TEXTURES = {
    "concrete": "concrete_floor", "plywood": "plywood", "rebar": "rusty_metal_02",
    "gravel": "gravel", "soil": "raked_dirt", "asphalt": "asphalt_02", "blocks": "concrete_block_wall",
}


def material(name: str) -> bpy.types.Material:
    """Procedural Principled material by name (cached). Sets the ``reflectivity`` prop."""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    rgb, rough, metal, refl, nscale, nstrength = _MAT_SPECS[name]
    mat = bpy.data.materials.new(name)
    mat["reflectivity"] = float(refl)
    mat.diffuse_color = (*rgb, 1.0)
    nt = mat.node_tree
    if nt is None:  # Blender < 5 needed use_nodes; 5.x always has a tree
        mat.use_nodes = True
        nt = mat.node_tree
    bsdf = nt.nodes.get("Principled BSDF") or nt.nodes.new("ShaderNodeBsdfPrincipled")
    out = nt.nodes.get("Material Output") or nt.nodes.new("ShaderNodeOutputMaterial")
    nt.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    if name == "glass" or name == "screen":
        bsdf.inputs["Specular IOR Level"].default_value = 0.25
        bsdf.inputs["Coat Weight"].default_value = 0.6
    if name == "screen":
        bsdf.inputs["Emission Color"].default_value = (0.15, 0.45, 0.9, 1.0)
        bsdf.inputs["Emission Strength"].default_value = 0.6

    texcoord = nt.nodes.new("ShaderNodeTexCoord")
    img_path = ASSETS / f"{_ASSET_TEXTURES.get(name, '_none')}_diff_1k.jpg"
    if img_path.exists():
        mapping = nt.nodes.new("ShaderNodeMapping")
        mapping.inputs["Scale"].default_value = (0.5, 0.5, 0.5)
        tex = nt.nodes.new("ShaderNodeTexImage")
        tex.image = bpy.data.images.load(str(img_path), check_existing=True)
        tex.projection = "BOX"
        tex.projection_blend = 0.2
        nt.links.new(texcoord.outputs["Object"], mapping.inputs["Vector"])
        nt.links.new(mapping.outputs["Vector"], tex.inputs["Vector"])
        nt.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    elif nstrength > 0:
        noise = nt.nodes.new("ShaderNodeTexNoise")
        noise.inputs["Scale"].default_value = nscale
        noise.inputs["Detail"].default_value = 6.0
        nt.links.new(texcoord.outputs["Object"], noise.inputs["Vector"])
        ramp = nt.nodes.new("ShaderNodeValToRGB")
        lo = [max(0.0, c * (1 - nstrength)) for c in rgb]
        hi = [min(1.0, c * (1 + nstrength * 0.6)) for c in rgb]
        ramp.color_ramp.elements[0].color = (*lo, 1.0)
        ramp.color_ramp.elements[1].color = (*hi, 1.0)
        nt.links.new(noise.outputs["Fac"], ramp.inputs["Fac"])
        nt.links.new(ramp.outputs["Color"], bsdf.inputs["Base Color"])
    else:
        bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    return mat


def emission_material(name: str, rgb, strength=1.0, attribute: str | None = None):
    """Unlit-looking material for overlays; optionally colored by a point attribute."""
    if name in bpy.data.materials:
        return bpy.data.materials[name]
    mat = bpy.data.materials.new(name)
    mat["reflectivity"] = 0.0
    nt = mat.node_tree
    if nt is None:
        mat.use_nodes = True
        nt = mat.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    emit = nt.nodes.new("ShaderNodeEmission")
    emit.inputs["Strength"].default_value = strength
    if attribute:
        attr = nt.nodes.new("ShaderNodeAttribute")
        attr.attribute_name = attribute
        nt.links.new(attr.outputs["Color"], emit.inputs["Color"])
    else:
        emit.inputs["Color"].default_value = (*rgb, 1.0)
    nt.links.new(emit.outputs["Emission"], out.inputs["Surface"])
    return mat


# ------------------------------------------------------------------- geometry

def collection(name: str, parent=None) -> bpy.types.Collection:
    col = bpy.data.collections.get(name)
    if col is None:
        col = bpy.data.collections.new(name)
        (parent or bpy.context.scene.collection).children.link(col)
    return col


def tag(obj, dt_class: str, mat_name: str | None = None, col=None):
    obj["dt_class"] = dt_class
    if mat_name:
        obj.data.materials.clear()
        obj.data.materials.append(material(mat_name))
    if col is not None:
        for c in list(obj.users_collection):
            c.objects.unlink(obj)
        col.objects.link(obj)
    return obj


def mesh_object(name, verts, faces, col=None):
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in verts], [], [tuple(f) for f in faces])
    me.update()
    obj = bpy.data.objects.new(name, me)
    (col or bpy.context.scene.collection).objects.link(obj)
    return obj


def box(name, size, loc, rot=(0.0, 0.0, 0.0), col=None, dt_class="props", mat=None, bevel=0.0):
    """Axis-aligned box of ``size`` (sx, sy, sz) centred at ``loc``, rotated by Euler XYZ."""
    sx, sy, sz = (s / 2 for s in size)
    v = [(-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz),
         (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    obj = mesh_object(name, v, f, col)
    obj.location = loc
    obj.rotation_euler = rot
    if bevel > 0:
        mod = obj.modifiers.new("bevel", "BEVEL")
        mod.width = bevel
        mod.segments = 3
        mod.limit_method = "NONE"
    return tag(obj, dt_class, mat)


def cylinder(name, radius, depth, loc, rot=(0.0, 0.0, 0.0), verts=12, col=None,
             dt_class="props", mat=None, cap=True):
    ring = [(radius * math.cos(2 * math.pi * i / verts), radius * math.sin(2 * math.pi * i / verts))
            for i in range(verts)]
    v = [(x, y, -depth / 2) for x, y in ring] + [(x, y, depth / 2) for x, y in ring]
    f = [(i, (i + 1) % verts, verts + (i + 1) % verts, verts + i) for i in range(verts)]
    if cap:
        f += [tuple(reversed(range(verts))), tuple(range(verts, 2 * verts))]
    obj = mesh_object(name, v, f, col)
    obj.location = loc
    obj.rotation_euler = rot
    return tag(obj, dt_class, mat)


def tube_between(name, p0, p1, radius, verts=8, col=None, dt_class="scaffold", mat="galvanised"):
    """Cylinder from p0 to p1."""
    p0, p1 = Vector(p0), Vector(p1)
    d = p1 - p0
    obj = cylinder(name, radius, d.length, (p0 + p1) / 2, verts=verts, col=col,
                   dt_class=dt_class, mat=mat, cap=False)
    obj.rotation_mode = "QUATERNION"
    obj.rotation_quaternion = Vector((0, 0, 1)).rotation_difference(d.normalized())
    return obj


def join(objs, name):
    """Joins objects into one (keeps the first's dt_class); returns the result."""
    objs = [o for o in objs if o is not None]
    if not objs:
        return None
    if len(objs) == 1:
        objs[0].name = name
        return objs[0]
    ctx = {"active_object": objs[0], "selected_editable_objects": objs, "object": objs[0]}
    for o in objs:
        for m in list(o.modifiers):
            with bpy.context.temp_override(object=o, active_object=o):
                bpy.ops.object.modifier_apply(modifier=m.name)
    with bpy.context.temp_override(**ctx):
        bpy.ops.object.join()
    objs[0].name = name
    return objs[0]


# ---------------------------------------------------------------------- lighting

def setup_sky(scene, sun_elevation=38.0, sun_rotation=140.0, strength=1.0):
    world = bpy.data.worlds.new("SiteSky")
    scene.world = world
    nt = world.node_tree
    if nt is None:
        world.use_nodes = True
        nt = world.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    sky = nt.nodes.new("ShaderNodeTexSky")
    try:
        sky.sky_type = "MULTIPLE_SCATTERING"
    except TypeError:
        sky.sky_type = "HOSEK_WILKIE"
    if hasattr(sky, "sun_elevation"):
        sky.sun_elevation = math.radians(sun_elevation)
        sky.sun_rotation = math.radians(sun_rotation)
    bg.inputs["Strength"].default_value = 0.12 * strength
    nt.links.new(sky.outputs["Color"], bg.inputs["Color"])
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])

    sun_data = bpy.data.lights.new("Sun", "SUN")
    sun_data.energy = 2.2 * strength
    sun_data.angle = math.radians(1.5)
    sun = bpy.data.objects.new("Sun", sun_data)
    scene.collection.objects.link(sun)
    sun.rotation_euler = (math.radians(90 - sun_elevation), 0.0, math.radians(sun_rotation))
    return sun


def setup_eevee(scene, width=1280, height=720, samples=8):
    scene.render.engine = "BLENDER_EEVEE"
    scene.eevee.taa_render_samples = samples
    scene.render.resolution_x = width
    scene.render.resolution_y = height
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGB"
    scene.view_settings.view_transform = "AgX" if "AgX" in [
        i.identifier for i in scene.view_settings.bl_rna.properties["view_transform"].enum_items
    ] else "Filmic"
    looks = [i.identifier for i in scene.view_settings.bl_rna.properties["look"].enum_items]
    for look in ("AgX - Medium High Contrast", "AgX - Base Contrast"):
        if look in looks:
            scene.view_settings.look = look
            break


# ------------------------------------------------------------------------ export

def export_mesh_npz(path, exclude_classes=("device",)):
    """Triangulated world-space mesh of all visible objects -> npz (design §3)."""
    dg = bpy.context.evaluated_depsgraph_get()
    verts, tris, cls, refl = [], [], [], []
    offset = 0
    for inst in dg.object_instances:
        ob = inst.object
        if ob.type != "MESH":
            continue
        orig = ob.original
        if not orig.visible_get() or orig.hide_render:
            continue
        name = orig.get("dt_class", "props")
        if name in exclude_classes:
            continue
        me = ob.to_mesh()
        try:
            me.calc_loop_triangles()
            nv, nt = len(me.vertices), len(me.loop_triangles)
            if nv == 0 or nt == 0:
                continue
            co = np.empty(nv * 3, np.float32)
            me.vertices.foreach_get("co", co)
            co = co.reshape(-1, 3)
            mw = np.array(inst.matrix_world, dtype=np.float64)
            co = (co @ mw[:3, :3].T + mw[:3, 3]).astype(np.float32)
            tv = np.empty(nt * 3, np.int32)
            me.loop_triangles.foreach_get("vertices", tv)
            mi = np.empty(nt, np.int32)
            me.loop_triangles.foreach_get("material_index", mi)
            mats = [m for m in me.materials]
            mrefl = np.array([float(m.get("reflectivity", 0.4)) if m else 0.4 for m in mats] or [0.4],
                             np.float32)
            verts.append(co)
            tris.append(tv.reshape(-1, 3) + offset)
            cls.append(np.full(nt, CLASS_ID.get(name, CLASS_ID["props"]), np.uint8))
            refl.append(mrefl[np.clip(mi, 0, len(mrefl) - 1)])
            offset += nv
        finally:
            ob.to_mesh_clear()
    V = np.concatenate(verts)
    F = np.concatenate(tris).astype(np.int32)
    np.savez_compressed(
        path,
        vertices=V.astype(np.float32),
        triangles=F,
        face_class=np.concatenate(cls).astype(np.uint8),
        face_reflectivity=np.concatenate(refl).astype(np.float32),
        classes=json.dumps({str(i): n for i, n in enumerate(CLASSES)}),
    )
    return len(V), len(F)


# ------------------------------------------------------------- rig / trajectory

def matrix_to_pose(M: Matrix) -> dict:
    loc, rot, _ = M.decompose()
    return {"xyz": [round(v, 6) for v in loc],
            "quat_xyzw": [round(rot.x, 8), round(rot.y, 8), round(rot.z, 8), round(rot.w, 8)]}


def write_rig_yaml(path: Path, rig: dict):
    """Minimal YAML writer (Blender has no PyYAML). Also writes a .json twin."""
    def fmt(v):
        if isinstance(v, dict):
            return "{" + ", ".join(f"{k}: {fmt(x)}" for k, x in v.items()) + "}"
        if isinstance(v, (list, tuple)):
            return "[" + ", ".join(fmt(x) for x in v) + "]"
        if isinstance(v, str):
            return v
        return repr(float(v)) if isinstance(v, float) else str(v)
    lines = ["# Generated by sim/blender/gen_device.py from the DT-1 device model.",
             "# T_a_b = pose of frame b in frame a (p_a = T_a_b * p_b)."]
    for k, v in rig.items():
        lines.append(f"{k}: {fmt(v)}")
    path.write_text("\n".join(lines) + "\n")
    path.with_suffix(".json").write_text(json.dumps(rig, indent=2))


def load_rig_json(run: Path) -> dict:
    return json.loads((run / "rig_sim.json").read_text())


def pose_to_matrix(entry: dict) -> Matrix:
    x, y, z, w = entry["quat_xyzw"]
    return Matrix.LocRotScale(Vector(entry["xyz"]), Quaternion((w, x, y, z)), None)


def load_trajectory(run: Path):
    data = np.loadtxt(run / "trajectory.csv", delimiter=",", skiprows=1)
    return data[:, 0], data[:, 1:4], data[:, 4:8]  # t, xyz, quat xyzw


def interp_pose(t, ts, xyz, q) -> Matrix:
    """Pose of base_link at time t (linear position, slerp rotation)."""
    i = int(np.clip(np.searchsorted(ts, t) - 1, 0, len(ts) - 2))
    a = float(np.clip((t - ts[i]) / (ts[i + 1] - ts[i]), 0.0, 1.0))
    p = (1 - a) * xyz[i] + a * xyz[i + 1]
    q0 = Quaternion((q[i, 3], q[i, 0], q[i, 1], q[i, 2]))
    q1 = Quaternion((q[i + 1, 3], q[i + 1, 0], q[i + 1, 1], q[i + 1, 2]))
    return Matrix.LocRotScale(Vector(p), q0.slerp(q1, a), None)


# OpenCV optical (x right, y down, z fwd) -> Blender camera (x right, y up, looks -z).
OPTICAL_TO_BLENDER_CAM = Matrix(((1, 0, 0, 0), (0, -1, 0, 0), (0, 0, -1, 0), (0, 0, 0, 1)))


def setup_camera(cam_obj, rig: dict):
    """Sets lens/sensor/shift so the Blender camera matches fx, fy, cx, cy exactly."""
    c = rig["camera"]
    scene = bpy.context.scene
    scene.render.resolution_x, scene.render.resolution_y = c["width"], c["height"]
    scene.render.pixel_aspect_x = 1.0
    scene.render.pixel_aspect_y = c["fx"] / c["fy"]
    cam = cam_obj.data
    cam.sensor_fit = "HORIZONTAL"
    cam.sensor_width = 36.0
    cam.lens = c["fx"] * cam.sensor_width / c["width"]
    m = max(c["width"], c["height"])
    cam.shift_x = (c["width"] / 2 - c["cx"]) / m
    cam.shift_y = (c["cy"] - c["height"] / 2) / m
    cam.clip_start = 0.05
    cam.clip_end = 300.0
