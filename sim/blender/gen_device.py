"""Parametric DT-1 handheld scanner (design §5, gen_device; research/08 "Proposed device").

blender -b --factory-startup -P sim/blender/gen_device.py -- --out RUN --profile mid360|airy [--render]

Device frame = base_link at the grip centre, ROS axes (x forward, y left, z up).
Sensor frames are empties (base_link, lidar_link, imu_link, zed_camera_link,
zed_left_optical) and are the single source of truth for RUN/rig_sim.yaml (+ .json).
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bpy  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

import common as C  # noqa: E402

GRIP_TILT = math.radians(12.0)     # handle top leans forward
WEDGE_PITCH = math.radians(30.0)   # LiDAR tilted forward
ZED_PITCH = math.radians(10.0)     # ZED nose down
HEAD = (0.075, 0.19, 0.085)
ZED = (0.03025, 0.17525, 0.0431)
BASELINE = 0.12
# ZED 2i (2.1 mm) at HD720: fx ~ 527 px -> HFOV ~ 101 deg (Stereolabs nominal 102 x 57).
CAMERA = {"width": 1280, "height": 720, "fx": 527.0, "fy": 527.0, "cx": 640.0, "cy": 360.0,
          "baseline": BASELINE}
LIDAR_ORIGIN_HEIGHT = {"mid360": 0.047, "airy": 0.035}  # above the mounting face
IMU_IN_LIDAR = {"mid360": (0.011, 0.02329, -0.04412), "airy": (0.0, 0.0, 0.0)}

R_LINK_OPTICAL = Matrix(((0, 0, 1), (-1, 0, 0), (0, -1, 0)))


def rot_y(a):
    return Matrix.Rotation(a, 3, "Y")


def place(obj, root, loc, rot3=None):
    obj.parent = root
    M = Matrix.Translation(Vector(loc))
    if rot3 is not None:
        M = M @ rot3.to_4x4()
    obj.matrix_world = M
    return obj


def empty(name, M, root, col, size=0.03):
    e = bpy.data.objects.new(name, None)
    e.empty_display_type = "ARROWS"
    e.empty_display_size = size
    col.objects.link(e)
    e.parent = root
    e.matrix_world = M
    e["dt_class"] = "device"
    return e


def rounded_box(name, size, col, mat, bevel, segments=4):
    obj = C.box(name, size, (0, 0, 0), col=col, dt_class="device", mat=mat)
    mod = obj.modifiers.new("bevel", "BEVEL")
    mod.width = bevel
    mod.segments = segments
    mod.limit_method = "NONE"
    obj.modifiers.new("wn", "WEIGHTED_NORMAL") if hasattr(bpy.types, "WeightedNormalModifier") else None
    for p in obj.data.polygons:
        p.use_smooth = True
    return obj


def text_mesh(name, body, size, col, mat):
    curve = bpy.data.curves.new(name, "FONT")
    curve.body = body
    curve.size = size
    curve.extrude = 0.0008
    curve.align_x = "CENTER"
    curve.align_y = "CENTER"
    obj = bpy.data.objects.new(name, curve)
    col.objects.link(obj)
    obj.data.materials.append(C.material(mat))
    obj["dt_class"] = "device"
    return obj


def build_device(profile):
    col = C.collection("DT_device")
    root = bpy.data.objects.new("DT1", None)  # == base_link
    root.empty_display_type = "PLAIN_AXES"
    root.empty_display_size = 0.05
    col.objects.link(root)
    root["dt_class"] = "device"

    # Handle: rubber grip with an orange record trigger and a tripod foot.
    R_grip = rot_y(GRIP_TILT)
    grip = C.cylinder("Grip", 0.019, 0.135, (0, 0, 0), verts=32, col=col, dt_class="device", mat="rubber")
    for p in grip.data.polygons:
        p.use_smooth = True
    place(grip, root, (0, 0, 0), R_grip)
    foot = C.cylinder("GripFoot", 0.023, 0.012, (0, 0, 0), verts=32, col=col, dt_class="device",
                      mat="anodised_dark")
    place(foot, root, R_grip @ Vector((0, 0, -0.0735)), R_grip)
    top = R_grip @ Vector((0, 0, 0.0675))
    trigger = rounded_box("Trigger", (0.012, 0.014, 0.022), col, "anodised_accent", 0.004)
    place(trigger, root, top + R_grip @ Vector((0.019, 0, -0.025)), R_grip)

    # Head body with an accent stripe and DT-1 lettering.
    hc = Vector((top.x, 0.0, top.z + HEAD[2] / 2 + 0.004))
    head = rounded_box("Head", HEAD, col, "anodised_dark", 0.008)
    place(head, root, hc)
    for side in (-1, 1):
        stripe = C.box("Stripe", (HEAD[0] * 0.9, 0.002, 0.006), (0, 0, 0), col=col, dt_class="device",
                       mat="anodised_accent")
        place(stripe, root, hc + Vector((0, side * (HEAD[1] / 2 + 0.0008), -0.025)))
        label = text_mesh("Label", "DT-1", 0.016, col, "white_plastic")
        # Text x runs toward the viewer's right, text normal points out of the side face.
        R_text = Matrix(((-side, 0, 0), (0, 0, side), (0, 1, 0)))
        place(label, root, hc + Vector((0.004, side * (HEAD[1] / 2 + 0.0012), 0.012)), R_text)
    led = C.box("StatusLED", (0.002, 0.06, 0.004), (0, 0, 0), col=col, dt_class="device", mat="screen")
    place(led, root, hc + Vector((-HEAD[0] / 2 - 0.0008, 0, 0.03)))

    # ZED 2i recessed in the front face, pitched down.
    R_zed = rot_y(ZED_PITCH)
    zc = hc + Vector((HEAD[0] / 2 - ZED[0] / 2 + 0.006, 0, -0.006))
    zed = rounded_box("ZED2i", ZED, col, "anodised_dark", 0.014, segments=6)
    place(zed, root, zc, R_zed)
    front = C.box("ZEDGlass", (0.002, ZED[1] - 0.03, ZED[2] - 0.01), (0, 0, 0), col=col, dt_class="device",
                  mat="glass")
    place(front, root, zc + R_zed @ Vector((ZED[0] / 2 + 0.001, 0, 0)), R_zed)
    for side in (-1, 1):
        lens = C.cylinder("Lens", 0.0105, 0.004, (0, 0, 0), verts=32, col=col, dt_class="device", mat="glass")
        place(lens, root, zc + R_zed @ Vector((ZED[0] / 2 + 0.003, side * BASELINE / 2, 0)),
              R_zed @ rot_y(math.pi / 2))
        ring = C.cylinder("LensRing", 0.0135, 0.003, (0, 0, 0), verts=32, col=col, dt_class="device",
                          mat="steel")
        place(ring, root, zc + R_zed @ Vector((ZED[0] / 2 + 0.0015, side * BASELINE / 2, 0)),
              R_zed @ rot_y(math.pi / 2))

    # LiDAR wedge on the top deck, pitched forward.
    deck_z = hc.z + HEAD[2] / 2
    P = Vector((hc.x, 0.0, deck_z + 0.03))
    t = math.tan(WEDGE_PITCH)
    hw = 0.0375
    vb = [(hc.x + dx, dy, deck_z) for dx, dy in ((-hw, -hw), (hw, -hw), (hw, hw), (-hw, hw))]
    vt = [(hc.x + dx, dy, P.z - t * dx) for dx, dy in ((-hw, -hw), (hw, -hw), (hw, hw), (-hw, hw))]
    wedge = C.mesh_object("Wedge", vb + vt, [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5),
                                             (2, 3, 7, 6), (3, 0, 4, 7)], col)
    C.tag(wedge, "device", "anodised_dark")
    wedge.parent = root
    R_l = rot_y(WEDGE_PITCH)
    up = R_l @ Vector((0, 0, 1))
    if profile == "mid360":
        body = rounded_box("Mid360Body", (0.065, 0.065, 0.030), col, "anodised_dark", 0.008)
        place(body, root, P + up * 0.015, R_l)
        window = C.cylinder("Mid360Window", 0.0315, 0.024, (0, 0, 0), verts=48, col=col, dt_class="device",
                            mat="glass")
        place(window, root, P + up * 0.042, R_l)
        cap = rounded_box("Mid360Cap", (0.062, 0.062, 0.007), col, "anodised_dark", 0.006)
        place(cap, root, P + up * 0.0565, R_l)
    else:
        body = C.cylinder("AiryBase", 0.030, 0.024, (0, 0, 0), verts=48, col=col, dt_class="device",
                          mat="sensor_white")
        place(body, root, P + up * 0.012, R_l)
        bpy.ops.mesh.primitive_uv_sphere_add(radius=0.029, segments=48, ring_count=24)
        dome = bpy.context.active_object
        dome.name = "AiryDome"
        C.tag(dome, "device", "glass", col=col)
        import bmesh
        bm = bmesh.new()
        bm.from_mesh(dome.data)
        bmesh.ops.delete(bm, geom=[v for v in bm.verts if v.co.z < -1e-6], context="VERTS")
        bm.to_mesh(dome.data)
        bm.free()
        for p in dome.data.polygons:
            p.use_smooth = True
        place(dome, root, P + up * 0.024, R_l)
    for p in body.data.polygons:
        p.use_smooth = True

    # Rear phone clamp with the operator phone (screen toward the user).
    R_phone = rot_y(math.radians(30))
    pc = hc + Vector((-HEAD[0] / 2 - 0.045, 0, 0.03))
    arm = C.box("ClampArm", (0.05, 0.012, 0.012), (0, 0, 0), col=col, dt_class="device", mat="anodised_dark")
    place(arm, root, hc + Vector((-HEAD[0] / 2 - 0.02, 0, 0.02)), rot_y(math.radians(-20)))
    phone = rounded_box("Phone", (0.008, 0.075, 0.155), col, "anodised_dark", 0.004)
    place(phone, root, pc, R_phone)
    screen = C.box("PhoneScreen", (0.001, 0.068, 0.145), (0, 0, 0), col=col, dt_class="device", mat="screen")
    place(screen, root, pc + R_phone @ Vector((-0.0045, 0, 0)), R_phone)

    # Sensor frames.
    lidar_pos = P + up * LIDAR_ORIGIN_HEIGHT[profile]
    M_lidar = Matrix.LocRotScale(lidar_pos, R_l.to_quaternion(), None)
    M_imu = M_lidar @ Matrix.Translation(Vector(IMU_IN_LIDAR[profile]))
    M_zed_link = Matrix.LocRotScale(zc, R_zed.to_quaternion(), None)
    M_left = Matrix.LocRotScale(zc + R_zed @ Vector((ZED[0] / 2, BASELINE / 2, 0)),
                                (R_zed @ R_LINK_OPTICAL).to_quaternion(), None)
    frames = {
        "base_link": Matrix.Identity(4),
        "lidar_link": M_lidar,
        "imu_link": M_imu,
        "zed_camera_link": M_zed_link,
        "zed_left_optical": M_left,
    }
    for name, M in frames.items():
        empty(name, M, root, col)
    return root, frames


def build_extras(root):
    """Hip box + coiled tether, only for hero renders (collection DT_hero_extras)."""
    col = C.collection("DT_hero_extras")
    hip = rounded_box("HipBox", (0.15, 0.2, 0.095), col, "anodised_dark", 0.01)
    hip.location = (-0.12, -0.32, -0.2825)
    for k in range(9):
        C.box("Fin", (0.004, 0.003, 0.06), (-0.12 + 0.077, -0.32 - 0.08 + k * 0.02, -0.2825), col=col,
              dt_class="device", mat="anodised_dark")
    C.box("HipAccent", (0.151, 0.004, 0.012), (-0.12, -0.32 + 0.1, -0.2525), col=col, dt_class="device",
                  mat="anodised_accent")
    curve = bpy.data.curves.new("Tether", "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = 0.004
    spline = curve.splines.new("POLY")
    pts = []
    start = Vector((0.0, 0.0, -0.08))
    end = Vector((-0.12, -0.22, -0.2825))
    n = 160
    for i in range(n):
        s = i / (n - 1)
        base = start.lerp(end, s)
        a = s * 2 * math.pi * 9
        pts.append(base + Vector((0.012 * math.cos(a), 0.012 * math.sin(a), 0)))
    spline.points.add(n - 1)
    for p, co in zip(spline.points, pts):
        p.co = (*co, 1.0)
    tether = bpy.data.objects.new("Tether", curve)
    tether.data.materials.append(C.material("rubber"))
    col.objects.link(tether)
    return col


def write_rig(run, profile, frames):
    inv = frames["lidar_link"].inverted()
    rig = {
        "profile": profile,
        "T_base_lidar": C.matrix_to_pose(frames["lidar_link"]),
        "T_lidar_imu": C.matrix_to_pose(inv @ frames["imu_link"]),
        "T_lidar_camera": C.matrix_to_pose(inv @ frames["zed_camera_link"]),
        "T_lidar_camera_optical": C.matrix_to_pose(inv @ frames["zed_left_optical"]),
        "camera_time_offset": 0.0,
        "camera": CAMERA,
    }
    C.write_rig_yaml(run / "rig_sim.yaml", rig)
    return rig


def look_at(obj, target):
    d = Vector(target) - obj.location
    obj.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def render_hero(run, profile, root):
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    for dev in ("OPTIX", "CUDA"):
        try:
            prefs.compute_device_type = dev
            prefs.get_devices()
            for d in prefs.devices:
                d.use = d.type != "CPU"
            scene.cycles.device = "GPU"
            break
        except TypeError:
            continue
    scene.cycles.samples = 96
    scene.cycles.use_denoising = True
    scene.render.resolution_x, scene.render.resolution_y = 1600, 1200
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "AgX"

    world = bpy.data.worlds.new("Studio")
    scene.world = world
    nt = world.node_tree or (setattr(world, "use_nodes", True) or world.node_tree)
    bg = nt.nodes.get("Background") or nt.nodes.new("ShaderNodeBackground")
    bg.inputs["Color"].default_value = (0.02, 0.022, 0.025, 1)
    bg.inputs["Strength"].default_value = 1.0
    out = nt.nodes.get("World Output") or nt.nodes.new("ShaderNodeOutputWorld")
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])

    # Seamless studio backdrop.
    bpy.ops.mesh.primitive_plane_add(size=6, location=(0, 0, -0.33))
    floor = bpy.context.active_object
    floor.data.materials.append(C.material("white_plastic"))
    floor["dt_class"] = "props"
    for name, loc, energy, size, color in (
            ("Key", (0.9, -0.7, 0.9), 120, 0.8, (1.0, 0.96, 0.9)),
            ("Fill", (-0.8, -0.9, 0.4), 40, 1.2, (0.85, 0.9, 1.0)),
            ("Rim", (-0.6, 0.9, 0.8), 90, 0.5, (1.0, 1.0, 1.0))):
        light = bpy.data.objects.new(name, bpy.data.lights.new(name, "AREA"))
        light.data.energy = energy
        light.data.size = size
        light.data.color = color
        scene.collection.objects.link(light)
        light.location = loc
        look_at(light, (0, 0, 0.08))

    cam = bpy.data.objects.new("HeroCam", bpy.data.cameras.new("HeroCam"))
    scene.collection.objects.link(cam)
    scene.camera = cam
    cam.data.lens = 70
    cam.data.dof.use_dof = True
    cam.data.dof.aperture_fstop = 5.6
    target = (0.0, 0.0, 0.12)
    paths = []
    for tag_, loc, lens in (("", (0.62, -0.72, 0.42), 60), ("_side", (0.05, -0.95, 0.16), 70),
                            ("_full", (0.95, -1.15, 0.45), 50)):
        cam.location = loc
        cam.data.lens = lens
        look_at(cam, target if tag_ != "_full" else (-0.05, -0.12, -0.08))
        cam.data.dof.focus_distance = (Vector(loc) - Vector(target)).length
        extras = bpy.data.collections.get("DT_hero_extras")
        if extras:
            extras.hide_render = tag_ != "_full"
        path = run / "device" / f"hero_{profile}{tag_}.png"
        scene.render.filepath = str(path)
        bpy.ops.render.render(write_still=True)
        paths.append(path)
    return paths


def main():
    args = C.parse_args(lambda p: (
        p.add_argument("--out", required=True),
        p.add_argument("--profile", default="mid360", choices=["mid360", "airy"]),
        p.add_argument("--render", action="store_true"),
    ))
    run = C.run_dir(args.out)
    (run / "device").mkdir(exist_ok=True)
    t0 = time.time()
    C.reset_scene()
    root, frames = build_device(args.profile)
    rig = write_rig(run, args.profile, frames)
    build_extras(root)
    bpy.ops.wm.save_as_mainfile(filepath=str(run / "device" / f"device_{args.profile}.blend"))
    print(f"[gen_device] {args.profile}: rig -> {run / 'rig_sim.yaml'}")
    for k in ("T_base_lidar", "T_lidar_camera_optical"):
        print(f"  {k}: {rig[k]}")
    if args.render:
        for p in render_hero(run, args.profile, root):
            print(f"[gen_device] rendered {p}")
    print(f"[gen_device] done in {time.time() - t0:.1f}s")


if __name__ == "__main__":
    main()
