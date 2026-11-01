"""Operator walk with the handheld device (design §5, walk).

blender -b --factory-startup -P sim/blender/walk.py -- --out RUN --variant day1 [--duration 90] [--seed 3]

Loop: start outside the building, enter on the south side, cross the ground floor,
exit east, pass the excavation and return near the start (loop closure).
Writes RUN/trajectory.csv (t,x,y,z,qx,qy,qz,qw of base_link, 1 kHz) and
RUN/world/walk_preview_<variant>.blend (world + animated device + follow camera).
Clearance against both world variants is checked; the script fails on collisions.
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix, Quaternion, Vector  # noqa: E402
from mathutils.bvhtree import BVHTree  # noqa: E402

import common as C  # noqa: E402

RATE = 1000
WAYPOINTS = [(-20.0, -12.0), (-14.0, -9.5), (-9.0, -7.5), (-9.0, -1.0), (-8.0, 5.0), (-2.0, 5.2),
             (4.0, 4.8), (10.0, 5.0), (15.0, 4.5), (15.5, -1.0), (13.4, -3.4), (12.9, -6.0), (11.0, -9.5),
             (2.0, -10.5), (-8.0, -11.0), (-16.0, -12.5), (-19.5, -12.3)]
HOLD_HEIGHT = 1.25      # base_link (grip) above ground
STATIC_START = 3.0
STATIC_END = 2.0
RAMP = 2.0
CLEARANCE = 0.35


def smooth_path(points, step=0.05, sigma=1.0):
    """Densely sampled polyline -> Gaussian-smoothed, arc-length parametrised (s, xy)."""
    pts = np.asarray(points, float)
    dense = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        n = max(int(np.linalg.norm(b - a) / step), 1)
        dense += [a + (b - a) * (k / n) for k in range(1, n + 1)]
    dense = np.array(dense)
    k = int(3 * sigma / step)
    w = np.exp(-0.5 * (np.arange(-k, k + 1) * step / sigma) ** 2)
    w /= w.sum()
    padded = np.vstack([np.repeat(dense[:1], k, 0), dense, np.repeat(dense[-1:], k, 0)])
    sm = np.stack([np.convolve(padded[:, i], w, mode="valid") for i in range(2)], 1)
    seg = np.linalg.norm(np.diff(sm, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    return s, sm


def bandlimited_noise(rng, n, lo, hi, rms):
    x = rng.standard_normal(n)
    f = np.fft.rfftfreq(n, 1.0 / RATE)
    X = np.fft.rfft(x)
    X[(f < lo) | (f > hi)] = 0
    y = np.fft.irfft(X, n)
    return y * (rms / (y.std() + 1e-12))


def lowpass(x, seconds):
    k = max(int(seconds * RATE), 1)
    w = np.ones(k) / k
    padded = np.concatenate([np.repeat(x[:1], k), x, np.repeat(x[-1:], k)])
    return np.convolve(padded, w, mode="same")[k:-k]


def world_bvh(run, variant):
    d = np.load(run / "world" / f"mesh_{variant}.npz")
    V = d["vertices"].astype(np.float64)
    F = d["triangles"]
    return BVHTree.FromPolygons([tuple(v) for v in V], [tuple(f) for f in F], all_triangles=True)


def ground_height(bvh, x, y):
    hit = bvh.ray_cast(Vector((x, y, 1.6)), Vector((0, 0, -1)), 10.0)
    return hit[0].z if hit[0] is not None else 0.0


def hand_attitude(t, yaw, gait, ramp, phase, rng, pitch0=math.radians(-3.0)):
    """Quaternions (x, y, z, w) for a hand-held device: heading ``yaw`` plus gait sway,
    slow wrist drift and 4-12 Hz tremor (scaled down when standing still)."""
    n = len(t)
    yaw = yaw + math.radians(2.0) * np.sin(phase / 2) * gait
    pitch = pitch0 + math.radians(2.0) * np.sin(2 * math.pi * 0.07 * t + 1.0)
    roll = math.radians(1.5) * np.sin(2 * math.pi * 0.11 * t + 2.0)
    tremor_scale = 0.3 + 0.7 * ramp  # continuous at walk start/stop
    yaw = yaw + np.radians(bandlimited_noise(rng, n, 4.0, 12.0, 0.12)) * tremor_scale
    pitch = pitch + np.radians(bandlimited_noise(rng, n, 4.0, 12.0, 0.15)) * tremor_scale
    roll = roll + np.radians(bandlimited_noise(rng, n, 4.0, 12.0, 0.15)) * tremor_scale
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return np.stack([qx, qy, qz, qw], 1)


def generate(duration, seed, bvh):
    rng = np.random.default_rng(seed)
    s_path, xy = smooth_path(WAYPOINTS)
    L = s_path[-1]
    walk_time = duration - STATIC_START - STATIC_END
    v = L / (walk_time - RAMP)  # trapezoid speed profile
    if not 0.8 <= v <= 1.5:
        raise SystemExit(f"[walk] path {L:.1f} m needs {v:.2f} m/s for {duration}s; adjust --duration")
    n = int(duration * RATE) + 1
    t = np.arange(n) / RATE
    tw = np.clip(t - STATIC_START, 0, walk_time)
    # Speed with slow variation; trapezoid ramps at both ends.
    speed = v * (1 + 0.08 * np.sin(2 * math.pi * 0.05 * t + rng.uniform(0, 6)))
    ramp = np.clip(np.minimum(tw / RAMP, (walk_time - tw) / RAMP), 0, 1)
    ramp = 0.5 - 0.5 * np.cos(math.pi * ramp)
    walking = (t > STATIC_START) & (t < STATIC_START + walk_time)
    vel = np.where(walking, speed * ramp, 0.0)
    s = np.cumsum(vel) / RATE
    s *= L / s[-1]
    px = np.interp(s, s_path, xy[:, 0])
    py = np.interp(s, s_path, xy[:, 1])
    # The polyline is piecewise linear; smooth it in time so accelerations stay physical.
    px = lowpass(lowpass(px, 0.25), 0.25)
    py = lowpass(lowpass(py, 0.25), 0.25)
    tang = np.gradient(xy, axis=0)
    yaw_path = np.unwrap(np.arctan2(tang[:, 1], tang[:, 0]))
    yaw = lowpass(lowpass(np.interp(s, s_path, yaw_path), 1.5), 1.5)

    gait = ramp * walking
    phase = 2 * math.pi * 1.9 * t
    left = np.stack([-np.sin(yaw), np.cos(yaw)], 1)
    sway = 0.02 * np.sin(phase / 2) * gait
    px += left[:, 0] * sway
    py += left[:, 1] * sway
    ground = np.array([ground_height(bvh, x, y) for x, y in zip(px[::100], py[::100])])
    ground = lowpass(np.interp(t, t[::100], ground), 0.5)
    pz = ground + HOLD_HEIGHT + 0.025 * np.sin(phase) * gait + 0.01 * np.sin(2 * math.pi * 0.13 * t)

    # Slow look-around pans while walking.
    pans = np.zeros(n)
    for tc, amp in ((18.0, 0.6), (37.0, -0.7), (55.0, 0.5), (72.0, -0.5)):
        if tc < duration - 6:
            pans += amp * np.exp(-0.5 * ((t - tc) / 1.6) ** 2)
    q = hand_attitude(t, yaw + pans, gait, ramp, phase, rng)
    return t, np.stack([px, py, pz], 1), q, L, v


# Pillar scan: (hold height of base_link, orbit radius) per lap around the column.
ORBIT_LAPS = ((1.25, 1.2), (0.85, 1.0), (1.60, 1.4))


def generate_orbit(seed, speed=0.3, laps=ORBIT_LAPS, center=(0.0, 0.0)):
    """Close laps around one object, always facing it (the way a column is scanned by hand).

    Each lap changes height and distance so every face is seen from below, level and above;
    the walk starts and ends at the same spot, still, for loop closure. Speed is the walking
    speed along the circle (0.3 m/s ~ 14 deg/s of turning at 1.2 m).
    """
    rng = np.random.default_rng(seed)
    lap_len = np.array([2 * math.pi * r for _, r in laps])
    knots = np.concatenate([[0.0], np.cumsum(lap_len)])
    L = float(knots[-1])
    walk_time = L / speed + RAMP
    duration = STATIC_START + walk_time + STATIC_END
    n = int(duration * RATE) + 1
    t = np.arange(n) / RATE
    tw = np.clip(t - STATIC_START, 0, walk_time)
    ramp = np.clip(np.minimum(tw / RAMP, (walk_time - tw) / RAMP), 0, 1)
    ramp = 0.5 - 0.5 * np.cos(math.pi * ramp)
    walking = (t > STATIC_START) & (t < STATIC_START + walk_time)
    vel = np.where(walking, speed * ramp, 0.0)
    s = np.cumsum(vel) / RATE
    s *= L / s[-1]
    lap = np.clip(np.searchsorted(knots, s, side="right") - 1, 0, len(laps) - 1)
    theta = 2 * math.pi * (s - knots[lap]) / lap_len[lap] + 2 * math.pi * lap
    # Height and radius ease from one lap's values to the next over the first quarter of a lap.
    hs = np.array([h for h, _ in laps])
    rs = np.array([r for _, r in laps])
    frac = np.clip((s - knots[lap]) / (0.25 * lap_len[lap]), 0, 1)
    ease = 0.5 - 0.5 * np.cos(math.pi * frac)
    prev = np.maximum(lap - 1, 0)
    r = rs[prev] + (rs[lap] - rs[prev]) * ease
    hold = hs[prev] + (hs[lap] - hs[prev]) * ease
    # ...and over the last quarter of the final lap back to the start, so the scan ends where it began.
    last = lap == len(laps) - 1
    back = np.clip((s - (knots[-1] - 0.25 * lap_len[-1])) / (0.25 * lap_len[-1]), 0, 1)
    back = np.where(last, 0.5 - 0.5 * np.cos(math.pi * back), 0.0)
    r = r + (rs[0] - r) * back
    hold = hold + (hs[0] - hold) * back
    r, hold = lowpass(r, 0.5), lowpass(hold, 0.5)
    px = center[0] + r * np.cos(theta)
    py = center[1] + r * np.sin(theta)
    yaw = theta + math.pi  # facing the column
    gait = ramp * walking
    phase = 2 * math.pi * 1.9 * t
    pz = hold + 0.02 * np.sin(phase) * gait + 0.01 * np.sin(2 * math.pi * 0.13 * t)
    # Aim halfway toward the column's middle (1.4 m): nose up on the low lap, down on the high
    # one (a positive pitch about +y points the device's x axis down).
    pitch0 = 0.5 * np.arctan2(hold - 1.4, r) + math.radians(-3.0)
    q = hand_attitude(t, yaw, gait, ramp, phase, rng, pitch0=pitch0)
    return t, np.stack([px, py, pz], 1), q, L, speed, duration


def check_clearance(bvhs, xyz):
    bad = []
    for i in range(0, len(xyz), 200):
        x, y, z = xyz[i]
        for name, bvh in bvhs.items():
            for dz in (0.0, -0.4, -0.8):
                p = Vector((x, y, z + dz))
                loc, _, _, dist = bvh.find_nearest(p, 2.0)
                if loc is not None and dist < CLEARANCE and loc.z > 0.15:
                    bad.append((i / RATE, name, round(dist, 2), tuple(round(c, 1) for c in loc)))
    return bad


def write_preview(run, variant, t, xyz, q, profile):
    bpy.ops.wm.open_mainfile(filepath=str(run / "world" / f"world_{variant}.blend"))
    with bpy.data.libraries.load(str(run / "device" / f"device_{profile}.blend")) as (src, dst):
        dst.collections = ["DT_device"]
    col = dst.collections[0]
    bpy.context.scene.collection.children.link(col)
    root = bpy.data.objects["DT1"]
    root.rotation_mode = "QUATERNION"
    scene = bpy.context.scene
    fps = 25
    scene.render.fps = fps
    step = RATE // fps
    scene.frame_start, scene.frame_end = 1, len(t[::step])
    for f, i in enumerate(range(0, len(t), step), start=1):
        root.location = xyz[i]
        root.rotation_quaternion = Quaternion((q[i, 3], q[i, 0], q[i, 1], q[i, 2]))
        root.keyframe_insert("location", frame=f)
        root.keyframe_insert("rotation_quaternion", frame=f)
    # Third-person follow camera (child of the device, behind and above).
    cam = bpy.data.objects.new("FollowCam", bpy.data.cameras.new("FollowCam"))
    scene.collection.objects.link(cam)
    cam.parent = root
    cam.location = (-2.6, -0.9, 1.1)
    cam.rotation_mode = "XYZ"
    d = Vector((0.6, 0.0, 0.1)) - Vector(cam.location)
    cam.rotation_euler = d.to_track_quat("-Z", "Z").to_euler()
    cam.data.lens = 22
    scene.camera = cam
    out = run / "world" / f"walk_preview_{variant}.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(out))
    return out


def main():
    args = C.parse_args(lambda p: (
        p.add_argument("--out", required=True),
        p.add_argument("--variant", default="day1", choices=["day1", "day2", "pillar"]),
        p.add_argument("--duration", type=float, default=90.0),
        p.add_argument("--speed", type=float, default=0.3, help="pillar orbit walking speed (m/s)"),
        p.add_argument("--seed", type=int, default=3),
        p.add_argument("--no-preview", action="store_true"),
    ))
    run = C.run_dir(args.out)
    t0 = time.time()
    if args.variant == "pillar":
        bvhs = {"pillar": world_bvh(run, "pillar")}
        t, xyz, q, L, v, args.duration = generate_orbit(args.seed, args.speed)
    else:
        bvhs = {v: world_bvh(run, v) for v in ("day1", "day2") if (run / "world" / f"mesh_{v}.npz").exists()}
        t, xyz, q, L, v = generate(args.duration, args.seed, bvhs[args.variant])
    bad = check_clearance(bvhs, xyz)
    if bad:
        for b in bad[:20]:
            print("[walk] clearance violation", b)
        raise SystemExit(f"[walk] {len(bad)} clearance violations")
    with open(run / "trajectory.csv", "w") as f:
        f.write("t,x,y,z,qx,qy,qz,qw\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.3f},{xyz[i, 0]:.8f},{xyz[i, 1]:.8f},{xyz[i, 2]:.8f},"
                    f"{q[i, 0]:.10f},{q[i, 1]:.10f},{q[i, 2]:.10f},{q[i, 3]:.10f}\n")
    gap = np.linalg.norm(xyz[-1] - xyz[0])
    print(f"[walk] {L:.1f} m at {v:.2f} m/s, {args.duration:.0f}s, start-end gap {gap:.2f} m, "
          f"checked against {sorted(bvhs)} -> {run / 'trajectory.csv'}")
    if not args.no_preview:
        profile = C.load_rig_json(run)["profile"]
        print(f"[walk] preview -> {write_preview(run, args.variant, t, xyz, q, profile)}")
    print(f"[walk] done in {time.time() - t0:.1f}s")


if __name__ == "__main__":
    main()
