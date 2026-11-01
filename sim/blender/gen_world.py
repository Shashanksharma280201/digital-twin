"""Procedural construction site (design §5, gen_world).

blender -b --factory-startup -P sim/blender/gen_world.py -- --out RUN --variant day1 [--seed 7]

Writes RUN/world/world_<variant>.blend and RUN/world/mesh_<variant>.npz.
day2 is the same site one "week" later: roof slab poured, more blockwork, scaffold
extended, excavator moved, spoil pile reduced, a new pallet stack.
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bpy  # noqa: E402
import numpy as np  # noqa: E402

import common as C  # noqa: E402

S = C.SITE
COLX, COLY = S["col_x"], S["col_y"]
H = S["storey"]
SLAB = 0.25
STAIR_BAY = (6.0, 12.0, 8.0, 14.0)  # x0, x1, y0, y1


# ----------------------------------------------------------------------- shapes

def frustum(name, r0, r1, h, loc, verts=16, col=None, dt_class="props", mat=None):
    ring = [(math.cos(2 * math.pi * i / verts), math.sin(2 * math.pi * i / verts)) for i in range(verts)]
    v = [(r0 * x, r0 * y, 0.0) for x, y in ring] + [(r1 * x, r1 * y, h) for x, y in ring]
    f = [(i, (i + 1) % verts, verts + (i + 1) % verts, verts + i) for i in range(verts)]
    f += [tuple(reversed(range(verts))), tuple(range(verts, 2 * verts))]
    obj = C.mesh_object(name, v, f, col)
    obj.location = loc
    return C.tag(obj, dt_class, mat)


def mound(name, radius, height, loc, rng, col, mat="soil", res=24):
    """Irregular spoil pile: displaced cone grid."""
    verts, faces = [], []
    rings = 10
    for r in range(rings + 1):
        rr = radius * r / rings
        for k in range(res):
            a = 2 * math.pi * k / res
            jitter = 1.0 + 0.12 * math.sin(3 * a + rng.uniform(0, 6)) if r > 0 else 1.0
            z = height * (1 - (r / rings) ** 1.6) * (1 + 0.05 * rng.standard_normal())
            verts.append((rr * jitter * math.cos(a), rr * jitter * math.sin(a), max(z, 0.0) - 0.05))
    for r in range(rings):
        for k in range(res):
            a, b = r * res + k, r * res + (k + 1) % res
            faces.append((a, b, b + res, a + res))
    obj = C.mesh_object(name, verts, faces, col)
    obj.location = loc
    return C.tag(obj, "ground", mat)


def terrain(col, rng):
    x0, x1 = S["extent_x"]
    y0, y1 = S["extent_y"]
    px0, px1, py0, py1, depth = S["pit"]
    step = 0.5
    xs = np.arange(x0 - 4, x1 + 4 + 1e-6, step)
    ys = np.arange(y0 - 4, y1 + 4 + 1e-6, step)
    X, Y = np.meshgrid(xs, ys)
    Z = np.zeros_like(X)
    for _ in range(6):  # smooth low-frequency undulation
        kx, ky = rng.uniform(0.05, 0.25, 2)
        Z += 0.05 * np.sin(kx * X + rng.uniform(0, 6)) * np.cos(ky * Y + rng.uniform(0, 6))
    Z += 0.015 * rng.standard_normal(Z.shape)
    # Flat working pad around the building.
    pad = (X > COLX[0] - 4) & (X < COLX[-1] + 4) & (Y > COLY[0] - 4) & (Y < COLY[-1] + 4)
    Z[pad] = -0.02
    # Excavation pit with 1:1 batters.
    dx = np.maximum(np.maximum(px0 - X, X - px1), 0)
    dy = np.maximum(np.maximum(py0 - Y, Y - py1), 0)
    d = np.hypot(dx, dy)
    Z = np.where(d < depth, np.minimum(Z, -depth + d), Z)
    ny, nx = X.shape
    verts = np.stack([X, Y, Z], -1).reshape(-1, 3)
    faces = [(j * nx + i, j * nx + i + 1, (j + 1) * nx + i + 1, (j + 1) * nx + i)
             for j in range(ny - 1) for i in range(nx - 1)]
    obj = C.mesh_object("Terrain", verts, faces, col)
    C.tag(obj, "ground", "soil")
    obj.data.materials.append(C.material("gravel"))
    # Gravel on the working pad, soil elsewhere.
    centers = np.array([[(X[j, i] + X[j, i + 1]) / 2, (Y[j, i] + Y[j + 1, i]) / 2]
                        for j in range(ny - 1) for i in range(nx - 1)])
    in_pad = ((centers[:, 0] > COLX[0] - 4) & (centers[:, 0] < COLX[-1] + 4)
              & (centers[:, 1] > COLY[0] - 4) & (centers[:, 1] < COLY[-1] + 4))
    obj.data.polygons.foreach_set("material_index", in_pad.astype(np.int32))
    for p in obj.data.polygons:
        p.use_smooth = True
    return obj


# --------------------------------------------------------------------- structure

def bays():
    for i in range(len(COLX) - 1):
        for j in range(len(COLY) - 1):
            yield COLX[i], COLX[i + 1], COLY[j], COLY[j + 1]


def is_stair_bay(x0, x1, y0, y1):
    return (x0, x1, y0, y1) == STAIR_BAY


def slab_level(level, col, mat="concrete", name="Slab"):
    """Slab with top at level*H (bays, stair bay left open above ground) + downstand beams."""
    top = level * H
    parts = []
    for x0, x1, y0, y1 in bays():
        if level > 0 and is_stair_bay(x0, x1, y0, y1):
            continue
        parts.append(C.box(f"{name}_bay", (x1 - x0 + 0.01, y1 - y0 + 0.01, SLAB),
                           ((x0 + x1) / 2, (y0 + y1) / 2, top - SLAB / 2), col=col,
                           dt_class="concrete_structure", mat=mat))
    if level > 0:
        for y in COLY:
            parts.append(C.box(f"{name}_beamx", (COLX[-1] - COLX[0] + 0.4, 0.3, 0.5),
                               (0.0, y, top - SLAB - 0.25), col=col,
                               dt_class="concrete_structure", mat=mat))
        for x in COLX:
            parts.append(C.box(f"{name}_beamy", (0.3, COLY[-1] - COLY[0] + 0.4, 0.5),
                               (x, (COLY[0] + COLY[-1]) / 2, top - SLAB - 0.25), col=col,
                               dt_class="concrete_structure", mat=mat))
    return C.join(parts, f"{name}_L{level}")


def columns(level, col):
    base = level * H
    h = H - SLAB - 0.5  # up to beam soffit
    parts = [C.box("col", (0.4, 0.4, h), (x, y, base + h / 2), col=col,
                   dt_class="concrete_structure", mat="concrete")
             for x in COLX for y in COLY]
    return C.join(parts, f"Columns_L{level}")


def stair_core(col):
    x0, x1, y0, y1 = STAIR_BAY
    top = S["levels"] * H
    t = 0.2
    walls = [
        C.box("core_e", (t, y1 - y0, top), (x1 - 0.3, (y0 + y1) / 2, top / 2), col=col,
              dt_class="concrete_structure", mat="concrete"),
        C.box("core_n", (x1 - x0 - 0.6, t, top), ((x0 + x1) / 2, y1 - 0.3, top / 2), col=col,
              dt_class="concrete_structure", mat="concrete"),
    ]
    steps = []
    rise, going, n = H / 18, 0.25, 18
    for lvl in range(S["levels"]):
        for k in range(n):
            z = lvl * H + (k + 0.5) * rise
            steps.append(C.box("step", (going, 1.2, rise), (x0 + 1.0 + k * going, y1 - 1.2, z - rise / 2 + rise / 2),
                               col=col, dt_class="concrete_structure", mat="concrete"))
    return C.join(walls + steps, "StairCore")


def blockwork(col, variant):
    """Ground-floor masonry walls; day2 has more of them."""
    t, h = 0.19, H - SLAB - 0.5
    specs = [  # (x0, y0, x1, y1, height fraction)
        (-12, 14, 0, 14, 1.0),
        (-12, 2, -12, 14, 1.0),
        (0, 14, 6, 14, 0.55),        # in progress: stepped top
    ]
    if variant == "day2":
        specs[2] = (0, 14, 6, 14, 1.0)
        specs += [(0, 8, 0, 14, 1.0), (6, -4, 12, -4, 0.4), (-12, -4, -12, 2, 0.7)]
    parts = []
    for x0, y0, x1, y1, frac in specs:
        length = math.hypot(x1 - x0, y1 - y0) - 0.4
        hh = h * frac
        horizontal = abs(y1 - y0) < 1e-6
        size = (length, t, hh) if horizontal else (t, length, hh)
        parts.append(C.box("wall", size, ((x0 + x1) / 2, (y0 + y1) / 2, hh / 2), col=col,
                           dt_class="masonry", mat="blocks"))
        if frac < 1.0:  # loose blocks on top of an unfinished wall
            for k in range(4):
                off = -length / 2 + 0.4 + k * 0.5
                pos = ((x0 + x1) / 2 + (off if horizontal else 0), (y0 + y1) / 2 + (0 if horizontal else off),
                       hh + 0.095)
                parts.append(C.box("block", (0.39 if horizontal else t, t if horizontal else 0.39, 0.19), pos,
                                   col=col, dt_class="masonry", mat="blocks"))
    return C.join(parts, "Blockwork")


def roof(col, variant, rng):
    top = S["levels"] * H
    parts = []
    if variant == "day1":
        # Formwork deck + edge forms + two layers of rebar mesh; shores on level 2.
        x0, x1, y0, y1 = COLX[0] - 0.2, COLX[-1] + 0.2, COLY[0] - 0.2, COLY[-1] + 0.2
        deck = C.box("deck", (x1 - x0, y1 - y0, 0.02), ((x0 + x1) / 2, (y0 + y1) / 2, top - SLAB - 0.01),
                     col=col, dt_class="formwork", mat="plywood")
        edges = [C.box("edge", (x1 - x0, 0.02, SLAB + 0.05), ((x0 + x1) / 2, yy, top - SLAB / 2 + 0.02),
                       col=col, dt_class="formwork", mat="formwork_red") for yy in (y0, y1)]
        edges += [C.box("edge", (0.02, y1 - y0, SLAB + 0.05), (xx, (y0 + y1) / 2, top - SLAB / 2 + 0.02),
                        col=col, dt_class="formwork", mat="formwork_red") for xx in (x0, x1)]
        C.join([deck] + edges, "RoofFormwork")
        bars = []
        for zl in (top - SLAB + 0.05, top - 0.06):
            for y in np.arange(y0 + 0.15, y1, 0.3):
                bars.append(C.tube_between("bar", (x0 + 0.1, y, zl), (x1 - 0.1, y, zl), 0.008, verts=6,
                                           col=col, dt_class="rebar", mat="rebar"))
            for x in np.arange(x0 + 0.15, x1, 0.3):
                bars.append(C.tube_between("bar", (x, y0 + 0.1, zl + 0.016), (x, y1 - 0.1, zl + 0.016), 0.008,
                                           verts=6, col=col, dt_class="rebar", mat="rebar"))
        C.join(bars, "RoofRebar")
        shores = []
        base = 2 * H
        for x in np.arange(COLX[0] + 0.9, COLX[-1], 1.5):
            for y in np.arange(COLY[0] + 0.9, COLY[-1], 1.5):
                if any(abs(x - cx) < 0.5 for cx in COLX) and any(abs(y - cy) < 0.5 for cy in COLY):
                    continue
                if STAIR_BAY[0] < x < STAIR_BAY[1] and STAIR_BAY[2] < y < STAIR_BAY[3]:
                    continue
                shores.append(C.tube_between("shore", (x, y, base), (x, y, top - SLAB - 0.03), 0.024,
                                             verts=6, col=col, dt_class="formwork", mat="steel"))
        C.join(shores, "Shores")
    else:
        parts.append(slab_level(S["levels"], col, mat="concrete_fresh", name="Roof"))
        starters = []
        for x in COLX:
            for y in COLY:
                for dx in (-0.12, 0.12):
                    for dy in (-0.12, 0.12):
                        starters.append(C.tube_between("starter", (x + dx, y + dy, top), (x + dx, y + dy, top + 1.0),
                                                       0.01, verts=6, col=col, dt_class="rebar", mat="rebar"))
        C.join(starters, "StarterBars")
    return parts


def scaffold(col, variant):
    """Tube-and-board scaffold along the north façade (outside y = 14)."""
    x0, x1 = -12.5, (0.0 if variant == "day1" else 12.5)
    yi, yo = COLY[-1] + 0.45, COLY[-1] + 0.45 + 0.73
    lifts = [2.0 * k for k in range(1, 6)]
    xs = np.arange(x0, x1 + 1e-6, 2.5)
    tubes = []
    for x in xs:
        for y in (yi, yo):
            tubes.append(C.tube_between("std", (x, y, 0.0), (x, y, lifts[-1] + 1.0), 0.024, col=col))
    for z in lifts:
        for y in (yi, yo):
            tubes.append(C.tube_between("ledger", (x0, y, z), (x1, y, z), 0.024, col=col))
            tubes.append(C.tube_between("guard", (x0, y if y == yo else yo, z + 1.0),
                                        (x1, y if y == yo else yo, z + 1.0), 0.024, col=col))
        for x in xs:
            tubes.append(C.tube_between("transom", (x, yi - 0.1, z), (x, yo + 0.1, z), 0.024, col=col))
    for k, x in enumerate(xs[:-1]):
        if k % 2 == 0:
            tubes.append(C.tube_between("brace", (x, yo + 0.05, 0.0), (x + 2.5, yo + 0.05, lifts[-1]), 0.024,
                                        col=col))
    boards = [C.box("deck", (x1 - x0, 0.7, 0.04), ((x0 + x1) / 2, (yi + yo) / 2, z + 0.045), col=col,
                    dt_class="scaffold", mat="plywood") for z in lifts]
    C.join(tubes + boards, "Scaffold")


# ------------------------------------------------------------------------- props

def cone(name, loc, col):
    base = C.box(f"{name}_base", (0.38, 0.38, 0.03), (loc[0], loc[1], 0.015), col=col, dt_class="props",
                 mat="safety_orange")
    body = frustum(f"{name}_body", 0.15, 0.03, 0.72, (loc[0], loc[1], 0.03), col=col, mat="safety_orange")
    band = frustum(f"{name}_band", 0.098, 0.077, 0.13, (loc[0], loc[1], 0.33), col=col, mat="white_plastic")
    return C.join([base, body, band], name)


def jersey_barrier(name, loc, yaw, col):
    prof = [(-0.30, 0.0), (0.30, 0.0), (0.30, 0.08), (0.18, 0.33), (0.10, 0.81), (-0.10, 0.81),
            (-0.18, 0.33), (-0.30, 0.08)]
    L = 2.0
    v = [(-L / 2, y, z) for y, z in prof] + [(L / 2, y, z) for y, z in prof]
    n = len(prof)
    f = [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
    f += [tuple(reversed(range(n))), tuple(range(n, 2 * n))]
    obj = C.mesh_object(name, v, f, col)
    obj.location = loc
    obj.rotation_euler = (0, 0, yaw)
    return C.tag(obj, "props", "concrete")


def pallet(name, loc, col, load=True, yaw=0.0, rng=None):
    parts = []
    for dy in (-0.35, 0.0, 0.35):
        parts.append(C.box("runner", (1.2, 0.1, 0.1), (0, dy, 0.05), col=col, dt_class="props", mat="plywood"))
    for dx in np.linspace(-0.55, 0.55, 7):
        parts.append(C.box("slat", (0.1, 0.8, 0.022), (dx, 0, 0.111), col=col, dt_class="props", mat="plywood"))
    if load:
        for i in range(3):
            for j in range(2):
                for k in range(4):
                    parts.append(C.box("blk", (0.39, 0.19, 0.19),
                                       (-0.4 + i * 0.4, -0.2 + j * 0.4 * 0.95, 0.217 + k * 0.195),
                                       col=col, dt_class="props", mat="blocks"))
    obj = C.join(parts, name)
    obj.location = loc
    obj.rotation_euler = (0, 0, yaw)
    return obj


def fence(col):
    x0, x1 = S["extent_x"]
    y0, y1 = S["extent_y"]
    parts = []
    runs = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
    for (ax, ay), (bx, by) in runs:
        length = math.hypot(bx - ax, by - ay)
        n = int(length // 3.5)
        for k in range(n):
            t0, t1 = k / n, (k + 1) / n
            cx, cy = ax + (bx - ax) * (t0 + t1) / 2, ay + (by - ay) * (t0 + t1) / 2
            if ay == y0 and by == y0 and -6 < cx < 0:  # site gate gap on the south side
                continue
            yaw = math.atan2(by - ay, bx - ax)
            panel = C.box("panel", (length / n - 0.05, 0.02, 1.9), (cx, cy, 1.05), (0, 0, yaw), col=col,
                          dt_class="fence", mat="fence_mesh")
            foot = C.box("foot", (0.6, 0.2, 0.12), (ax + (bx - ax) * t0, ay + (by - ay) * t0, 0.06),
                         (0, 0, yaw), col=col, dt_class="fence", mat="concrete")
            parts += [panel, foot]
    return C.join(parts, "SiteFence")


def site_office(col):
    ox, oy = S["office"]
    body = C.box("office", (6.06, 2.44, 2.59), (ox, oy, 1.295 + 0.15), col=col, dt_class="equipment",
                 mat="container_blue", bevel=0.02)
    wins = [C.box("win", (1.2, 0.04, 0.9), (ox + dx, oy - 1.23, 1.7), col=col, dt_class="equipment", mat="glass")
            for dx in (-1.8, 1.8)]
    door = C.box("door", (0.9, 0.04, 2.0), (ox, oy - 1.23, 1.15), col=col, dt_class="equipment", mat="steel")
    blocks = [C.box("sleeper", (0.3, 2.4, 0.15), (ox + dx, oy, 0.075), col=col, dt_class="equipment",
                    mat="concrete") for dx in (-2.6, 0.0, 2.6)]
    C.join([body] + wins + [door] + blocks, "SiteOffice")
    gen = C.box("generator", (1.6, 0.9, 1.1), (ox + 4.5, oy, 0.55), col=col, dt_class="equipment",
                mat="safety_yellow", bevel=0.03)
    gen.name = "Generator"
    for k in range(4):
        C.cylinder(f"barrel{k}", 0.29, 0.88, (ox - 4.2 + (k % 2) * 0.62, oy - 0.3 + (k // 2) * 0.62, 0.44),
                   verts=16, col=col, dt_class="props", mat="container_blue" if k % 2 else "safety_orange")


def excavator(col, loc, yaw, boom_angle=-35.0):
    """Parametric ~20 t excavator. Origin at ground under the slew ring."""
    parts = []
    for side in (-1, 1):
        parts.append(C.box("track", (4.4, 0.6, 0.85), (0, side * 1.3, 0.425), col=col, dt_class="equipment",
                           mat="rubber", bevel=0.2))
    parts.append(C.box("carbody", (2.2, 2.0, 0.5), (0, 0, 0.85), col=col, dt_class="equipment", mat="steel"))
    house = C.box("house", (3.0, 2.5, 1.0), (-0.4, 0, 1.6), col=col, dt_class="equipment", mat="safety_yellow",
                  bevel=0.05)
    cw = C.box("counterweight", (0.6, 2.5, 0.9), (-2.0, 0, 1.45), col=col, dt_class="equipment",
               mat="safety_yellow", bevel=0.1)
    cab = C.box("cab", (1.0, 0.9, 1.5), (0.6, 0.75, 2.6), col=col, dt_class="equipment", mat="safety_yellow",
                bevel=0.03)
    glass = C.box("cabglass", (1.02, 0.8, 0.9), (0.62, 0.75, 2.8), col=col, dt_class="equipment", mat="glass")
    parts += [house, cw, cab, glass]
    # Boom / arm / bucket as a planar linkage in the local XZ plane.
    a_boom = math.radians(boom_angle + 75.0)
    p0 = np.array([1.0, -0.2, 1.8])
    p1 = p0 + 5.7 * np.array([math.cos(a_boom), 0, math.sin(a_boom)])
    a_arm = a_boom - math.radians(120)
    p2 = p1 + 2.9 * np.array([math.cos(a_arm), 0, math.sin(a_arm)])
    for name, a, b, w in (("boom", p0, p1, 0.55), ("arm", p1, p2, 0.4)):
        mid = (a + b) / 2
        d = b - a
        ang = -math.atan2(d[2], d[0])
        parts.append(C.box(name, (float(np.linalg.norm(d)), 0.45, w), tuple(mid), (0, ang, 0), col=col,
                           dt_class="equipment", mat="safety_yellow", bevel=0.04))
    parts.append(C.box("bucket", (0.9, 1.1, 0.8), tuple(p2 + np.array([0.1, 0, -0.3])), (0, 0.6, 0), col=col,
                       dt_class="equipment", mat="steel", bevel=0.05))
    obj = C.join(parts, "Excavator")
    obj.location = loc
    obj.rotation_euler = (0, 0, yaw)
    return obj


def props(col, variant, rng):
    for k, (x, y) in enumerate([(-12.5, -7.0), (-7.5, -7.0), (13.5, 0.0), (-3.0, -7.0), (15.0, -5.2),
                                (19.0, -5.2), (23.0, -5.2), (-22.0, -8.0)]):
        cone(f"Cone{k}", (x, y), col)
    for k in range(5):
        jersey_barrier(f"Barrier{k}", (17.0 + 2.05 * k, -5.0, 0.0), 0.0, col)
    pallet("PalletA", (-3.0, 11.0, 0.0), col)
    pallet("PalletB", (3.0, -1.0, 0.0), col, yaw=0.4)
    pallet("PalletC", (-18.0, -2.0, 0.0), col)
    pallet("PalletD", (-19.5, -0.6, 0.0), col, load=False, yaw=0.2)
    if variant == "day2":
        pallet("PalletE", (-16.0, -4.5, 0.0), col, yaw=-0.3)
        pallet("PalletF", (-16.0, -6.0, 0.0), col, yaw=0.1)
    bundle = [C.cylinder("rb", 0.016, 6.0, (-16.0 + 0.04 * (k % 6), 9.0 + 0.04 * (k // 6), 0.1 + 0.03 * (k % 3)),
                         rot=(0, math.pi / 2, 0), verts=6, col=col, dt_class="rebar", mat="rebar")
              for k in range(36)]
    sleepers = [C.box("sl", (0.1, 1.0, 0.08), (-16.0 + dx, 9.1, 0.04), col=col, dt_class="props", mat="plywood")
                for dx in (-2.0, 0.0, 2.0)]
    C.join(bundle + sleepers, "RebarBundle")


# ------------------------------------------------------------------- pillar scene

PILLAR = 0.45          # column section (m)
PILLAR_H = 3.0
COURT = 4.5            # half-size of the courtyard walls around it


def pillar_scene(col):
    """One site column with small features to resolve, in an open courtyard (context for tracking).

    Detail sizes: 2 cm chamfers, 1 cm joint band, 2.4 cm conduit, 2 cm rebar, 1 cm sign plate.
    """
    h = PILLAR / 2
    parts = [C.box("Floor", (2 * COURT + 1, 2 * COURT + 1, 0.2), (0, 0, -0.1), col=col,
                   dt_class="ground", mat="concrete")]
    for i, (sx, sy, x, y) in enumerate(((2 * COURT, 0.2, 0, COURT), (2 * COURT, 0.2, 0, -COURT),
                                        (0.2, 2 * COURT, COURT, 0), (0.2, 2 * COURT, -COURT, 0))):
        parts.append(C.box(f"CourtWall{i}", (sx, sy, 2.0), (x, y, 1.0), col=col,
                           dt_class="masonry", mat="blocks"))
    col_parts = [
        C.box("Plinth", (PILLAR + 0.2, PILLAR + 0.2, 0.12), (0, 0, 0.06), col=col,
              dt_class="concrete_structure", mat="concrete_fresh", bevel=0.01),
        C.box("Column", (PILLAR, PILLAR, PILLAR_H), (0, 0, PILLAR_H / 2), col=col,
              dt_class="concrete_structure", mat="concrete", bevel=0.02),
        C.box("JointBand", (PILLAR + 0.02, PILLAR + 0.02, 0.03), (0, 0, 1.0), col=col,
              dt_class="concrete_structure", mat="concrete_fresh"),
    ]
    for k, (dx, dy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):  # starter bars on top
        col_parts.append(C.cylinder(f"Rebar{k}", 0.01, 0.5, (dx * (h - 0.06), dy * (h - 0.06), PILLAR_H + 0.25),
                                    verts=8, col=col, dt_class="rebar", mat="rebar"))
    # +x face: junction box with a conduit down to the floor
    col_parts.append(C.box("JunctionBox", (0.08, 0.15, 0.2), (h + 0.04, 0.0, 1.4), col=col,
                           dt_class="props", mat="safety_yellow", bevel=0.005))
    col_parts.append(C.cylinder("Conduit", 0.012, 1.3, (h + 0.015, 0.0, 0.65), verts=10, col=col,
                                dt_class="props", mat="galvanised"))
    # -y face: sign plate; -x face: steel anchor plate
    col_parts.append(C.box("SignPlate", (0.3, 0.01, 0.2), (0.0, -h - 0.005, 1.6), col=col,
                           dt_class="props", mat="white_plastic"))
    col_parts.append(C.box("AnchorPlate", (0.02, 0.2, 0.2), (-h - 0.01, 0.0, 0.8), col=col,
                           dt_class="props", mat="steel"))
    C.join(parts, "Courtyard")
    return C.join(col_parts, "Pillar")


# -------------------------------------------------------------------------- main

def build(variant, seed):
    rng = np.random.default_rng(seed)
    scene = C.reset_scene()
    site = C.collection("Site")
    if variant == "pillar":
        pillar_scene(site)
        C.setup_sky(scene)
        C.setup_eevee(scene)
        return scene
    struct = C.collection("Structure", site)
    temp = C.collection("Temporary", site)
    equip = C.collection("Equipment", site)

    terrain(site, rng)
    px0, px1, py0, py1, _ = S["pit"]
    mound("SpoilPileA", 3.0, 1.8 if variant == "day1" else 0.9, (10.0, -16.0, 0.0), rng, site)
    if variant == "day1":
        mound("SpoilPileB", 2.2, 1.3, (5.0, -15.5, 0.0), rng, site)

    slab_level(0, struct)
    for lvl in range(S["levels"]):
        columns(lvl, struct)
    for lvl in (1, 2):
        slab_level(lvl, struct)
    stair_core(struct)
    blockwork(struct, variant)
    roof(struct, variant, rng)
    scaffold(temp, variant)
    fence(temp)
    site_office(equip)
    if variant == "day1":
        excavator(equip, (21.5, -2.5, 0.0), -math.pi / 2)
    else:
        excavator(equip, (-21.0, 6.0, 0.0), math.radians(25), boom_angle=-15.0)
    props(temp, variant, rng)
    C.setup_sky(scene)
    C.setup_eevee(scene)
    return scene


def main():
    args = C.parse_args(lambda p: (
        p.add_argument("--out", required=True),
        p.add_argument("--variant", default="day1", choices=["day1", "day2", "pillar"]),
        p.add_argument("--seed", type=int, default=7),
    ))
    run = C.run_dir(args.out)
    (run / "world").mkdir(exist_ok=True)
    t0 = time.time()
    build(args.variant, args.seed)
    blend = run / "world" / f"world_{args.variant}.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(blend))
    nv, nf = C.export_mesh_npz(run / "world" / f"mesh_{args.variant}.npz")
    print(f"[gen_world] {args.variant}: {len(bpy.data.objects)} objects, {nf} triangles, {nv} vertices "
          f"-> {blend} ({time.time() - t0:.1f}s)")


if __name__ == "__main__":
    main()
