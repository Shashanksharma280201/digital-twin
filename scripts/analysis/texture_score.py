"""Score colour against real frames that were held back: render the model from those camera poses.

python scripts/analysis/texture_score.py SESSION [--holdout 25] [--frames 40]

Runs `dtp texture --holdout` and recolours the mesh vertices from the same frames, so neither model
has seen the frames it is scored on. Per frame, one brightness gain is fitted first (auto-exposure
is not what is being measured). Writes products/textured/score.json and score.jpg
(real | vertex colour | texture). Held-out frames are spread over the whole recording.
"""
import argparse
import json
from contextlib import ExitStack
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d

from dt_pipeline import steps
from dt_pipeline.colour import GpuColourer, colour_frames
from dt_pipeline.geometry import inv
from dt_pipeline.io import Bag
from dt_pipeline.io.svo import Svo
from dt_pipeline.session import Session


def open_frames(session, stack, scale):
    svo = session.path / 'zed.svo2'
    if svo.exists() and svo.stat().st_size > 1_000_000:
        s = stack.enter_context(Svo(svo, scale))
        return s.K, s.images(lazy=True), s.frames
    b = stack.enter_context(Bag(session.bag))
    return b.camera_info().K, b.images(lazy=True), len(b.image_stamps())


def render(scene, K, T_wc, w, h):
    rays = scene.create_rays_pinhole(o3d.core.Tensor(K), o3d.core.Tensor(inv(T_wc)), w, h)
    ans = scene.cast_rays(rays)
    return ans['primitive_ids'].numpy(), ans['primitive_uvs'].numpy()


def score(pred, real, mask):
    p, r = pred[mask].astype(np.float32), real[mask].astype(np.float32)
    ok = (r.mean(1) > 12) & (r.mean(1) < 243)
    g = np.median(r[ok].mean(1) / np.maximum(p[ok].mean(1), 1))
    return 10 * np.log10(255 ** 2 / max(((np.clip(p * g, 0, 255) - r) ** 2).mean(), 1e-6)), g


def detail(img, mask):
    lap = cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_32F)
    return float(lap[cv2.erode(mask.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0].var())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', type=Path)
    ap.add_argument('--holdout', type=int, default=25)
    ap.add_argument('--frames', type=int, default=40)
    ap.add_argument('--scale', type=float, default=1.0)
    ap.add_argument('--sharpen', type=float, default=4.0)
    ap.add_argument('--min-cover', type=float, default=0.8, help='skip views where the mesh fills less than this')
    a = ap.parse_args()
    s = Session(a.session)
    steps.texture(s, svo_scale=a.scale, holdout=a.holdout, sharpen=a.sharpen)
    tr = steps.load_traj(s)
    T_lo = s.rig.lidar_to_optical
    tex = o3d.io.read_triangle_mesh(str(s.products / 'textured/mesh.obj'), True)
    atlas = np.asarray(tex.textures[0])[..., ::-1].copy()
    uvs = np.asarray(tex.triangle_uvs).reshape(-1, 3, 2)
    faces = np.asarray(tex.triangles)
    side = atlas.shape[0]

    # Vertex colours from the same frames, also blind to the held-out ones.
    m = o3d.io.read_triangle_mesh(str(s.products / 'mesh.ply'))
    verts = np.asarray(m.vertices)
    vfaces = np.asarray(m.triangles)
    with ExitStack() as st:
        K, frames, _ = open_frames(s, st, a.scale)
        col = GpuColourer(verts, K, 15.0, sharpen=3.0)
        colour_frames(col, frames, tr, T_lo, s.rig.camera_time_offset, holdout=a.holdout)
    vrgb = col.result()[0][:, ::-1]  # BGR

    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(tex))
    rows, shots = [], []
    with ExitStack() as st:
        K, frames, total = open_frames(s, st, a.scale)
        every = a.holdout * max(1, total // (a.holdout * a.frames))  # spread over the whole recording
        for i, (stamp, get) in enumerate(frames):
            if i % every:
                continue
            t = stamp + s.rig.camera_time_offset
            if not tr.covers(np.array([t]))[0]:
                continue
            if len(rows) >= a.frames:
                break
            real = get().copy() if callable(get) else get
            h, w = real.shape[:2]
            T_wc = tr.interpolate(t)[0] @ T_lo
            ids, bary = render(scene, K, T_wc, w, h)
            mask = ids != scene.INVALID_ID
            if mask.mean() < a.min_cover:  # mostly geometry the mesh lacks (operator zone, sky): nothing to compare
                continue
            I = ids[mask].astype(np.int64)
            b = np.column_stack([1 - bary[mask].sum(1), bary[mask]])
            vimg = np.zeros_like(real)
            vimg[mask] = (vrgb[vfaces[I]] * b[..., None]).sum(1).clip(0, 255).astype(np.uint8)
            uv = (uvs[I] * b[..., None]).sum(1)
            mx, my = np.zeros((h, w), np.float32), np.zeros((h, w), np.float32)
            mx[mask], my[mask] = uv[:, 0] * side - 0.5, (1 - uv[:, 1]) * side - 0.5
            timg = cv2.remap(atlas, mx, my, cv2.INTER_LINEAR)
            timg[~mask] = 0
            pv, gv = score(vimg, real, mask)
            pt, gt = score(timg, real, mask)
            rows.append([pv, pt, detail(vimg, mask), detail(timg, mask), detail(real, mask)])
            if len(rows) % max(1, a.frames // 4) == 1:
                shots.append(np.hstack([real, np.clip(vimg * gv, 0, 255).astype(np.uint8),
                                        np.clip(timg * gt, 0, 255).astype(np.uint8)]))
    r = np.array(rows)
    out = {'frames_scored': len(r), 'holdout_every': a.holdout,
           'vertex_psnr_db': round(float(r[:, 0].mean()), 2), 'texture_psnr_db': round(float(r[:, 1].mean()), 2),
           'detail_vertex': round(float(np.median(r[:, 2])), 1), 'detail_texture': round(float(np.median(r[:, 3])), 1),
           'detail_real': round(float(np.median(r[:, 4])), 1)}
    (s.products / 'textured/score.json').write_text(json.dumps(out, indent=2))
    sheet = np.vstack(shots)
    cv2.imwrite(str(s.products / 'textured/score.jpg'), cv2.resize(sheet, (1920, int(1920 * sheet.shape[0] / sheet.shape[1]))),
                [cv2.IMWRITE_JPEG_QUALITY, 90])
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
