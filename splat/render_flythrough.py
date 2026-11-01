"""Render a fly-through video of a trained splat twin along (an offset of) the capture path.

python splat/render_flythrough.py --splat SESSION/products/splat/point_cloud.ply \
    --data SESSION/products/colmap --out flythrough.mp4 [--lift 2.0] [--pitch -20] [--frames 600]

The camera follows the recorded image poses (smoothed), raised by --lift metres and pitched by
--pitch degrees, so most of the video is from viewpoints the scanner never occupied.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from gsplat import rasterization
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation, Slerp

from train_splat import read_colmap_text


def load_ply(path, device):
    with open(path, 'rb') as f:
        names, n = [], 0
        while (line := f.readline().decode().strip()) != 'end_header':
            p = line.split()
            if p[:2] == ['element', 'vertex']:
                n = int(p[2])
            elif p[0] == 'property':
                names.append(p[2])
        data = np.frombuffer(f.read(n * 4 * len(names)), dtype=np.float32).reshape(n, len(names))
    col = {k: i for i, k in enumerate(names)}
    take = lambda prefix: data[:, [col[k] for k in names if k.startswith(prefix)]]
    sh0, rest = take('f_dc_'), take('f_rest_')
    k_rest = rest.shape[1] // 3
    sh = np.concatenate([sh0[:, None, :], rest.reshape(n, 3, k_rest).transpose(0, 2, 1)], 1)
    t = lambda a: torch.tensor(np.ascontiguousarray(a), device=device)
    return dict(means=t(data[:, :3]), scales=torch.exp(t(take('scale_'))),
                quats=F.normalize(t(take('rot_')), dim=-1),
                opacities=torch.sigmoid(t(data[:, col['opacity']])), sh=t(sh),
                degree=int(round(np.sqrt(sh.shape[1]))) - 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--splat', type=Path, required=True)
    ap.add_argument('--data', type=Path, required=True, help='dtp colmap export (for camera + path)')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--lift', type=float, default=2.0)
    ap.add_argument('--pitch', type=float, default=-20.0, help='degrees, negative looks down')
    ap.add_argument('--frames', type=int, default=600)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--scale', type=float, default=1.0, help='output resolution factor vs camera')
    args = ap.parse_args()

    dev = torch.device('cuda')
    g = load_ply(args.splat, dev)
    cams, images, _, _ = read_colmap_text(args.data / 'sparse' / '0')
    cam = cams[1]
    w, h = int(cam['w'] * args.scale), int(cam['h'] * args.scale)
    K = torch.tensor([[cam['fx'] * args.scale, 0, cam['cx'] * args.scale],
                      [0, cam['fy'] * args.scale, cam['cy'] * args.scale], [0, 0, 1]], device=dev)

    c2w = np.stack([np.linalg.inv(im['viewmat']) for im in images])
    s = np.linspace(0, 1, len(c2w))
    pos = gaussian_filter1d(c2w[:, :3, 3], sigma=4, axis=0)
    rots = Slerp(s, Rotation.from_matrix(c2w[:, :3, :3]))
    q = np.linspace(0, 1, args.frames)
    P = np.stack([np.interp(q, s, pos[:, i]) for i in range(3)], 1)
    R = rots(q).as_matrix()
    pitch = Rotation.from_euler('x', args.pitch, degrees=True).as_matrix()  # about camera x (OpenCV)
    sky = torch.tensor([0.62, 0.72, 0.85], device=dev)

    writer = cv2.VideoWriter(str(args.out), cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (w, h))
    with torch.no_grad():
        for k in range(args.frames):
            M = np.eye(4)
            M[:3, :3] = R[k] @ pitch
            M[:3, 3] = P[k] + [0.0, 0.0, args.lift]
            V = torch.tensor(np.linalg.inv(M), dtype=torch.float32, device=dev)[None]
            img, _, _ = rasterization(g['means'], g['quats'], g['scales'], g['opacities'], g['sh'], V, K[None],
                                      w, h, sh_degree=g['degree'], backgrounds=sky[None],
                                      packed=False)  # gsplat 1.5.3 rejects backgrounds when packed
            frame = (img[0].clamp(0, 1).cpu().numpy()[..., ::-1] * 255).astype(np.uint8)
            writer.write(frame)
            if k == args.frames // 2:
                cv2.imwrite(str(args.out.with_suffix('.png')), frame)
    writer.release()
    print(f'wrote {args.out} ({args.frames} frames, {w}x{h}) and {args.out.with_suffix(".png")}')


if __name__ == '__main__':
    main()
