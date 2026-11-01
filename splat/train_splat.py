"""Train a photoreal Gaussian-splat digital twin from a dtp COLMAP export (LiDAR + camera).

LiDAR + camera fusion:
- poses come from LiDAR-inertial odometry (no SfM), so the splat is in metric map coordinates;
- Gaussians are initialised from the camera-coloured LiDAR map (points3D.txt);
- a LiDAR depth loss pulls rendered depth towards the LiDAR map in every view.

python splat/train_splat.py --data SESSION/products/colmap --out SESSION/products/splat [--iters 15000]

Outputs: point_cloud.ply (standard 3DGS layout, opens in SuperSplat / any 3DGS viewer),
renders/test_*.png (ground truth | render), novel/*.png (views off the walked path), metrics.json.
Needs: torch (CUDA), gsplat 1.5.x, numpy, opencv-python-headless.
"""
import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from gsplat import rasterization
from gsplat.strategy import DefaultStrategy

SH_C0 = 0.28209479177387814


def read_colmap_text(sparse):
    cams = {}
    for line in (sparse / 'cameras.txt').read_text().splitlines():
        if line and not line.startswith('#'):
            p = line.split()
            assert p[1] == 'PINHOLE', 'expected rectified PINHOLE camera'
            cams[int(p[0])] = dict(w=int(p[2]), h=int(p[3]), fx=float(p[4]), fy=float(p[5]), cx=float(p[6]), cy=float(p[7]))
    images = []
    lines = [l for l in (sparse / 'images.txt').read_text().splitlines() if not l.startswith('#')]
    for line in lines[::2]:
        p = line.split()
        if len(p) < 10:
            continue
        qw, qx, qy, qz, tx, ty, tz = map(float, p[1:8])
        R = quat_to_rot(qw, qx, qy, qz)
        viewmat = np.eye(4)
        viewmat[:3, :3], viewmat[:3, 3] = R, [tx, ty, tz]
        images.append(dict(viewmat=viewmat, cam=int(p[8]), name=p[9]))
    pts = np.loadtxt(sparse / 'points3D.txt', comments='#', usecols=range(7))
    return cams, images, pts[:, 1:4], pts[:, 4:7] / 255.0


def quat_to_rot(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def lidar_depth_maps(points, viewmats, K, w, h, device, chunk=16):
    """Sparse depth target per view: nearest LiDAR point per pixel (0 = no LiDAR)."""
    pts = torch.as_tensor(points, dtype=torch.float32, device=device)
    out = torch.zeros(len(viewmats), h, w, dtype=torch.float16)
    for s in range(0, len(viewmats), chunk):
        V = viewmats[s:s + chunk]
        pc = torch.einsum('cij,nj->cni', V[:, :3, :3], pts) + V[:, None, :3, 3]
        z = pc[..., 2]
        u = (pc[..., 0] / z * K[0, 0] + K[0, 2]).long()
        v = (pc[..., 1] / z * K[1, 1] + K[1, 2]).long()
        ok = (z > 0.2) & (z < 40) & (u >= 0) & (u < w) & (v >= 0) & (v < h)
        for c in range(len(V)):
            idx = v[c, ok[c]] * w + u[c, ok[c]]
            d = torch.full((h * w,), float('inf'), device=device)
            d.scatter_reduce_(0, idx, z[c, ok[c]], reduce='amin')
            d[torch.isinf(d)] = 0
            out[s + c] = d.view(h, w).half().cpu()
    return out


def ssim(a, b, win=11):
    """a, b: [B,3,H,W] in [0,1]."""
    g = torch.exp(-(torch.arange(win, device=a.device) - win // 2) ** 2 / (2 * 1.5 ** 2))
    g = (g / g.sum())
    k = (g[:, None] @ g[None, :]).expand(3, 1, win, win)
    mu_a, mu_b = F.conv2d(a, k, groups=3), F.conv2d(b, k, groups=3)
    s_aa = F.conv2d(a * a, k, groups=3) - mu_a ** 2
    s_bb = F.conv2d(b * b, k, groups=3) - mu_b ** 2
    s_ab = F.conv2d(a * b, k, groups=3) - mu_a * mu_b
    c1, c2 = 0.01 ** 2, 0.03 ** 2
    return (((2 * mu_a * mu_b + c1) * (2 * s_ab + c2)) / ((mu_a ** 2 + mu_b ** 2 + c1) * (s_aa + s_bb + c2))).mean()


def knn_scale(points, k=4):
    from scipy.spatial import cKDTree
    d, _ = cKDTree(points).query(points, k=k)
    return np.sqrt((d[:, 1:] ** 2).mean(1)).clip(0.005, 0.5)


def save_ply(path, params):
    means = params['means'].detach().cpu().numpy()
    n = len(means)
    sh0 = params['sh0'].detach().cpu().numpy().reshape(n, -1)
    shN = params['shN'].detach().transpose(1, 2).cpu().numpy().reshape(n, -1)
    cols = [means, np.zeros_like(means), sh0, shN, params['opacities'].detach().cpu().numpy()[:, None],
            params['scales'].detach().cpu().numpy(), params['quats'].detach().cpu().numpy()]
    names = ['x', 'y', 'z', 'nx', 'ny', 'nz'] + [f'f_dc_{i}' for i in range(sh0.shape[1])] + \
        [f'f_rest_{i}' for i in range(shN.shape[1])] + ['opacity'] + [f'scale_{i}' for i in range(3)] + \
        [f'rot_{i}' for i in range(4)]
    data = np.concatenate(cols, 1).astype(np.float32)
    header = 'ply\nformat binary_little_endian 1.0\nelement vertex %d\n' % n
    header += ''.join(f'property float {nm}\n' for nm in names) + 'end_header\n'
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(data.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', type=Path, required=True, help='dtp colmap export dir')
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--iters', type=int, default=15000)
    ap.add_argument('--downscale', type=int, default=2, help='train at 1/N resolution')
    ap.add_argument('--max-init', type=int, default=400_000, help='initial Gaussians from LiDAR')
    ap.add_argument('--depth-weight', type=float, default=0.05, help='LiDAR depth loss weight (0 = off)')
    ap.add_argument('--sh-degree', type=int, default=3)
    ap.add_argument('--test-every', type=int, default=8)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)
    dev = torch.device('cuda')
    out = args.out
    (out / 'renders').mkdir(parents=True, exist_ok=True)
    (out / 'novel').mkdir(exist_ok=True)

    cams, images, pts, rgb = read_colmap_text(args.data / 'sparse' / '0')
    cam = cams[1]
    w, h = cam['w'] // args.downscale, cam['h'] // args.downscale
    K = torch.tensor([[cam['fx'] / args.downscale, 0, cam['cx'] / args.downscale],
                      [0, cam['fy'] / args.downscale, cam['cy'] / args.downscale], [0, 0, 1]], device=dev)
    viewmats = torch.tensor(np.stack([im['viewmat'] for im in images]), dtype=torch.float32, device=dev)
    imgs = []
    for im in images:
        bgr = cv2.imread(str(args.data / 'images' / im['name']))
        imgs.append(cv2.resize(bgr, (w, h), interpolation=cv2.INTER_AREA)[..., ::-1])
    imgs = torch.as_tensor(np.stack(imgs))  # uint8 [N,H,W,3] on CPU
    test = np.arange(len(images)) % args.test_every == 0
    train_idx, test_idx = np.flatnonzero(~test), np.flatnonzero(test)
    print(f'{len(images)} images ({len(train_idx)} train / {len(test_idx)} test) at {w}x{h}, {len(pts):,} LiDAR points')

    depth = lidar_depth_maps(pts, viewmats, K, w, h, dev) if args.depth_weight > 0 else None
    sky = imgs[:, : h // 10].float().mean((0, 1, 2)).to(dev) / 255.0  # background where nothing is mapped

    init = rng.choice(len(pts), min(args.max_init, len(pts)), replace=False)
    p0, c0 = pts[init], rgb[init]
    cam_centers = torch.linalg.inv(viewmats)[:, :3, 3].cpu().numpy()
    scene_scale = float(np.linalg.norm(cam_centers - cam_centers.mean(0), axis=1).max() * 1.1)
    n = len(p0)
    nsh = (args.sh_degree + 1) ** 2
    params = torch.nn.ParameterDict({
        'means': torch.nn.Parameter(torch.tensor(p0, dtype=torch.float32)),
        'scales': torch.nn.Parameter(torch.log(torch.tensor(knn_scale(p0), dtype=torch.float32))[:, None].repeat(1, 3)),
        'quats': torch.nn.Parameter(F.normalize(torch.rand(n, 4), dim=-1)),
        'opacities': torch.nn.Parameter(torch.logit(torch.full((n,), 0.5))),
        'sh0': torch.nn.Parameter(torch.tensor((c0 - 0.5) / SH_C0, dtype=torch.float32)[:, None, :]),
        'shN': torch.nn.Parameter(torch.zeros(n, nsh - 1, 3)),
    }).to(dev)
    lrs = {'means': 1.6e-4 * scene_scale, 'scales': 5e-3, 'quats': 1e-3, 'opacities': 5e-2,
           'sh0': 2.5e-3, 'shN': 2.5e-3 / 20}
    opts = {k: torch.optim.Adam([{'params': params[k], 'lr': lr, 'name': k}], eps=1e-15) for k, lr in lrs.items()}
    means_decay = 0.01 ** (1.0 / args.iters)
    strategy = DefaultStrategy(refine_stop_iter=int(args.iters * 0.6), reset_every=3000, verbose=False)
    strategy.check_sanity(params, opts)
    state = strategy.initialize_state(scene_scale=scene_scale)

    def render(view_idx, sh_deg):
        V = viewmats[view_idx]
        colors = torch.cat([params['sh0'], params['shN']], 1)
        out, alpha, info = rasterization(
            params['means'], F.normalize(params['quats'], dim=-1), torch.exp(params['scales']),
            torch.sigmoid(params['opacities']), colors, V, K[None].expand(len(V), 3, 3), w, h,
            sh_degree=sh_deg, render_mode='RGB+ED', backgrounds=sky[None].expand(len(V), 3), packed=False)
        return out[..., :3].clamp(0, 1), out[..., 3], alpha, info

    t0 = time.time()
    for step in range(args.iters):
        i = int(rng.choice(train_idx))
        sh_deg = min(args.sh_degree, step // 1000)
        rgb_r, d_r, _, info = render(torch.tensor([i], device=dev), sh_deg)
        gt = imgs[i].to(dev).float()[None] / 255.0
        l1 = (rgb_r - gt).abs().mean()
        loss = 0.8 * l1 + 0.2 * (1 - ssim(rgb_r.permute(0, 3, 1, 2), gt.permute(0, 3, 1, 2)))
        if depth is not None:
            dt = depth[i].to(dev).float()[None]
            m = dt > 0
            if m.any():
                loss = loss + args.depth_weight * ((d_r - dt).abs()[m] / dt[m]).mean()
        strategy.step_pre_backward(params, opts, state, step, info)
        loss.backward()
        for o in opts.values():
            o.step()
            o.zero_grad(set_to_none=True)
        for g in opts['means'].param_groups:
            g['lr'] *= means_decay
        strategy.step_post_backward(params, opts, state, step, info, packed=False)
        if step % 500 == 0 or step == args.iters - 1:
            print(f'step {step:6d}  loss {loss.item():.4f}  gaussians {len(params["means"]):,}  '
                  f'{time.time() - t0:.0f}s', flush=True)

    # Evaluation on held-out views + side-by-side renders.
    psnrs, ssims = [], []
    with torch.no_grad():
        for k, i in enumerate(test_idx):
            rgb_r, _, _, _ = render(torch.tensor([int(i)], device=dev), args.sh_degree)
            gt = imgs[i].to(dev).float()[None] / 255.0
            psnrs.append(float(-10 * torch.log10(((rgb_r - gt) ** 2).mean())))
            ssims.append(float(ssim(rgb_r.permute(0, 3, 1, 2), gt.permute(0, 3, 1, 2))))
            if k % 5 == 0:
                pair = torch.cat([gt[0], rgb_r[0]], 1).cpu().numpy()
                cv2.imwrite(str(out / 'renders' / f'test_{i:04d}.png'), (pair[..., ::-1] * 255).astype(np.uint8))
        # Novel views: lift the camera 1.5 m and pitch down, off the walked path.
        lift = torch.eye(4, device=dev)
        lift[:3, :3] = torch.tensor(cv2.Rodrigues(np.array([-0.35, 0.0, 0.0]))[0], dtype=torch.float32, device=dev)
        for k, i in enumerate(train_idx[:: max(1, len(train_idx) // 6)]):
            c2w = torch.linalg.inv(viewmats[int(i)])
            c2w[:3, 3] += torch.tensor([0.0, 0.0, 1.5], device=dev)
            V = lift @ torch.linalg.inv(c2w)
            old = viewmats[int(i)].clone()
            viewmats[int(i)] = V
            rgb_r, _, _, _ = render(torch.tensor([int(i)], device=dev), args.sh_degree)
            viewmats[int(i)] = old
            cv2.imwrite(str(out / 'novel' / f'novel_{k:02d}.png'),
                        (rgb_r[0].cpu().numpy()[..., ::-1] * 255).astype(np.uint8))

    save_ply(out / 'point_cloud.ply', params)
    metrics = {'test_psnr_db': float(np.mean(psnrs)), 'test_ssim': float(np.mean(ssims)),
               'gaussians': int(len(params['means'])), 'iters': args.iters, 'train_views': int(len(train_idx)),
               'test_views': int(len(test_idx)), 'resolution': [w, h], 'depth_weight': args.depth_weight,
               'minutes': round((time.time() - t0) / 60, 1)}
    (out / 'metrics.json').write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
