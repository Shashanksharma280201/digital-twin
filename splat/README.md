# Photoreal twin: LiDAR + camera Gaussian splatting

These scripts turn a capture session into a photoreal, metrically scaled 3D twin. This is milestone M4 in the capture-stack design.

LiDAR and camera are fused at three points:

| Stage | LiDAR provides | Camera provides |
|---|---|---|
| Poses | Every camera pose comes from LiDAR-inertial odometry (FAST-LIO or GLIM) plus the LiDAR→camera extrinsic. No structure-from-motion is used, so the twin is in metres. | |
| Initialisation | The starting Gaussians are LiDAR map points | The colours of those points (`dtp colorize`, occlusion-aware) |
| Training | A depth loss pulls the rendered depth toward the LiDAR depth in every view | The photometric loss (L1 + SSIM) |

## Steps
```bash
dtp map SESSION                         # LiDAR map (after dtp traj)
dtp colorize SESSION                    # true-colour point cloud: products/map_rgb.laz
dtp colmap SESSION --min-translation 0.15 --min-rotation 5      # images + poses + coloured points
python splat/train_splat.py --data SESSION/products/colmap --out SESSION/products/splat --iters 20000
python splat/render_flythrough.py --splat SESSION/products/splat/point_cloud.ply \
       --data SESSION/products/colmap --out flythrough.mp4 --lift 2.5 --pitch -20
```

**Environment.** Use a separate venv, because PyTorch is about 4 GB:
```bash
uv venv -p python3.10 ~/dt_data/venvs/splat
uv pip install -p ~/dt_data/venvs/splat/bin/python torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cu124
uv pip install -p ~/dt_data/venvs/splat/bin/python "gsplat==1.5.3+pt24cu124" --index-url https://docs.gsplat.studio/whl/ \
    --extra-index-url https://pypi.org/simple --index-strategy unsafe-best-match
uv pip install -p ~/dt_data/venvs/splat/bin/python "numpy<2" scipy opencv-python-headless "laspy[lazrs]" packaging setuptools jaxtyping rich
```
This uses a prebuilt gsplat wheel, so there is no CUDA compile. On the Jetson, train on an x86 GPU machine instead.

## Viewing
- **Interactively:** open [SuperSplat](https://superspl.at/editor) and drag in `point_cloud.ply`. It is the standard 3DGS layout and runs locally in the browser, with no upload.
- **Video:** `flythrough.mp4`.
- **Side-by-side comparisons:** `renders/test_*.png` show ground truth | render on held-out views.

## Results: Blender sim, day 1, Mid-360 + ZED 2i at 5 Hz
- **Training data:** 417 images (364 train / 53 test) at 640×360.
- **Model:** 400k LiDAR-seeded Gaussians grew to 857k.
- **Training time:** 20k steps in 13.6 min on an RTX 4060 laptop.
- **Held-out views:** PSNR **39.1 dB**, SSIM **0.953**.

**Limits:**
- Views far from the walked path degrade into blur or "fog", for example near ceilings the scanner passed under.
- The sim is idealised: perfect calibration and sync, no motion blur, no exposure changes. Real data will score lower; typical real outdoor 3DGS scores are 22–28 dB.
