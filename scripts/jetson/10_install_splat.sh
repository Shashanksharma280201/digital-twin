#!/usr/bin/env bash
# OPTIONAL / EXPERIMENTAL: Gaussian-splat training on the Orin (splat/train_splat.py).
# Recommended path is to train on an x86 RTX PC (prebuilt gsplat, ~15 min). On Jetson this uses
# NVIDIA's JetPack PyTorch wheel and compiles gsplat from source for sm_87 (~30-60 min).
# Usage: 10_install_splat.sh [--venv DIR]
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "10 Gaussian splatting (experimental on Jetson)"
venv="${DT_DATA}/venvs/splat"
[ "${1:-}" = "--venv" ] && venv="$2"
require_l4t
export PATH="${HOME}/.local/bin:${PATH}"
have uv || die "uv missing (run 08 first)"

case "$L4T_VERSION" in
  36.3) torch_url="https://developer.download.nvidia.com/compute/redist/jp/v60/pytorch/torch-2.4.0a0+07cecf4168.nv24.05.14710581-cp310-cp310-linux_aarch64.whl" ;;
  *)    torch_url="https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl" ;;
esac
apt_install libopenblas-dev libjpeg-dev zlib1g-dev
# torch >= nv24.06 links cuSPARSELt; it is in the CUDA repo that JetPack configures.
dpkg -s libcusparselt0 >/dev/null 2>&1 || sudo apt-get install -y libcusparselt0 libcusparselt-dev \
  || warn "libcusparselt not installable from apt; if 'import torch' fails, install cuSPARSELt from developer.nvidia.com"

[ -x "${venv}/bin/python" ] || uv venv -p /usr/bin/python3.10 "$venv"
log "installing JetPack PyTorch: ${torch_url##*/}"
uv pip install -p "${venv}/bin/python" "numpy<2" "$torch_url"
"${venv}/bin/python" -c "import torch; assert torch.cuda.is_available(); print('torch', torch.__version__, torch.cuda.get_device_name(0))" \
  || die "PyTorch cannot see the GPU"

log "compiling gsplat 1.5.3 for sm_87 (slow)"
uv pip install -p "${venv}/bin/python" setuptools wheel ninja packaging jaxtyping rich scipy opencv-python-headless "laspy[lazrs]"
TORCH_CUDA_ARCH_LIST="8.7" MAX_JOBS=4 uv pip install -p "${venv}/bin/python" --no-build-isolation "gsplat==1.5.3"
"${venv}/bin/python" -c "from gsplat.cuda._backend import _C; import gsplat; print('gsplat', gsplat.__version__, 'cuda backend ok')" \
  || die "gsplat CUDA backend failed to load"
ok "splat environment ready: ${venv}/bin/python splat/train_splat.py ..."
