#!/usr/bin/env bash
# Installs the offline Python tools (dt_pipeline `dtp`, sim/dt_sim `dt_sim`) into a venv and tests them.
# Usage: 08_install_python_tools.sh [--venv DIR] [--no-tests]
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "08 Python tools (dtp, dt_sim)"
venv="${DT_DATA}/venvs/dt"; tests=1
while [ $# -gt 0 ]; do
  case "$1" in
    --venv) venv="$2"; shift ;;
    --no-tests) tests=0 ;;
    *) die "unknown option $1" ;;
  esac
  shift
done

if ! have uv && [ ! -x "${HOME}/.local/bin/uv" ]; then
  log "installing uv (Python package manager)"
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi
export PATH="${HOME}/.local/bin:${PATH}"
have uv || die "uv not on PATH after install"

[ -x "${venv}/bin/python" ] || uv venv -p /usr/bin/python3.10 "$venv"
log "installing dt_pipeline + dt_sim (editable) into ${venv}"
uv pip install -p "${venv}/bin/python" -e "${DT_ROOT}/pipeline[test]" -e "${DT_ROOT}/sim/dt_sim" pytest

# PyTorch for the GPU steps. Jetson: link JetPack's NVIDIA build (pip has no CUDA aarch64 wheel);
# x86-64 with an NVIDIA driver: the CUDA 12.4 wheel. The ZED Python API (pyzed) is linked the same way.
site="$("${venv}/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
jp="/usr/local/lib/python3.10/dist-packages"
if [ -d "${jp}/torch" ]; then
  log "linking JetPack PyTorch into the venv"
  for d in "${jp}"/torch "${jp}"/torchgen "${jp}"/functorch "${jp}"/torch-*.dist-info; do
    [ -e "$d" ] && ln -sfn "$d" "${site}/$(basename "$d")"
  done
  uv pip install -p "${venv}/bin/python" filelock fsspec jinja2 networkx sympy typing-extensions
elif have nvidia-smi; then
  log "installing PyTorch (CUDA 12.4 wheel)"
  uv pip install -p "${venv}/bin/python" "torch==2.4.1" --index-url https://download.pytorch.org/whl/cu124 \
    --extra-index-url https://pypi.org/simple
else
  warn "no NVIDIA GPU: GPU steps (dtp fuse / colorize) need PyTorch; install the CPU wheel to run them slowly"
fi
for zp in /usr/local/lib/python3.10/dist-packages/pyzed "${HOME}/.local/lib/python3.10/site-packages/pyzed"; do
  [ -d "$zp" ] && { ln -sfn "$zp" "${site}/pyzed"; break; }
done
"${venv}/bin/python" -c 'import torch; print("torch", torch.__version__, "cuda", torch.cuda.is_available())' || warn "PyTorch not importable"

log "checking aarch64 wheels: open3d ray casting, small_gicp, rosbags"
"${venv}/bin/python" - <<'EOF'
import numpy as np, open3d as o3d, small_gicp, rosbags  # noqa: F401
scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(o3d.geometry.TriangleMesh.create_box(2, 2, 2)))
rays = o3d.core.Tensor(np.array([[1, 1, 1, 1, 0, 0]], np.float32))
hit = scene.cast_rays(rays)["t_hit"].numpy()[0]
assert abs(hit - 1.0) < 1e-4, hit
print(f"open3d {o3d.__version__} ray casting ok")
EOF

if [ "$tests" -eq 1 ]; then
  log "running pipeline + simulator tests"
  "${venv}/bin/python" -m pytest -q "${DT_ROOT}/pipeline/tests" "${DT_ROOT}/sim/dt_sim/tests"
fi
ok "Python tools ready: ${venv}/bin/dtp, ${venv}/bin/dt_sim (env.sh adds them to PATH)"
