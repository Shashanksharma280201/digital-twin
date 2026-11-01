#!/usr/bin/env bash
# Run a sim/blender script headless with the NVIDIA EGL device forced (fast EEVEE).
# Usage: sim/blender/bl.sh <script.py> [script args...]
#        sim/blender/bl.sh --blend FILE.blend <script.py> [args...]   (open a .blend first)
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
blender_bin="${BLENDER:-blender}"
blend=()
if [ "${1:-}" = "--blend" ]; then blend=("$2"); shift 2; fi
script="$1"; shift
[ -f "$script" ] || script="${here}/${script}"
if [ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]; then
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
  export DRI_PRIME=1   # without these, hybrid laptops render EEVEE on the iGPU (~6x slower)
fi
if [ ${#blend[@]} -eq 0 ]; then
  exec "$blender_bin" -b --factory-startup -P "$script" -- "$@"
else
  exec "$blender_bin" -b --factory-startup "${blend[0]}" -P "$script" -- "$@"
fi
