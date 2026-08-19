#!/usr/bin/env bash
set -euo pipefail

# Install the tested cuRobo GPU backend into the active Python environment.
# CUDA 12.8 is required for Blackwell (compute capability 12.0) GPUs.
CUDA_ROOT="${CUDA_HOME:-/usr/local/cuda-12.8}"
CUROBO_TAG="${CUROBO_VERSION:-v0.8.0}"
CUROBO_SOURCE="${CUROBO_SOURCE_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/spark/curobo-${CUROBO_TAG}}"

if [[ ! -x "${CUDA_ROOT}/bin/nvcc" ]]; then
    echo "CUDA compiler not found at ${CUDA_ROOT}/bin/nvcc" >&2
    exit 1
fi

export CUDA_HOME="${CUDA_ROOT}"
export PATH="${CUDA_ROOT}/bin:${PATH}"
export TORCH_CUDA_ARCH_LIST="${TORCH_CUDA_ARCH_LIST:-12.0}"

python -m pip install --upgrade ninja 'cuda-core[cu12]>=0.7,<0.8' 'nvidia-cuda-runtime-cu12>=12,<13'

if [[ ! -d "${CUROBO_SOURCE}/.git" ]]; then
    mkdir -p "$(dirname "${CUROBO_SOURCE}")"
    git clone --branch "${CUROBO_TAG}" --depth 1 https://github.com/NVlabs/curobo.git "${CUROBO_SOURCE}"
fi

# Editable installation is intentional: cuRobo's runtime-compiled CUDA kernels
# and bundled robot/config assets must remain available together.
python -m pip install --editable "${CUROBO_SOURCE}" --no-build-isolation

python - <<'PY'
import torch
from spark_robot.kinematics.curobo import CuroboIK

if not CuroboIK.available():
    raise SystemExit("cuRobo installed, but CUDA is not available to PyTorch")
print(f"cuRobo ready: torch={torch.__version__}, gpu={torch.cuda.get_device_name(0)}")
PY
