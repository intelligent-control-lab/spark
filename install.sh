#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST_OS="$(uname -s)"
HOST_ARCH="$(uname -m)"

ENV_MANAGER="conda"
ENV_NAME=""
ENV_PATH=""
PYTHON_VERSION="3.10"
PYTHON_VERSION_EXPLICIT=false
PROFILE="core"
INSTALL_ROS=false
INSTALL_TORCH=false
INSTALL_LEARNED=false
INSTALL_UNITREE_SDK=false
INSTALL_DEV=false

# Versions validated together by the SPARK Isaac compatibility check.
ISAAC_SIM_VERSION="${ISAAC_SIM_VERSION:-6.0.0.1}"
ISAAC_LAB_REF="${ISAAC_LAB_REF:-95b6ed797eac1d59193402421913ee86fb29b30c}"
CUROBO_REF="${CUROBO_REF:-8e734f3ced1df898990bcd92de40abce475907db}"
PYTORCH_VERSION="${PYTORCH_VERSION:-2.11.0}"
TORCHVISION_VERSION="${TORCHVISION_VERSION:-0.26.0}"
PYTORCH_CUDA_INDEX="${PYTORCH_CUDA_INDEX:-https://download.pytorch.org/whl/cu128}"
UNITREE_SDK_REF="${UNITREE_SDK_REF:-a035adeaa6f8ea171bef9a43e8477abb87a0b35e}"

usage() {
    cat <<'EOF'
Usage:
  ./install.sh --name ENV_NAME [options]
  ./install.sh --env-manager uv [--path PATH] [options]

Options:
  --name NAME                 Conda environment name (required for Conda).
  --env-manager conda|uv|venv Environment manager (default: conda).
  --path PATH                 Virtual-environment path (default: .venv).
  --profile core|mujoco|isaac Installation profile (default: core).
  --python VERSION            Python version (default: 3.10; Isaac: 3.12).
  --torch                     Install PyTorch. Isaac selects its tested CUDA
                              build automatically; other profiles use PyPI.
  --learned                   Install learned-policy runtimes (implies --torch).
  --unitree-sdk               Install CycloneDDS and Unitree SDK2 for hardware.
  --ros                       Install ROS Python helper packages. ROS itself
                              remains provided by the host OS installation.
  --dev                       Install test and interactive development tools.
  -h, --help                  Show this help.

Profiles:
  core    Numerical control, estimation, planning, CPU kinematics, and plots.
  mujoco  The core profile plus MuJoCo and OpenCV support.
  isaac   The mujoco profile plus Isaac Sim, Isaac Lab, CUDA PyTorch, cuRobo,
          and learned-policy runtimes.

Platforms:
  Linux x86_64       core, MuJoCo, and the qualified Isaac/hardware options.
  macOS arm64 12+    core and MuJoCo; optional CPU/MPS learned runtimes.
  macOS x86_64       core only (MuJoCo >=3.4 has no Intel macOS wheel).

Environment overrides for the Isaac profile:
  ISAAC_SIM_VERSION, ISAAC_LAB_REF, CUROBO_REF, PYTORCH_VERSION,
  TORCHVISION_VERSION, PYTORCH_CUDA_INDEX, UNITREE_SDK_REF
EOF
}

fail() {
    echo "Error: $*" >&2
    exit 1
}

run_pip() {
    # A sourced ROS installation can add its own site-packages through
    # PYTHONPATH. Do not let those host packages participate in this env's
    # resolver; ROS bindings remain available when users source ROS at runtime.
    env -u PYTHONPATH "$PYTHON_BIN" -m pip "$@"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --name)
            [[ $# -ge 2 ]] || fail "--name requires a value"
            ENV_NAME="$2"
            shift 2
            ;;
        --env-manager)
            [[ $# -ge 2 ]] || fail "--env-manager requires a value"
            ENV_MANAGER="$2"
            shift 2
            ;;
        --path)
            [[ $# -ge 2 ]] || fail "--path requires a value"
            ENV_PATH="$2"
            shift 2
            ;;
        --profile)
            [[ $# -ge 2 ]] || fail "--profile requires a value"
            PROFILE="$2"
            shift 2
            ;;
        --python)
            [[ $# -ge 2 ]] || fail "--python requires a value"
            PYTHON_VERSION="$2"
            PYTHON_VERSION_EXPLICIT=true
            shift 2
            ;;
        --torch)
            INSTALL_TORCH=true
            shift
            ;;
        --learned)
            INSTALL_LEARNED=true
            INSTALL_TORCH=true
            shift
            ;;
        --unitree-sdk)
            INSTALL_UNITREE_SDK=true
            shift
            ;;
        --ros)
            INSTALL_ROS=true
            shift
            ;;
        --dev)
            INSTALL_DEV=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            fail "unknown option: $1"
            ;;
    esac
done

case "$PROFILE" in
    core|mujoco|isaac) ;;
    *) fail "--profile must be 'core', 'mujoco', or 'isaac'" ;;
esac

if [[ ! "$PYTHON_VERSION" =~ ^([0-9]+)\.([0-9]+)(\..*)?$ ]]; then
    fail "--python must contain a major and minor version, for example 3.10"
fi
PYTHON_MAJOR="${BASH_REMATCH[1]}"
PYTHON_MINOR="${BASH_REMATCH[2]}"
if (( PYTHON_MAJOR < 3 || (PYTHON_MAJOR == 3 && PYTHON_MINOR < 10) )); then
    fail "SPARK requires Python >=3.10"
fi

case "$HOST_OS" in
    Linux) ;;
    Darwin)
        case "$HOST_ARCH" in
            arm64|x86_64) ;;
            *) fail "macOS architecture $HOST_ARCH is not supported" ;;
        esac
        if [[ "$PROFILE" == "isaac" ]]; then
            fail "the Isaac profile requires Linux x86_64 with an NVIDIA GPU"
        fi
        if [[ "$INSTALL_UNITREE_SDK" == true ]]; then
            fail "the released Unitree SDK hardware integration requires Linux"
        fi
        if [[ "$PROFILE" == "mujoco" && "$HOST_ARCH" != "arm64" ]]; then
            fail "the MuJoCo profile on macOS requires native Apple Silicon; use --profile core on Intel Macs"
        fi
        if [[ "$HOST_ARCH" == "arm64" ]]; then
            MACOS_VERSION="$(sw_vers -productVersion)"
            MACOS_MAJOR="${MACOS_VERSION%%.*}"
            if (( MACOS_MAJOR < 12 )); then
                fail "Apple Silicon installation requires macOS 12 or newer"
            fi
        fi
        ;;
    *) fail "the installer supports Linux and macOS; found $HOST_OS" ;;
esac

if [[ "$PROFILE" == "isaac" ]]; then
    if [[ "$PYTHON_VERSION_EXPLICIT" == false ]]; then
        PYTHON_VERSION="3.12"
    elif [[ "$PYTHON_VERSION" != "3.12" && "$PYTHON_VERSION" != 3.12.* ]]; then
        fail "the release-qualified Isaac profile requires Python 3.12"
    fi
    INSTALL_TORCH=true
    INSTALL_LEARNED=true
fi

case "$ENV_MANAGER" in
    conda)
        [[ -n "$ENV_NAME" ]] || fail "--name is required when using Conda"
        command -v conda >/dev/null 2>&1 || fail "Conda is not installed"
        CONDA_BASE="$(conda info --base)"
        # shellcheck disable=SC1091
        source "$CONDA_BASE/etc/profile.d/conda.sh"
        if [[ "$ENV_NAME" == "base" ]]; then
            ENV_PREFIX="$CONDA_BASE"
        else
            ENV_PREFIX="$CONDA_BASE/envs/$ENV_NAME"
        fi
        if [[ -x "$ENV_PREFIX/bin/python" ]]; then
            echo "Using existing Conda environment: $ENV_NAME"
        else
            echo "Creating Conda environment: $ENV_NAME (Python $PYTHON_VERSION)"
            conda create --name "$ENV_NAME" "python=$PYTHON_VERSION" pip -y
        fi
        conda activate "$ENV_NAME"
        PYTHON_BIN="$CONDA_PREFIX/bin/python"
        ENV_ROOT="$CONDA_PREFIX"
        ;;
    uv)
        command -v uv >/dev/null 2>&1 || fail "uv is not installed"
        ENV_PATH="${ENV_PATH:-$REPO_ROOT/.venv}"
        if [[ ! -x "$ENV_PATH/bin/python" ]]; then
            echo "Creating uv environment: $ENV_PATH (Python $PYTHON_VERSION)"
            uv venv --python "$PYTHON_VERSION" --seed "$ENV_PATH"
        else
            echo "Using existing uv environment: $ENV_PATH"
        fi
        PYTHON_BIN="$ENV_PATH/bin/python"
        ENV_ROOT="$(cd "$ENV_PATH" && pwd)"
        ;;
    venv)
        ENV_PATH="${ENV_PATH:-$REPO_ROOT/.venv}"
        PYTHON_COMMAND="python$PYTHON_VERSION"
        command -v "$PYTHON_COMMAND" >/dev/null 2>&1 || fail "$PYTHON_COMMAND is not installed"
        if [[ ! -x "$ENV_PATH/bin/python" ]]; then
            echo "Creating venv: $ENV_PATH"
            "$PYTHON_COMMAND" -m venv "$ENV_PATH"
        else
            echo "Using existing venv: $ENV_PATH"
        fi
        PYTHON_BIN="$ENV_PATH/bin/python"
        ENV_ROOT="$(cd "$ENV_PATH" && pwd)"
        ;;
    *)
        fail "--env-manager must be conda, uv, or venv"
        ;;
esac

echo "Python: $($PYTHON_BIN --version)"
"$PYTHON_BIN" - <<'PY'
import sys

if sys.version_info < (3, 10):
    raise SystemExit(
        f"SPARK requires Python >=3.10; found {sys.version.split()[0]}"
    )
PY
if [[ "$HOST_OS" == "Darwin" && "$PROFILE" == "mujoco" ]]; then
    PYTHON_ARCH="$($PYTHON_BIN -c 'import platform; print(platform.machine())')"
    if [[ "$PYTHON_ARCH" != "arm64" ]]; then
        fail "the macOS MuJoCo profile requires a native arm64 Python environment, not $PYTHON_ARCH"
    fi
fi
run_pip install --upgrade pip wheel

if [[ "$PROFILE" == "isaac" ]]; then
    if [[ "$HOST_OS" != "Linux" || "$HOST_ARCH" != "x86_64" ]]; then
        fail "the tested Isaac profile currently requires Linux x86_64"
    fi
    GLIBC_OUTPUT="$(ldd --version 2>&1)"
    GLIBC_FIRST_LINE="${GLIBC_OUTPUT%%$'\n'*}"
    GLIBC_VERSION="${GLIBC_FIRST_LINE##* }"
    "$PYTHON_BIN" - "$GLIBC_VERSION" <<'PY'
import sys
from packaging.version import Version
if Version(sys.argv[1]) < Version("2.35"):
    raise SystemExit(
        "Isaac Sim pip packages require glibc >= 2.35. "
        "Use Ubuntu 22.04+ or an NVIDIA Isaac container."
    )
PY

    echo "Installing CUDA PyTorch $PYTORCH_VERSION"
    run_pip install --upgrade \
        "torch==$PYTORCH_VERSION" "torchvision==$TORCHVISION_VERSION" \
        --index-url "$PYTORCH_CUDA_INDEX"

    echo "Installing Isaac Sim $ISAAC_SIM_VERSION"
    run_pip install --upgrade \
        "isaacsim[all,extscache]==$ISAAC_SIM_VERSION" \
        --extra-index-url https://pypi.nvidia.com \
        --pre
fi

echo "Installing SPARK runtime dependencies"
runtime_packages=(
    "numpy>=2,<3"
    "scipy"
    "casadi"
    "pyyaml"
    "osqp"
    "proxsuite"
    "tensorboardX"
    "scikit-image"
    "matplotlib"
    "meshcat"
    "pin>=4.1,<5"
)
if [[ "$PROFILE" == "isaac" ]]; then
    # Keep Isaac Sim's compatible numeric/runtime versions explicit. Coverage
    # is the documented exception to NVIDIA's otherwise exact pins.
    runtime_packages+=(
        "numba==0.63.1"
        "llvmlite==0.46.0"
        "psutil==5.9.8"
        "typing_extensions==4.12.2"
        # Isaac Sim pins coverage 7.4.4, but that release lacks the Tracer API
        # used by its simultaneously pinned Numba 0.63.1. The newer coverage
        # release is required for SPARK/Numba imports to function on Python 3.12.
        "coverage==7.15.2"
        "prettytable>=3.3"
    )
else
    runtime_packages+=("numba")
fi
if [[ "$PROFILE" == "mujoco" || "$PROFILE" == "isaac" ]]; then
    runtime_packages+=("mujoco>=3.4" "opencv-python" "imageio")
fi
if [[ "$INSTALL_LEARNED" == true ]]; then
    runtime_packages+=("onnx" "onnxruntime" "onnx2torch" "tensorboard")
fi
if [[ "$INSTALL_DEV" == true ]]; then
    runtime_packages+=(
        "pytest<9"
        "ipdb"
        "build"
        "twine"
        # Twine 7 accepts Rich 14+, while cuRobo's Viser dependency currently
        # rejects Rich 15. Keep --dev composable with learned/Isaac profiles.
        "rich>=14.3.3,<15"
        "ruff==0.16.3"
    )
fi
run_pip install "${runtime_packages[@]}"

if [[ "$INSTALL_UNITREE_SDK" == true ]]; then
    # The SDK pins an old source-only CycloneDDS binding that does not build on
    # supported modern Python versions. Version 11 provides compatible wheels
    # and preserves the API used by SPARK. The SDK itself must be editable because
    # its published package manifest omits robot subpackages from wheels.
    run_pip install "cyclonedds==11.0.1"
    UNITREE_SDK_ROOT="$ENV_ROOT/src/unitree_sdk2_python"
    if [[ ! -d "$UNITREE_SDK_ROOT/.git" ]]; then
        mkdir -p "$ENV_ROOT/src"
        git init "$UNITREE_SDK_ROOT"
        git -C "$UNITREE_SDK_ROOT" remote add origin \
            https://github.com/unitreerobotics/unitree_sdk2_python.git
    fi
    git -C "$UNITREE_SDK_ROOT" fetch --depth 1 origin "$UNITREE_SDK_REF"
    git -C "$UNITREE_SDK_ROOT" checkout --detach FETCH_HEAD
    run_pip install --no-deps -e "$UNITREE_SDK_ROOT"
fi

if [[ "$INSTALL_TORCH" == true && "$PROFILE" != "isaac" ]]; then
    # The Isaac profile installs its tested CUDA build above.
    run_pip install torch
fi

echo "Installing SPARK packages in editable mode"
spark_modules=(
    "module/spark_agent"
    "module/spark_env"
    "module/spark_policy"
    "module/spark_robot"
    "module/spark_task"
    "module/spark_utils"
    "pipeline"
)
for module in "${spark_modules[@]}"; do
    run_pip install --no-deps -e "$REPO_ROOT/$module"
done

if [[ "$INSTALL_ROS" == true ]]; then
    echo "Installing ROS helper packages"
    run_pip install rospkg catkin_pkg
    echo "ROS 1/ROS 2 bindings are supplied by the host ROS installation and"
    echo "selected at runtime by spark_task's teleoperation adapter."
fi

if [[ "$PROFILE" == "isaac" ]]; then
    ISAAC_LAB_ROOT="$ENV_ROOT/src/IsaacLab"
    if [[ ! -d "$ISAAC_LAB_ROOT/.git" ]]; then
        mkdir -p "$ENV_ROOT/src"
        echo "Initializing Isaac Lab checkout in $ISAAC_LAB_ROOT"
        git init "$ISAAC_LAB_ROOT"
        git -C "$ISAAC_LAB_ROOT" remote add origin https://github.com/isaac-sim/IsaacLab.git
    fi
    echo "Checking out Isaac Lab $ISAAC_LAB_REF"
    git -C "$ISAAC_LAB_ROOT" fetch --depth 1 origin "$ISAAC_LAB_REF"
    git -C "$ISAAC_LAB_ROOT" checkout --detach FETCH_HEAD

    echo "Installing the minimal Isaac Lab runtime"
    # The Isaac Lab helper installs every discovered extension before applying
    # its optional-feature selector. Install only what SPARK's Isaac backend
    # needs so RL, mimic, and teleoperation extras cannot perturb Isaac Sim's
    # tightly pinned dependencies.
    isaac_lab_modules=(
        "source/isaaclab"
        "source/isaaclab_contrib"
        "source/isaaclab_assets"
        "source/isaaclab_physx"
        # Provides the local Omniverse Kit visualizer selected by
        # ``--visualizer kit`` in the parallel benchmark.
        "source/isaaclab_visualizers"
        # Required by the current PhysX articulation module's optional
        # Newton-actuator capability probe, even when PhysX remains active.
        "source/isaaclab_newton"
    )
    for module in "${isaac_lab_modules[@]}"; do
        run_pip install --no-deps -e "$ISAAC_LAB_ROOT/$module"
    done

    echo "Installing cuRobo V2 at $CUROBO_REF"
    run_pip install --upgrade \
        "nvidia-curobo[cu12-torch] @ git+https://github.com/NVlabs/curobo.git@$CUROBO_REF"
    # cuRobo's optional visualizer otherwise upgrades websockets past the exact
    # version required by Isaac Sim's kernel. This older visualizer supports the
    # Isaac pin and is not used by SPARK's solver backend.
    run_pip install --no-deps "viser==0.1.32" "websockets==12.0"
    run_pip install \
        gdown msgpack nodeenv pyliblzfse "tyro==0.9.17" \
        "typing_extensions==4.12.2" "websockets==12.0"
fi

echo
echo "SPARK installation completed"
echo "  profile:     $PROFILE"
echo "  platform:    $HOST_OS/$HOST_ARCH"
echo "  environment: $ENV_ROOT"
echo "  python:      $($PYTHON_BIN --version)"
echo "  learned:     $INSTALL_LEARNED"
echo "  unitree SDK: $INSTALL_UNITREE_SDK"
echo "  ROS helpers: $INSTALL_ROS"
echo "  dev tools:   $INSTALL_DEV"
if [[ "$ENV_MANAGER" == "conda" ]]; then
    echo "Activate with: conda activate $ENV_NAME"
else
    echo "Activate with: source $ENV_ROOT/bin/activate"
fi
