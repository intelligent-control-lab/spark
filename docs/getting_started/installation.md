# Installation

SPARK separates its numerical core, simulators, learned-policy runtimes, and
hardware integrations so a basic installation does not download CUDA PyTorch,
Isaac, ROS helpers, or a hardware SDK. SPARK supports Python 3.10 and newer;
the `core` and `mujoco` profiles default to Python 3.10. Installing or using
the optional Isaac module requires Python 3.12.x because the release-qualified
Isaac Sim 6 packages require that exact Python minor version.

Python 3.10 or newer is required by every SPARK distribution. This minimum is
declared in each package and checked by `install.sh`, including when the script
reuses an existing environment.

## Numerical core

Use the core profile for dynamics, control, estimation, planning, safety, and
CPU Pinocchio kinematics:

```bash
git clone https://github.com/intelligent-control-lab/spark.git
cd spark
./install.sh --name spark --profile core
conda activate spark
```

To use a `uv` virtual environment instead:

```bash
./install.sh --env-manager uv --path .venv --profile core
source .venv/bin/activate
```

## MuJoCo

Add the MuJoCo simulator and OpenCV rendering dependencies with:

```bash
./install.sh --name spark_mujoco --profile mujoco
conda activate spark_mujoco
```

## macOS

The supported macOS configurations are:

| Host | `core` | `mujoco` | Isaac/CUDA | Unitree hardware SDK |
|---|---:|---:|---:|---:|
| Apple Silicon, macOS 12+ | Yes | Yes | No | No |
| Intel Mac | Yes | No | No | No |

Current MuJoCo 3.4+ Python packages provide Apple Silicon wheels but not Intel
macOS wheels. On Apple Silicon, run a native arm64 terminal and use an arm64
Conda/Miniforge installation or `uv`; a Rosetta/x86 Python environment is not
compatible with the `mujoco` profile.

Install Apple's command line tools if they are not already present, then create
the environment normally:

```bash
xcode-select --install
git clone https://github.com/intelligent-control-lab/spark.git
cd spark
./install.sh --env-manager uv --path .venv --profile mujoco
source .venv/bin/activate
```

For an Intel Mac, replace `mujoco` with `core`. MuJoCo uses its native macOS
graphics path, so Linux-specific settings such as `MUJOCO_GL=egl` should not be
exported. The optional `--torch` and `--learned` flags install native macOS
PyTorch packages; MPS acceleration depends on support in the selected policy.
Host ROS integrations are not part of the macOS release gate.

## Docker

Docker Engine with the Compose v2 plugin provides the same installation
profiles without modifying the host Python environment. SPARK 2.0 supplies
five image variants:

| Launcher profile | Compose service | Python | Contents and host requirements |
| --- | --- | --- | --- |
| `core` | `spark-core` | 3.10 | Numerical toolbox; no simulator or GPU requirement |
| `default` or `mujoco` | `spark-default` | 3.10 | MuJoCo and X11 viewer support |
| `isaac` | `spark-isaac` | 3.12 | Complete Isaac profile; NVIDIA Container Toolkit required |
| `ros` | `spark-ros` | 3.12 | ROS Jazzy base plus MuJoCo and ROS helpers |
| `wsl` | `spark-wsl` | 3.10 | MuJoCo with WSLg/NVIDIA environment forwarding |

Build and smoke-test the default image:

```bash
docker compose --profile default build spark-default
./run_with_docker.sh default python -c \
  "import spark_agent, spark_pipeline, spark_robot; print('SPARK container ready')"
```

For backward compatibility with the 1.0 Docker launcher, a Python file may be
passed directly. All remaining arguments are forwarded without shell parsing:

```bash
./run_with_docker.sh default \
  example/kinova_gen3/run_kinova_gen3_benchmark.py \
  --backend mujoco --test-case arm_goal_static_v0 --headless
```

With no command, the launcher opens an interactive shell. The repository is
mounted at `/workspace/spark`, while the installed virtual environment remains
at `/opt/spark-venv` in the image.

For a Linux X11 or XWayland viewer, permit the local container connection and
revoke it when finished:

```bash
xhost +local:docker
./run_with_docker.sh default \
  example/kinova_gen3/run_kinova_gen3_teleop.py --backend mujoco
xhost -local:docker
```

Headless commands do not require X11 forwarding. Build the optional Isaac
image only on a supported NVIDIA host and explicitly accept NVIDIA's Isaac EULA
when running it:

```bash
OMNI_KIT_ACCEPT_EULA=YES docker compose --profile isaac build spark-isaac
OMNI_KIT_ACCEPT_EULA=YES ./run_with_docker.sh isaac \
  example/kinova_gen3/run_kinova_gen3_benchmark.py \
  --backend isaac --test-case arm_goal_static_v0 \
  --num-envs 1 --headless --max-num-steps 10
```

The default image does not install CUDA or request a GPU. Optional installer
flags are exposed as build variables, for example:

```bash
SPARK_WITH_LEARNED=true SPARK_WITH_DEV=true \
  docker compose --profile default build spark-default
```

The Dockerfile uses `install.sh` directly, so container and host dependency
contracts cannot silently diverge. The `.devcontainer` configuration selects
the MuJoCo service for VS Code users.

## Isaac and cuRobo

The Isaac profile includes the MuJoCo dependencies so both simulators can be
used from one environment. It selects the separately release-qualified Python
3.12 stack by default:

```bash
./install.sh --name spark_isaac --profile isaac
conda activate spark_isaac
```

Isaac requires a supported NVIDIA GPU environment and substantially more disk
space than the MuJoCo profile and is Linux x86_64-only. Start with MuJoCo unless
an Isaac-specific feature is required.

Verify that installation with an actual CUDA kernel and a headless SPARK
articulation run (SPARK uses `--headless` at the public entry point,
not an Isaac `--headless` argument):

```bash
python - <<'PY'
import torch

assert torch.cuda.is_available()
x = torch.randn(512, 512, device="cuda")
torch.cuda.synchronize()
print(torch.cuda.get_device_name(0), torch.linalg.vector_norm(x).item())
PY

OMNI_KIT_ACCEPT_EULA=YES python example/unitree_g1/run_unitree_g1_benchmark.py \
  --robot-config UnitreeG1WholeBodyDynamic1Config \
  --policy-config UnitreeG1WBTSafePolicy \
  --backend isaac --num-envs 1 \
  --test-case base_goal_static_v0 --headless \
  --max-num-steps 10 --isaac-device cuda:0
```

Success requires the benchmark summary to report `stability check: PASS`.
Package installation or import success alone is not considered GPU
validation. The pinned stack and broader parallel-policy gates are described
in {doc}`../tutorials/benchmarking_and_recording`.

Validate representative declarative single-environment adapters separately:

```bash
OMNI_KIT_ACCEPT_EULA=YES python tools/check_isaac_articulation.py \
  --robot-config KinovaGen3SingleArmDynamic1Config --device cuda:0
OMNI_KIT_ACCEPT_EULA=YES python tools/check_isaac_articulation.py \
  --robot-config KukaIIWA14SingleArmDynamic2Config --device cuda:0
```

Each command runs the robot configuration's two-second physical hold gate and
must print `PASS`.

The Isaac profile intentionally selects `coverage==7.15.2`: Isaac Sim 6's
package metadata requests 7.4.4, while the co-installed Numba 0.63.1 runtime on
Python 3.12 needs the newer `coverage.types.Tracer` API. Consequently,
`pip check` reports this known NVIDIA metadata mismatch even though the import,
test, and GPU gates above pass. Similarly, `--unitree-sdk` uses CycloneDDS 11's
Python 3.12 wheel instead of the SDK's obsolete source-only 0.10.2 metadata
pin. These are explicit compatibility overrides, not core-profile
dependencies.

## Optional capabilities

Flags can be combined with `core` or `mujoco`:

| Flag | Adds | Use when |
| --- | --- | --- |
| `--torch` | PyTorch | Developing a custom tensor component without the bundled learned policies |
| `--learned` | PyTorch, ONNX runtimes, and TensorBoard | Running WBT, Sport, learned collision, or other learned policies |
| `--unitree-sdk` | CycloneDDS and the pinned Unitree SDK2 checkout | Connecting to supported Unitree hardware |
| `--ros` | `rospkg` and `catkin_pkg` | Using a host ROS installation through SPARK teleoperation adapters |
| `--dev` | pytest, Ruff, build, twine, and IPython debugger support | Testing or preparing a release |

ROS 1/ROS 2 bindings themselves come from the host ROS installation. The
installer deliberately does not modify the host ROS installation. Isaac always
selects its validated CUDA PyTorch build and learned-policy runtimes, so
`--torch` and `--learned` are unnecessary with `--profile isaac`.

Examples:

```bash
./install.sh --name spark_course --profile core --dev
./install.sh --name spark_learned --profile mujoco --learned
./install.sh --name spark_hardware --profile mujoco --unitree-sdk --ros
```

## Verify the checkout

The complete suite exercises learned-policy and MuJoCo paths. Create the
corresponding development environment, then run:

```bash
./install.sh --name spark_test --profile mujoco --learned --dev
conda activate spark_test
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python tools/check_release.py
```

Continue to the {doc}`quickstart` after installation succeeds.
