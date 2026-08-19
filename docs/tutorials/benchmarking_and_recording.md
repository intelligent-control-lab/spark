# Benchmarking and recording

The Unitree G1 benchmark separates robot, policy, task, and backend selection.
Task cases are self-contained dictionaries and use `v0`, `v1`, and `v2` for
no-obstacle, sparse-obstacle, and dense-obstacle environments.

## Supported matrix

| Mode | MuJoCo | Isaac |
|---|---:|---:|
| WBT single environment | Supported | Supported |
| WBT parallel environments | Supported (isolated processes) | Supported tensor execution |
| Mobile base single/parallel | Supported | Experimental; validated at 1 and 64 environments |
| SONIC | Supported for base-goal cases | Shared server; base-goal execution supported |
| Sport Mode | Supported for base-goal cases | Supported tensor execution |
| Headless recording | Supported | Supported for WBT/mobile-base tensor runs |

## Run one WBT environment

```bash
conda activate spark_isaac
python example/unitree_g1/run_unitree_g1_benchmark.py \
  --robot-config UnitreeG1WholeBodyDynamic1Config \
  --policy-config UnitreeG1WBTSafePolicy \
  --backend mujoco --num-envs 1 \
  --test-case base_goal_static_v1
```

## Run four Isaac environments

```bash
python example/unitree_g1/run_unitree_g1_benchmark.py \
  --robot-config UnitreeG1WholeBodyDynamic1Config \
  --policy-config UnitreeG1WBTSafePolicy \
  --backend isaac --num-envs 4 \
  --test-case base_goal_static_v1 --headless
```

Isaac `--num-envs 1` and `--num-envs 4` use the same tensor adapter and CUDA
execution path. Rows own independent recurrent and episode state and reset
independently.

## Cross-robot conformance and demos

The backend-neutral runner covers every declarative AgiBot, Galaxea, Kinova,
KUKA, and FANUC configuration without robot-name branches:

```bash
python example/run_robot_matrix_case.py \
  --robot-config GalaxeaR1LiteMobileBaseDynamic1Config \
  --backend isaac --num-envs 4 \
  --mode benchmark --dynamics-backend model
```

Run the complete release matrix and retain machine-readable evidence with:

```bash
python tools/run_robot_support_matrix.py \
  --backend mujoco isaac \
  --report docs/reference/robot_support_matrix.json
```

Standalone cross-robot conformance cases execute ten independently reset episodes
by default. Their JSON includes `reset_reports`, aggregate goal-error mean and
standard deviation, reset pose/velocity error, and per-stage timing. Override
the default with `--num-resets N`:

```bash
python example/run_robot_matrix_case.py \
  --robot-config AgiBotG1MobileBaseDynamic1Config \
  --backend mujoco --test-case arm_goal_static_v1 \
  --num-resets 5 --report build/agibot_mujoco.json

python example/run_robot_matrix_case.py \
  --robot-config AgiBotG1MobileBaseDynamic1Config \
  --backend isaac --device cpu --test-case arm_goal_static_v1 \
  --num-resets 5 --report build/agibot_isaac.json
```

Compare `max_goal_error`, `goal_error_mean`, `goal_error_std`,
`reset_initial_*`, and `stage_ms_per_step` between the two reports. A
single-environment Isaac comparison should use CPU; CUDA is intended for
parallel throughput.

Generate the complete local visual matrix for every documented robot profile
and its five MuJoCo/Isaac teleoperation and benchmark workflows with:

```bash
python tools/generate_robot_demo_gifs.py --clean \
  --output-dir build/media/robot_matrix \
  --mujoco-python "$CONDA_PREFIX/bin/python" \
  --isaac-python /path/to/spark_isaac/bin/python \
  --report build/media/robot_matrix/manifest.json
```

This produces 105 real-time GIFs for 21 AgiBot, Galaxea, Kinova, KUKA, FANUC,
and Unitree profiles. It uses robot-only simulator assets, consistent camera
and floor styling, and visible goal, obstacle, path, and collision-volume
overlays. Teleoperation must exercise three visible safety interactions; a GIF
is accepted only when its numerical and encoding checks pass. The separate
numeric release matrix remains the exhaustive configuration/backend gate.

Isaac Sim can live in a dedicated optional environment: `--isaac-python`
selects it without adding Isaac to the core/MuJoCo environment. Isaac parallel
recordings contain the complete four-environment clone grid. The full matrix
remains under the ignored `build/media/` directory. The release checks in only
the eight curated README demonstrations and their compact manifest under
`docs/media/demos/robot_matrix/`.

## Record documentation media

```bash
python tools/record_benchmark.py \
  --robot-config UnitreeG1WholeBodyDynamic1Config \
  --policy-config UnitreeG1WBTSafePolicy \
  --test-case base_goal_static_v1 \
  --backend mujoco --safe-algo rssa \
  --duration 8 --fps 25 --resolution 1280 720 \
  --camera-lookat 0.5 0.0 1.3 \
  --camera-distance 1.7 --camera-azimuth 145 --camera-elevation -20 \
  --output build/media/wbt_v1.mp4 --gif build/media/wbt_v1.gif
```

The tool does not open an interactive viewer. It uses MuJoCo offscreen capture
or Isaac viewport-to-buffer capture, including pipeline overlays. Duration
and frame cadence use simulated time, and the seed defaults to zero. For a
parallel run, pass `--num-envs 4`; the output is a tiled recording of the real
worker processes.

For Isaac parallel recording, select `--backend isaac --num-envs 64` and a
CUDA device. The wrapper enables the explicitly experimental parallel route
and frames the complete cloned environment grid automatically.

MP4 is the archival format. GIF is optional and intended for pages where
inline video is inconvenient. Every recording also produces a
`.command.txt` reproduction record.

## Interactive demonstration review

Use the commands above to inspect scenes interactively before recording final
media. Unreviewed recordings and command sidecars are build artifacts. Curated
documentation GIFs may be committed under `docs/media/demos/` after the
corresponding matrix result passes.
