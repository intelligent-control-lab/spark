"""Internal tensor runtime for Unitree G1 Isaac benchmark execution.

Users should launch ``example/unitree_g1/run_unitree_g1_benchmark.py``.  This
module remains a separate process boundary because Isaac Lab must initialize
Kit before importing its tensor simulation APIs.
"""

from __future__ import annotations

import argparse
import time


class _ViewportVideoRecorder:
    """Asynchronously capture the active Kit viewport into an MP4 stream."""

    def __init__(self, path, *, width, height, fps, control_period):
        import cv2
        from pathlib import Path
        from omni.kit.viewport.utility import get_active_viewport

        output = Path(path).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        self._cv2 = cv2
        self._writer = cv2.VideoWriter(
            str(output),
            cv2.VideoWriter_fourcc(*"mp4v"),
            float(fps),
            (int(width), int(height)),
        )
        if not self._writer.isOpened():
            raise RuntimeError(f"Could not open Isaac video output {output}")
        self.viewport = get_active_viewport()
        if self.viewport is None:
            raise RuntimeError("Isaac recording requires an active Kit viewport")
        self.viewport.resolution = (int(width), int(height))
        self.width = int(width)
        self.height = int(height)
        self.frame_period = 1.0 / float(fps)
        self.control_period = float(control_period)
        self.next_time = 0.0
        self.pending = False
        self.frames = 0

    def schedule(self, step_index):
        from omni.kit.viewport.utility import capture_viewport_to_buffer

        simulated_time = float(step_index) * self.control_period
        if self.pending or simulated_time + 1.0e-12 < self.next_time:
            return
        self.pending = True

        def on_capture(buffer, buffer_size, width, height, pixel_format):
            import numpy as np
            import omni.kit.renderer_capture

            pixels = omni.kit.renderer_capture.convert_raw_bytes_to_list(
                buffer, buffer_size, width, height, pixel_format
            )
            rgba = np.asarray(pixels, dtype=np.uint8).reshape(height, width, -1)
            rgb = rgba[..., :3]
            if (width, height) != (self.width, self.height):
                rgb = self._cv2.resize(rgb, (self.width, self.height))
            self._writer.write(self._cv2.cvtColor(rgb, self._cv2.COLOR_RGB2BGR))
            self.frames += 1
            self.next_time += self.frame_period
            self.pending = False

        capture_viewport_to_buffer(self.viewport, on_capture, is_hdr=False)

    def close(self):
        self._writer.release()
        print(f"[SPARK] Isaac recording wrote {self.frames} frames", flush=True)


def _build_parser():
    # Isaac Lab's launcher selects the matching Kit experience and initializes
    # its extensions in the order required by the tensor backends.  A raw
    # SimulationApp is sufficient for the scalar Isaac adapter, but not for an
    # Isaac Lab InteractiveScene.
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--num-envs",
        type=int,
        default=1,
        help="Start learned-policy validation with one robot; batching is gated explicitly.",
    )
    parser.add_argument("--show-simulation-info", action="store_true")
    parser.add_argument("--num-steps", type=int, default=500)
    parser.add_argument("--record-video-path", default=None)
    parser.add_argument("--record-fps", type=float, default=15.0)
    parser.add_argument("--record-width", type=int, default=1280)
    parser.add_argument("--record-height", type=int, default=720)
    parser.add_argument("--viewer-lookat", type=float, nargs=3)
    parser.add_argument("--viewer-distance", type=float)
    parser.add_argument("--viewer-azimuth", type=float)
    parser.add_argument("--viewer-elevation", type=float)
    parser.add_argument(
        "--warmup-steps",
        type=int,
        default=20,
        help="Cold-start stabilization steps; reapplied only after a physical reset.",
    )
    parser.add_argument("--max-episode-length", type=int, default=1000)
    parser.add_argument("--with-hand", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--dt", type=float, default=0.005)
    parser.add_argument("--control-decimation", type=int, default=4)
    parser.add_argument(
        "--render-every",
        type=int,
        default=1,
        help="Present one Kit frame every N control actions; physics and policy still run every action.",
    )
    parser.add_argument(
        "--isaac-render-quality",
        choices=("performance", "quality", "cinematic"),
        default="performance",
        help="Isaac viewport quality preset; does not alter physics or control.",
    )
    parser.add_argument(
        "--policy",
        choices=("position", "mobile_base", "sport", "wbt", "sonic"),
        default="position",
        help="Controller included in the timed GPU loop; safety is intentionally excluded.",
    )
    parser.add_argument(
        "--sonic-endpoints",
        nargs="+",
        default=None,
        help="One shared SONIC endpoint or one independent endpoint per environment.",
    )
    parser.add_argument("--sonic-timeout-ms", type=int, default=100)
    parser.add_argument(
        "--sonic-locomotion-mode",
        choices=("slow", "walk", "run", "hybrid"),
        default="walk",
        help="SONIC gait used far from a pose goal; near goals use SLOW_WALK.",
    )
    parser.add_argument(
        "--velocity-command",
        type=float,
        nargs=3,
        metavar=("VX", "VY", "YAW_RATE"),
        default=(0.0, 0.0, 0.0),
        help="Body-frame locomotion command. The stability test defaults to standing still.",
    )
    parser.add_argument(
        "--goal-mode",
        choices=("velocity", "fixed", "random"),
        default="random",
        help="Use a raw velocity command or track a fixed/random world-frame base pose.",
    )
    parser.add_argument(
        "--base-goal",
        type=float,
        nargs=3,
        metavar=("X", "Y", "YAW"),
        default=(0.35, 0.0, 0.0),
        help="Per-environment base displacement for fixed goal mode.",
    )
    parser.add_argument(
        "--base-goal-range",
        type=float,
        nargs=6,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "YAWMIN", "YAWMAX"),
        default=(0.2, 0.5, -0.2, 0.2, -0.35, 0.35),
    )
    parser.add_argument(
        "--base-goal-height-range",
        type=float,
        nargs=2,
        default=(0.793, 0.793),
        metavar=("ZMIN", "ZMAX"),
        help="Marker/root-height range retained by the scalar benchmark contract.",
    )
    parser.add_argument(
        "--base-goal-minimum-distance",
        type=float,
        default=0.0,
        help="Minimum planar displacement from the current base for sampled goals.",
    )
    parser.add_argument(
        "--base-goal-workspace-range",
        type=float,
        nargs=4,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
        default=None,
        help="Absolute per-environment XY bounds for sampled base goals.",
    )
    parser.add_argument(
        "--base-goal-relative-to-current",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Interpret random base X/Y bounds as reset-relative displacements.",
    )
    parser.add_argument(
        "--base-goal-velocity",
        type=float,
        default=0.0,
        help="Brownian base-goal speed in m/s; zero keeps the goal static.",
    )
    parser.add_argument("--base-position-kp", type=float, default=0.8)
    parser.add_argument("--base-yaw-kp", type=float, default=0.8)
    parser.add_argument("--base-velocity-kd", type=float, default=0.4)
    parser.add_argument("--base-yaw-velocity-kd", type=float, default=0.25)
    parser.add_argument(
        "--wbt-max-forward-speed",
        type=float,
        default=0.12,
        help="Conservative learned-WBT goal-tracking envelope in m/s.",
    )
    parser.add_argument("--wbt-max-lateral-speed", type=float, default=0.10)
    parser.add_argument("--wbt-max-yaw-rate", type=float, default=0.30)
    parser.add_argument("--wbt-min-translation-speed", type=float, default=0.05)
    parser.add_argument("--wbt-min-yaw-rate", type=float, default=0.08)
    parser.add_argument(
        "--wbt-command-acceleration",
        type=float,
        default=0.30,
        help="Rate limit for learned-WBT planar safety commands in m/s^2.",
    )
    parser.add_argument("--wbt-yaw-acceleration", type=float, default=0.75)
    parser.add_argument(
        "--wbt-base-settle-steps",
        type=int,
        default=50,
        help=(
            "Consecutive in-goal control steps before WBT hands locomotion "
            "back to squat; 50 spans more than one 1.5 Hz gait cycle."
        ),
    )
    parser.add_argument("--dynamics-order", type=int, choices=(1, 2), default=1)
    parser.add_argument("--second-order-position-gain", type=float, default=1.0)
    parser.add_argument("--second-order-velocity-gain", type=float, default=1.0)
    parser.add_argument("--second-order-upper-acceleration-limit", type=float, default=100.0)
    parser.add_argument("--second-order-jacobian-rate-filter", type=float, default=0.2)
    parser.add_argument("--second-order-jacobian-rate-limit", type=float, default=10.0)
    parser.add_argument("--second-order-curvature-gain", type=float, default=1.0)
    parser.add_argument(
        "--wbt-safety-base-blend",
        type=float,
        default=0.0,
        help=(
            "Fraction of the ideal tensor-QP locomotion correction admitted "
            "to the learned WBT policy; 0 is upper-body-only and 1 is the "
            "unmodified ideal-base projection. The stable default remains 0 "
            "until a learned-policy feasibility gate is validated."
        ),
    )
    parser.add_argument("--base-goal-tolerance", type=float, default=0.08)
    parser.add_argument("--base-yaw-tolerance", type=float, default=0.10)
    parser.add_argument(
        "--arm-goal-mode",
        choices=("default", "fixed", "random"),
        default="random",
        help="Choose base-relative Cartesian dual-arm goals, matching BenchmarkTask.",
    )
    parser.add_argument(
        "--left-arm-goal-range",
        type=float,
        nargs=6,
        default=None,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
    )
    parser.add_argument(
        "--right-arm-goal-range",
        type=float,
        nargs=6,
        default=None,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
    )
    parser.add_argument("--arm-goal-minimum-distance", type=float, default=0.0)
    parser.add_argument("--arm-goal-pair-keepout", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=20)
    parser.add_argument("--arm-goal-offset", type=float, default=0.25, help=argparse.SUPPRESS)
    parser.add_argument(
        "--arm-goal-tolerance",
        type=float,
        default=0.05,
        help="Cartesian end-effector tolerance shared with the scalar task.",
    )
    parser.add_argument("--ik-num-seeds", type=int, default=32)
    parser.add_argument(
        "--reset-on-success",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Reset only successful environments and independently sample their next goals.",
    )
    parser.add_argument(
        "--reset-on-timeout",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Reset and fully resample environments that reach the episode limit.",
    )
    parser.add_argument(
        "--reset-on-fall",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Reset each fallen environment independently.",
    )
    parser.add_argument("--fall-height-threshold", type=float, default=0.45)
    parser.add_argument("--fall-tilt-degrees", type=float, default=60.0)
    parser.add_argument(
        "--workspace-exit-margin",
        type=float,
        default=0.10,
        help="Root-center tolerance outside task sampling bounds before reset.",
    )
    parser.add_argument(
        "--alternate-grippers",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Alternate closed/open hand goals across environments during hand-control tests.",
    )
    parser.add_argument(
        "--num-obstacles",
        type=int,
        default=0,
        help=(
            "Number of obstacles per environment. In shared-scene mode the "
            "single global scene contains num-envs times this many obstacles."
        ),
    )
    parser.add_argument(
        "--shared-obstacle-environment",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Use one world-space moving obstacle set for all environments.",
    )
    parser.add_argument(
        "--shared-robot-yaw-range",
        type=float,
        nargs=2,
        metavar=("YAWMIN", "YAWMAX"),
        default=(-3.141592653589793, 3.141592653589793),
        help=(
            "Random world-yaw interval used only in shared-scene mode. "
            "Successful robots remain at their reached pose; falls respawn here."
        ),
    )
    parser.add_argument(
        "--shared-workspace-range",
        type=float,
        nargs=6,
        default=None,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        help=(
            "Absolute world bounds for shared robot spawning and obstacles. "
            "When omitted, the --obstacle-range X-Y area is automatically "
            "multiplied by num-envs while preserving its center and Z range."
        ),
    )
    parser.add_argument("--shared-robot-keepaway", type=float, default=0.8)
    parser.add_argument(
        "--obstacle-query-range",
        type=float,
        default=2.0,
        help="World-space radius used to build each robot's local obstacle set.",
    )
    parser.add_argument(
        "--max-nearby-obstacles",
        type=int,
        default=16,
        help="Fixed per-robot obstacle capacity after spatial filtering.",
    )
    parser.add_argument("--obstacle-radius", type=float, default=0.05)
    parser.add_argument(
        "--environment-representation",
        choices=("sphere", "point_cloud", "mesh"),
        default="sphere",
        help="Distance-query representation for benchmark obstacles.",
    )
    parser.add_argument(
        "--points-per-obstacle",
        type=int,
        default=64,
        help="Surface samples per obstacle in point-cloud mode.",
    )
    parser.add_argument(
        "--point-radius",
        type=float,
        default=None,
        help="Point-sphere radius; defaults to a coverage-derived value.",
    )
    parser.add_argument(
        "--dynamic-point-count",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Vary valid point counts per environment/update like perception output.",
    )
    parser.add_argument("--minimum-points-per-obstacle", type=int, default=4)
    parser.add_argument(
        "--point-visual-size",
        type=float,
        default=0.008,
        help="Rendered point diameter in meters, independent of collision radius.",
    )
    parser.add_argument("--mesh-latitude-segments", type=int, default=8)
    parser.add_argument("--mesh-longitude-segments", type=int, default=12)
    parser.add_argument(
        "--object-mesh-path",
        default=None,
        help="Optional STL/OBJ collision mesh; defaults to procedural spheres.",
    )
    parser.add_argument("--object-mesh-scale", type=float, default=1.0)
    parser.add_argument(
        "--obstacle-velocity",
        type=float,
        default=0.005,
        help="Physical obstacle speed in m/s; zero keeps obstacles static.",
    )
    parser.add_argument(
        "--obstacle-goal-keepaway",
        type=float,
        default=0.15,
        help="Minimum surface clearance between sampled obstacles and task goals.",
    )
    parser.add_argument(
        "--obstacle-keepout",
        type=float,
        default=0.05,
        help="Minimum surface clearance between sampled benchmark obstacles.",
    )
    parser.add_argument(
        "--obstacle-robot-keepaway",
        type=float,
        default=0.05,
        help="Initial surface clearance from enabled robot collision volumes.",
    )
    parser.add_argument(
        "--obstacle-range",
        type=float,
        nargs=6,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        default=(0.1, 0.7, -0.4, 0.4, 0.4, 1.1),
    )
    parser.add_argument(
        "--safe-algo",
        choices=("bypass", "ssa", "rssa", "sss", "rsss", "cbf", "rcbf", "pfm", "sma"),
        default="bypass",
        help="SPARK safety algorithm implemented by its batched tensor backend.",
    )
    parser.add_argument("--minimum-distance", type=float, default=0.05)
    parser.add_argument(
        "--self-collision-minimum-distance",
        type=float,
        default=0.0,
        help="Separate signed-clearance threshold for non-adjacent robot self pairs.",
    )
    parser.add_argument("--safety-activation-distance", type=float, default=0.15)
    parser.add_argument("--safety-alpha", type=float, default=4.0)
    parser.add_argument("--safety-qp-iterations", type=int, default=20)
    parser.add_argument("--safety-slack-weight", type=float, default=1000.0)
    parser.add_argument("--safety-reactive-gain", type=float, default=0.5)
    parser.add_argument(
        "--profile-safety-stages",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Synchronize CUDA and report collision/constraint/solver latency.",
    )
    parser.add_argument(
        "--safety-nearest-points-per-link",
        type=int,
        default=2,
        help="Retain this many nearest environment constraints per robot sphere.",
    )
    parser.add_argument(
        "--safety-point-chunk-size",
        type=int,
        default=2048,
        help="Bound temporary memory for dynamic point-cloud distance queries.",
    )
    parser.add_argument(
        "--render-robot-collision-volumes",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--render-safety-trigger-constraints",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Render blue witness lines for constraints requiring safety correction.",
    )
    parser.add_argument(
        "--render-safety-violations",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Render thick purple witness lines for residual post-filter violations.",
    )
    parser.add_argument(
        "--enable-self-collision",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Apply self-collision checking in both dual-arm IK and safety filtering.",
    )
    parser.add_argument(
        "--wbt-stance",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Hold the WBT locomotion gait clock in stance (only used with --wbt-mode loco).",
    )
    parser.add_argument(
        "--wbt-mode",
        choices=("squat", "loco"),
        default="squat",
        help="WBT uses its squat controller for standing, matching the composed scalar policy.",
    )
    parser.add_argument(
        "--wbt-transition-steps",
        type=int,
        default=50,
        help="Live squat-to-locomotion target blend after warmup.",
    )
    parser.add_argument(
        "--hold-arm-goals-during-locomotion",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "Keep the last safe upper-body target while WBT is walking, then "
            "track the Cartesian arm goals after the base settles. Disabled "
            "by default so arm tracking remains active during locomotion."
        ),
    )
    parser.add_argument("--motor-kp-scale", type=float, default=None)
    parser.add_argument("--motor-kd-scale", type=float, default=None)
    AppLauncher.add_app_launcher_args(parser)
    return parser


def _parse_args():
    args = _build_parser().parse_args()
    # Isaac Lab 3.0's lean ``isaaclab.python.kit`` experience can stall in
    # SimulationApp._wait_for_viewport with Isaac Sim 6 on this supported local
    # GUI path.  The Isaac Sim Python experience initializes the same PhysX and
    # tensor APIs and reliably creates the Kit viewport.  Keep explicit user
    # experience selections untouched.
    if args.visualizer is not None and "kit" in args.visualizer and not args.experience:
        args.experience = "isaacsim.exp.base.python.kit"
    return args


def main():
    args = _parse_args()
    if args.hold_arm_goals_during_locomotion is None:
        args.hold_arm_goals_during_locomotion = False
    visualizer = str(args.visualizer or "off").lower()
    render_enabled = visualizer not in ("off", "none", "headless")
    # Scalar Sonic Isaac uses the WBT gain vector without an additional hand
    # multiplier. Keep generic hand scaling for the other tensor policies.
    motor_kp_scale = (
        1.0
        if args.policy == "sonic"
        else (
            1.2
            if args.with_hand and args.motor_kp_scale is None
            else float(args.motor_kp_scale or 1.0)
        )
    )
    motor_kd_scale = (
        1.0
        if args.policy == "sonic"
        else (
            1.5
            if args.with_hand and args.motor_kd_scale is None
            else float(args.motor_kd_scale or 1.0)
        )
    )
    if args.num_envs < 1:
        raise ValueError("--num-envs must be positive")
    if args.num_obstacles < 0:
        raise ValueError("--num-obstacles must be non-negative")

    from isaaclab.app import AppLauncher

    app_launcher = AppLauncher(args)
    simulation_app = app_launcher.app
    from spark_agent.simulation.isaac.render_quality import apply_isaac_render_preset

    render_config = apply_isaac_render_preset(args.isaac_render_quality)
    print(
        f"[SPARK] Isaac render quality: {render_config['quality']} "
        f"({render_config['renderer']}, {render_config['width']}x{render_config['height']})",
        flush=True,
    )
    print("[SPARK] Isaac Sim application ready", flush=True)
    agent = None
    policy = None
    video_recorder = None
    try:
        import numpy as np
        import torch

        from spark_agent import (
            UnitreeG1MobileBaseIsaacAgent,
            UnitreeG1WholeBodyIsaacAgent,
        )
        from spark_agent.simulation.viewer_config import DEFAULT_VIEWER_CONFIG
        from spark_policy.control.whole_body.unitree_g1 import (
            UnitreeG1BatchedSonicPolicy,
            UnitreeG1BatchedWBTPolicy,
            UnitreeG1SportPolicy,
        )
        from spark_policy.safety.geometry import (
            PointCloudBatch,
            TorchMeshCollisionBackend,
            TorchSphereCollisionBackend,
            TriangleMeshBatch,
            build_link_sphere_model,
            fibonacci_sphere,
            load_triangle_mesh,
            uv_sphere_mesh,
        )
        from spark_policy.safety.tensor import (
            BatchedQPSafetyFilter,
            BatchedReactiveSafetyFilter,
            BatchedRelaxedQPSafetyFilter,
            FirstOrderTensorSafetyIndex,
            SecondOrderTensorSafetyIndex,
            TensorSafetyConstraints,
        )
        from spark_robot import (
            UnitreeG1DualArmDynamic1Config,
            UnitreeG1FixedBaseDynamic1Config,
            UnitreeG1FixedBaseKinematics,
            UnitreeG1WholeBodyDynamic1Config,
            UnitreeG1WholeBodyKinematics,
            UnitreeG1WholeBodyWithHandDynamic1Config,
        )
        from spark_robot.kinematics import (
            BoundedLeastSquaresIK,
            IKProblem,
            IKSolverConfig,
            IKTarget,
            UnitreeG1CuroboDualArmIK,
        )
        from spark_robot.unitree_g1.kinematics.unitree_g1_dual_arm_kinematics import (
            UnitreeG1DualArmKinematics,
        )
        from spark_task.autonomy.benchmark_goals import (
            DEFAULT_LEFT_ARM_GOAL_RANGE,
            DEFAULT_RIGHT_ARM_GOAL_RANGE,
            benchmark_scenario_fingerprint,
            benchmark_scenario_seed,
            sample_benchmark_scenario,
            sample_bounded_position,
        )

        print("[SPARK] Isaac Lab and SPARK modules imported", flush=True)

        robot_cfg = (
            UnitreeG1WholeBodyWithHandDynamic1Config()
            if args.with_hand
            else UnitreeG1WholeBodyDynamic1Config()
        )
        agent_class = (
            UnitreeG1MobileBaseIsaacAgent
            if args.policy == "mobile_base"
            else UnitreeG1WholeBodyIsaacAgent
        )
        viewer_config = dict(DEFAULT_VIEWER_CONFIG)
        explicit_viewer = any(
            value is not None
            for value in (
                args.viewer_lookat,
                args.viewer_distance,
                args.viewer_azimuth,
                args.viewer_elevation,
            )
        )
        if args.viewer_lookat is not None:
            viewer_config["camera_lookat"] = tuple(args.viewer_lookat)
        for argument, field in (
            (args.viewer_distance, "camera_distance"),
            (args.viewer_azimuth, "camera_azimuth"),
            (args.viewer_elevation, "camera_elevation"),
        ):
            if argument is not None:
                viewer_config[field] = float(argument)
        if args.record_video_path and args.num_envs > 1 and not explicit_viewer:
            grid_side = int(np.ceil(np.sqrt(args.num_envs)))
            grid_extent = max(2.5, (grid_side - 1) * 2.5)
            viewer_config.update(
                camera_lookat=(0.0, 0.0, 0.8),
                camera_distance=1.45 * grid_extent,
                camera_azimuth=135.0,
                camera_elevation=-50.0,
                grid_extent=max(5.0, 0.75 * grid_extent),
            )
        agent_kwargs = {}
        if args.policy == "sonic":
            # Keep policy-specific plant parameters beside the policy while
            # making the tensor benchmark instantiate the exact same Isaac
            # plant as the stable scalar/teleop SONIC pipeline.
            from spark_policy.control.whole_body.unitree_g1.sonic.config import (
                sonic_isaac_actuation_config,
            )

            agent_kwargs.update(sonic_isaac_actuation_config())
        elif args.policy == "wbt":
            # Use the exact plant and actuation realization selected by
            # one-environment WBT teleoperation. Environment count must not
            # switch WBT to a different controller or dynamics asset.
            from spark_policy.control.whole_body.unitree_g1.wbt import (
                wbt_isaac_actuation_config,
            )

            agent_kwargs.update(wbt_isaac_actuation_config())
        agent = agent_class(
            robot_cfg,
            num_envs=args.num_envs,
            device=args.device,
            dt=args.dt,
            control_decimation=args.control_decimation,
            enable_hand_control=args.with_hand,
            render=render_enabled,
            viewer_show_simulation_info=args.show_simulation_info,
            render_decimation=args.render_every,
            defer_render=True,
            viewer_config=viewer_config,
            render_robot_collision_volumes=args.render_robot_collision_volumes,
            **agent_kwargs,
        )
        if args.record_video_path:
            video_recorder = _ViewportVideoRecorder(
                args.record_video_path,
                width=args.record_width,
                height=args.record_height,
                fps=args.record_fps,
                control_period=args.dt * args.control_decimation,
            )
        print("[SPARK] Batched Unitree G1 agent ready", flush=True)

        generator = torch.Generator(device=args.device)
        generator.manual_seed(args.seed)
        obstacle_batch = 1 if args.shared_obstacle_environment else args.num_envs
        obstacles_per_env = max(0, int(args.num_obstacles))
        obstacle_count = (
            obstacles_per_env * int(args.num_envs)
            if args.shared_obstacle_environment
            else obstacles_per_env
        )
        local_obstacle_capacity = min(
            max(1, int(args.max_nearby_obstacles)), max(1, obstacle_count)
        )
        obstacle_positions = torch.empty(
            obstacle_batch, 0, 3, device=args.device, dtype=torch.float32
        )
        obstacle_bounds = torch.tensor(
            args.obstacle_range, device=args.device, dtype=torch.float32
        ).reshape(3, 2)
        shared_workspace_bounds = torch.tensor(
            args.shared_workspace_range or args.obstacle_range,
            device=args.device,
            dtype=torch.float32,
        ).reshape(3, 2)
        if args.shared_obstacle_environment and args.shared_workspace_range is None:
            # Obstacles and robots occupy a ground-plane workspace. Preserve
            # the single-environment vertical interval and multiply horizontal
            # area by num_envs, matching the num-obstacles-per-env semantics.
            center = shared_workspace_bounds.mean(dim=1)
            half_span = 0.5 * (shared_workspace_bounds[:, 1] - shared_workspace_bounds[:, 0])
            half_span[:2] *= float(args.num_envs) ** 0.5
            shared_workspace_bounds[:, 0] = center - half_span
            shared_workspace_bounds[:, 1] = center + half_span
        if args.shared_obstacle_environment:
            bounds_list = shared_workspace_bounds.detach().cpu().tolist()
            print(
                "[SPARK] shared workspace: "
                + ", ".join(
                    f"{axis}=[{bounds[0]:+.2f}, {bounds[1]:+.2f}]"
                    for axis, bounds in zip("XYZ", bounds_list)
                )
                + (
                    " (explicit override)"
                    if args.shared_workspace_range is not None
                    else f" (automatic {args.num_envs}x X-Y area)"
                ),
                flush=True,
            )
        if obstacle_count:
            obstacle_positions = torch.zeros(
                obstacle_batch,
                obstacle_count,
                3,
                device=args.device,
                dtype=torch.float32,
            )
        print(
            f"[SPARK] obstacle count: {obstacles_per_env} per env, "
            f"{obstacles_per_env * int(args.num_envs)} total"
            + (f" ({obstacle_count} in shared scene)" if args.shared_obstacle_environment else ""),
            flush=True,
        )
        obstacle_directions = torch.zeros_like(obstacle_positions)

        safety_enabled = args.safe_algo != "bypass"
        collision_backend = None
        self_collision_backend = None
        collision_model = None
        safety_index = None
        self_safety_index = None
        safety_filter = None
        obstacle_cloud = None
        obstacle_mesh = None
        point_surface_offsets = None
        mesh_surface_offsets = None
        environment_point_radius = None
        safety_trigger_count = torch.zeros(args.num_envs, device=args.device, dtype=torch.long)
        safety_minimum_distance = torch.full((args.num_envs,), torch.inf, device=args.device)
        safety_environment_minimum_distance = torch.full(
            (args.num_envs,), torch.inf, device=args.device
        )
        safety_self_minimum_distance = torch.full((args.num_envs,), torch.inf, device=args.device)
        safety_max_residual = torch.zeros(args.num_envs, device=args.device, dtype=torch.float32)
        latest_collision_centers = None
        latest_collision_mask = None
        latest_collision_distance = None
        latest_safety_visualization = None
        safety_profile = {"query": 0.0, "constraints": 0.0, "solve": 0.0, "calls": 0}
        step_profile = {
            name: 0.0
            for name in (
                "obstacles",
                "feedback_goal",
                "safety_total",
                "policy_inference",
                "physics_step",
                "post_step",
                "reset_checks",
                "visualization",
            )
        }
        step_profile["calls"] = 0
        geometry_needed = safety_enabled or args.render_robot_collision_volumes
        if geometry_needed:
            if args.policy not in {"wbt", "sonic", "sport", "mobile_base"}:
                raise ValueError(
                    "Tensor collision geometry requires a Unitree G1 whole-body policy"
                )
            body_kinematics = agent.get_body_kinematics()
            collision_model = build_link_sphere_model(
                robot_cfg, body_kinematics["body_names"], device=args.device
            )
        if safety_enabled:
            if args.shared_obstacle_environment and args.environment_representation == "mesh":
                raise ValueError(
                    "Shared obstacle filtering currently supports sphere and point_cloud "
                    "representations; mesh filtering requires a spatial acceleration structure."
                )
            if args.environment_representation == "mesh":
                if args.num_envs != 1:
                    raise ValueError(
                        "Triangle-mesh safety is initially restricted to --num-envs 1; "
                        "use point_cloud for batched environments."
                    )
                collision_backend = TorchMeshCollisionBackend()
            else:
                collision_backend = TorchSphereCollisionBackend()
            # Environment meshes use triangle queries, while robot
            # self-collision remains sphere--sphere for every representation.
            # Do not dispatch query_self through the mesh backend.
            self_collision_backend = TorchSphereCollisionBackend()
            index_type = (
                SecondOrderTensorSafetyIndex
                if args.dynamics_order == 2
                else FirstOrderTensorSafetyIndex
            )
            index_kwargs = dict(
                minimum_distance=args.minimum_distance,
                activation_distance=(
                    1.0e6 if args.safe_algo in {"cbf", "rcbf"} else args.safety_activation_distance
                ),
            )
            if args.dynamics_order == 2:
                index_kwargs.update(
                    position_gain=args.second_order_position_gain,
                    velocity_gain=args.second_order_velocity_gain,
                    curvature_gain=args.second_order_curvature_gain,
                )
            else:
                index_kwargs.update(
                    alpha=args.safety_alpha,
                    mode=("safe_set" if args.safe_algo in {"ssa", "rssa"} else "sublevel"),
                )
            safety_index = index_type(**index_kwargs)
            self_index_kwargs = dict(
                minimum_distance=args.self_collision_minimum_distance,
                activation_distance=(
                    1.0e6 if args.safe_algo in {"cbf", "rcbf"} else args.safety_activation_distance
                ),
            )
            if args.dynamics_order == 2:
                self_index_kwargs.update(
                    position_gain=args.second_order_position_gain,
                    velocity_gain=args.second_order_velocity_gain,
                    curvature_gain=args.second_order_curvature_gain,
                )
            else:
                self_index_kwargs.update(alpha=args.safety_alpha, mode="sublevel")
            self_safety_index = index_type(**self_index_kwargs)
            if args.safe_algo in {"ssa", "sss", "cbf"}:
                safety_filter = BatchedQPSafetyFilter(iterations=args.safety_qp_iterations)
            elif args.safe_algo in {"rssa", "rsss", "rcbf"}:
                safety_filter = BatchedRelaxedQPSafetyFilter(
                    iterations=args.safety_qp_iterations,
                    slack_weight=args.safety_slack_weight,
                )
            else:
                safety_filter = BatchedReactiveSafetyFilter(gain=args.safety_reactive_gain)
            origins = agent.scene.env_origins[:, None, :]
            if args.environment_representation == "point_cloud":
                samples = fibonacci_sphere(args.points_per_obstacle)
                point_surface_offsets = torch.as_tensor(
                    samples, device=args.device, dtype=torch.float32
                ) * float(args.obstacle_radius)
                point_count = (
                    local_obstacle_capacity if args.shared_obstacle_environment else obstacle_count
                ) * args.points_per_obstacle
                coverage_radius = (
                    float(args.obstacle_radius) * (4.0 / max(4, args.points_per_obstacle)) ** 0.5
                )
                environment_point_radius = (
                    coverage_radius if args.point_radius is None else float(args.point_radius)
                )
                obstacle_cloud = PointCloudBatch(
                    positions=torch.zeros(
                        args.num_envs,
                        point_count,
                        3,
                        device=args.device,
                        dtype=torch.float32,
                    ),
                    radii=torch.full(
                        (args.num_envs, point_count),
                        environment_point_radius,
                        device=args.device,
                    ),
                    valid_mask=torch.ones(
                        args.num_envs,
                        point_count,
                        device=args.device,
                        dtype=torch.bool,
                    ),
                )
            elif args.environment_representation == "mesh":
                if args.object_mesh_path:
                    unit_vertices, unit_faces = load_triangle_mesh(
                        args.object_mesh_path, scale=args.object_mesh_scale
                    )
                else:
                    unit_vertices, unit_faces = uv_sphere_mesh(
                        args.mesh_latitude_segments,
                        args.mesh_longitude_segments,
                    )
                mesh_surface_offsets = torch.as_tensor(
                    unit_vertices, device=args.device, dtype=torch.float32
                )
                if not args.object_mesh_path:
                    mesh_surface_offsets *= float(args.obstacle_radius)
                vertices_per_obstacle = mesh_surface_offsets.shape[0]
                faces = torch.as_tensor(unit_faces, device=args.device, dtype=torch.long)
                combined_faces = (
                    torch.cat(
                        [faces + index * vertices_per_obstacle for index in range(obstacle_count)],
                        dim=0,
                    )
                    if obstacle_count
                    else faces[:0]
                )
                obstacle_mesh = TriangleMeshBatch(
                    vertices=torch.zeros(
                        1,
                        obstacle_count * vertices_per_obstacle,
                        3,
                        device=args.device,
                        dtype=torch.float32,
                    ),
                    faces=combined_faces,
                )
            else:
                obstacle_cloud = PointCloudBatch(
                    positions=(
                        torch.zeros(
                            args.num_envs,
                            local_obstacle_capacity,
                            3,
                            device=args.device,
                            dtype=torch.float32,
                        )
                        if args.shared_obstacle_environment
                        else obstacle_positions + origins
                    ),
                    radii=torch.full(
                        (
                            (args.num_envs, local_obstacle_capacity)
                            if args.shared_obstacle_environment
                            else obstacle_positions.shape[:2]
                        ),
                        float(args.obstacle_radius),
                        device=args.device,
                    ),
                    valid_mask=torch.ones(
                        (
                            (args.num_envs, local_obstacle_capacity)
                            if args.shared_obstacle_environment
                            else obstacle_positions.shape[:2]
                        ),
                        device=args.device,
                        dtype=torch.bool,
                    ),
                )
            print(
                f"[SPARK] tensor safety ready: {len(collision_model.body_names)} link spheres, "
                f"{collision_model.self_pair_i.numel()} self pairs, "
                f"{obstacle_positions.shape[1]} {args.environment_representation} obstacles; "
                f"environment minimum={args.minimum_distance:.3f} m, "
                f"self minimum={args.self_collision_minimum_distance:.3f} m; "
                f"unmapped frames={collision_model.unmapped_frames}",
                flush=True,
            )

        def sphere_centers_and_offsets():
            kin = agent.get_body_kinematics()
            body_positions = kin["body_position_w"][:, collision_model.body_ids]
            body_quaternions = kin["body_quaternion_w"][:, collision_model.body_ids]
            local = collision_model.local_offsets[None].expand(body_positions.shape[0], -1, -1)
            qvec = body_quaternions[:, :, :3]
            qw = body_quaternions[:, :, 3]
            t = 2.0 * torch.cross(qvec, local, dim=-1)
            world_offset = local + qw[:, :, None] * t + torch.cross(qvec, t, dim=-1)
            return body_positions + world_offset, world_offset

        target = agent.default_body_pos.clone()
        command = torch.zeros_like(target)
        gripper_closed = torch.zeros(args.num_envs, device=args.device, dtype=torch.bool)
        if args.alternate_grippers:
            gripper_closed = torch.arange(args.num_envs, device=args.device) % 2 == 0
        policy = None
        if args.policy == "sport":
            policy = UnitreeG1SportPolicy(robot_cfg, num_envs=args.num_envs, device=args.device)
        elif args.policy == "wbt":
            policy = UnitreeG1BatchedWBTPolicy(
                robot_cfg, num_envs=args.num_envs, device=args.device
            )
        elif args.policy == "sonic":
            if args.sonic_endpoints is None or len(args.sonic_endpoints) not in (1, args.num_envs):
                raise ValueError(
                    "SONIC requires one shared endpoint or one endpoint per environment"
                )
            policy = UnitreeG1BatchedSonicPolicy(
                args.sonic_endpoints,
                batch_size=args.num_envs,
                device=args.device,
                timeout_ms=args.sonic_timeout_ms,
                locomotion_mode=args.sonic_locomotion_mode,
            )
        velocity_command = torch.tensor(
            args.velocity_command, device=args.device, dtype=torch.float32
        ).repeat(args.num_envs, 1)

        def sample_shared_spawn_pose(env_ids):
            """Sample independent poses throughout one absolute shared world."""
            env_ids = torch.as_tensor(env_ids, device=args.device, dtype=torch.long)
            default_pose = agent.robot.data.default_root_pose.torch[env_ids].clone()
            default_pose[:, :3] += agent.scene.env_origins[env_ids]
            yaw_bounds = torch.tensor(
                args.shared_robot_yaw_range,
                device=args.device,
                dtype=torch.float32,
            )
            yaw_samples = yaw_bounds[0] + torch.rand(
                env_ids.numel(), generator=generator, device=args.device
            ) * (yaw_bounds[1] - yaw_bounds[0])
            # X/Y come from the shared workspace rather than per-clone
            # origins; yaw uses the configured spawn interval.
            workspace_lower = shared_workspace_bounds[:2, 0]
            workspace_upper = shared_workspace_bounds[:2, 1]
            goal_margin = float(args.base_goal_tolerance)
            if args.goal_mode == "random":
                relative_goal = torch.tensor(
                    args.base_goal_range,
                    device=args.device,
                    dtype=torch.float32,
                ).reshape(3, 2)[:2]
                lower_xy = workspace_lower + goal_margin - relative_goal[:, 1]
                upper_xy = workspace_upper - goal_margin - relative_goal[:, 0]
            elif args.goal_mode == "fixed":
                relative_goal = torch.tensor(
                    args.base_goal[:2], device=args.device, dtype=torch.float32
                )
                lower_xy = workspace_lower + goal_margin - relative_goal
                upper_xy = workspace_upper - goal_margin - relative_goal
            else:
                lower_xy = workspace_lower + goal_margin
                upper_xy = workspace_upper - goal_margin
            # The robot itself must also begin inside the workspace.
            lower_xy = torch.maximum(lower_xy, workspace_lower)
            upper_xy = torch.minimum(upper_xy, workspace_upper)
            if bool(torch.any(lower_xy > upper_xy).item()):
                raise ValueError(
                    "Shared workspace cannot contain both the sampled robot pose "
                    "and configured relative base-goal range"
                )
            selected_xy = default_pose[:, :2].clone()
            for row in range(env_ids.numel()):
                for _ in range(100):
                    candidate = lower_xy + torch.rand(
                        2, generator=generator, device=args.device
                    ) * (upper_xy - lower_xy)
                    if row == 0 or bool(
                        torch.all(
                            torch.linalg.vector_norm(selected_xy[:row] - candidate[None], dim=-1)
                            >= float(args.shared_robot_keepaway)
                        ).item()
                    ):
                        selected_xy[row] = candidate
                        break
                else:
                    raise RuntimeError(
                        "Shared workspace is too small for the requested robot keepaway"
                    )
            default_pose[:, :2] = selected_xy
            half_yaw = 0.5 * yaw_samples
            # Isaac Lab exposes root quaternions as XYZW.
            default_pose[:, 3:7] = 0.0
            default_pose[:, 5] = torch.sin(half_yaw)
            default_pose[:, 6] = torch.cos(half_yaw)
            return default_pose

        if args.shared_obstacle_environment:
            all_env_ids = torch.arange(args.num_envs, device=args.device, dtype=torch.long)
            shared_spawn_pose = sample_shared_spawn_pose(all_env_ids)
            agent.reset(env_ids=all_env_ids, root_pose_w=shared_spawn_pose)
            print(
                "[SPARK] shared-scene randomized world robot XY spawns: "
                + ", ".join(
                    f"env {index}=({xy[0]:+.2f}, {xy[1]:+.2f})"
                    for index, xy in enumerate(shared_spawn_pose[:, :2].detach().cpu().tolist())
                ),
                flush=True,
            )

        initial_feedback = agent.get_feedback()
        initial_root_pose = initial_feedback["root_pose_w"].clone()
        # Base benchmark goals are planar SE(2) targets.  Keep their marker at
        # a fixed nominal visualization height instead of attaching it to the
        # measured root z, which made the goal appear to bounce with the gait.
        base_goal_marker_heights = (
            (initial_root_pose[:, 2] - agent.scene.env_origins[:, 2]).detach().cpu().numpy()
        )
        initial_body_joint_pos = initial_feedback["body_joint_pos"].clone()
        base_goals = initial_root_pose[:, :2].new_zeros(args.num_envs, 3)
        base_goal_offsets = base_goals.clone()
        base_goal_directions = base_goals[:, :2].clone()

        def workspace_bounds(env_ids):
            """Return absolute XYZ bounds for selected environment rows."""
            env_ids = torch.as_tensor(env_ids, device=args.device, dtype=torch.long)
            if args.shared_obstacle_environment:
                lower = shared_workspace_bounds[:, 0].expand(env_ids.numel(), -1)
                upper = shared_workspace_bounds[:, 1].expand(env_ids.numel(), -1)
            else:
                origins = agent.scene.env_origins[env_ids]
                lower = origins + obstacle_bounds[:, 0]
                upper = origins + obstacle_bounds[:, 1]
                if args.base_goal_workspace_range is not None:
                    goal_workspace = torch.tensor(
                        args.base_goal_workspace_range,
                        device=args.device,
                        dtype=torch.float32,
                    ).reshape(2, 2)
                    lower[:, :2] = origins[:, :2] + goal_workspace[:, 0]
                    upper[:, :2] = origins[:, :2] + goal_workspace[:, 1]
            return lower, upper

        def sample_base_goal_rows(env_ids, root_pose):
            """Sample nearby goals while keeping every target in its workspace."""
            env_ids = torch.as_tensor(env_ids, device=args.device, dtype=torch.long)
            root_pose = root_pose[env_ids]
            lower_workspace, upper_workspace = workspace_bounds(env_ids)
            lower_workspace = lower_workspace[:, :2]
            upper_workspace = upper_workspace[:, :2]
            margin = float(args.base_goal_tolerance)
            lower_workspace = lower_workspace + margin
            upper_workspace = upper_workspace - margin
            if bool(torch.any(lower_workspace > upper_workspace).item()):
                raise ValueError("Workspace is smaller than the base-goal tolerance")

            result = torch.zeros(env_ids.numel(), 3, device=args.device)
            if args.goal_mode == "fixed":
                requested = torch.tensor(
                    args.base_goal, device=args.device, dtype=torch.float32
                ).repeat(env_ids.numel(), 1)
                result[:, :2] = torch.clamp(
                    root_pose[:, :2] + requested[:, :2],
                    lower_workspace,
                    upper_workspace,
                )
                result[:, 2] = requested[:, 2]
            elif args.goal_mode == "random":
                relative = torch.tensor(
                    args.base_goal_range, device=args.device, dtype=torch.float32
                ).reshape(3, 2)
                lower = torch.maximum(root_pose[:, :2] + relative[:2, 0], lower_workspace)
                upper = torch.minimum(root_pose[:, :2] + relative[:2, 1], upper_workspace)
                feasible = torch.all(lower <= upper, dim=1)
                if bool(torch.any(feasible).item()):
                    count = int(feasible.sum().item())
                    result[feasible, :2] = lower[feasible] + torch.rand(
                        count, 2, generator=generator, device=args.device
                    ) * (upper[feasible] - lower[feasible])

                # A forward-biased relative box can have an empty literal
                # intersection after a robot reaches a goal near a boundary.
                # Preserve its minimum/maximum displacement magnitude while
                # choosing an inward direction instead of failing the run.
                infeasible_rows = torch.nonzero(~feasible).flatten()
                if infeasible_rows.numel():
                    x_interval, y_interval = relative[0], relative[1]
                    zero = torch.zeros((), device=args.device)
                    dx_min = torch.where(
                        (x_interval[0] <= 0) & (x_interval[1] >= 0),
                        zero,
                        torch.minimum(torch.abs(x_interval[0]), torch.abs(x_interval[1])),
                    )
                    dy_min = torch.where(
                        (y_interval[0] <= 0) & (y_interval[1] >= 0),
                        zero,
                        torch.minimum(torch.abs(y_interval[0]), torch.abs(y_interval[1])),
                    )
                    minimum_radius = torch.sqrt(dx_min.square() + dy_min.square())
                    maximum_radius = torch.sqrt(
                        torch.max(x_interval.abs()).square() + torch.max(y_interval.abs()).square()
                    )
                    for row in infeasible_rows.detach().cpu().tolist():
                        for _ in range(200):
                            angle = (
                                2.0
                                * np.pi
                                * torch.rand((), generator=generator, device=args.device)
                            )
                            radius = minimum_radius + torch.rand(
                                (), generator=generator, device=args.device
                            ) * (maximum_radius - minimum_radius)
                            candidate = root_pose[row, :2] + radius * torch.stack(
                                (torch.cos(angle), torch.sin(angle))
                            )
                            if bool(
                                torch.all(
                                    (candidate >= lower_workspace[row])
                                    & (candidate <= upper_workspace[row])
                                ).item()
                            ):
                                result[row, :2] = candidate
                                break
                        else:
                            # This only occurs when the robot is already too
                            # far outside its workspace; workspace-exit reset
                            # will place it back inside on the following path.
                            result[row, :2] = torch.clamp(
                                root_pose[row, :2],
                                lower_workspace[row],
                                upper_workspace[row],
                            )
                minimum_distance = float(args.base_goal_minimum_distance)
                if minimum_distance > 0.0:
                    for row in range(env_ids.numel()):
                        current_distance = torch.linalg.vector_norm(
                            result[row, :2] - root_pose[row, :2]
                        )
                        if float(current_distance.item()) >= minimum_distance:
                            continue
                        for _ in range(500):
                            candidate_offset = relative[:2, 0] + torch.rand(
                                2, generator=generator, device=args.device
                            ) * (relative[:2, 1] - relative[:2, 0])
                            candidate = root_pose[row, :2] + candidate_offset
                            if float(
                                torch.linalg.vector_norm(candidate_offset).item()
                            ) >= minimum_distance and bool(
                                torch.all(
                                    (candidate >= lower_workspace[row])
                                    & (candidate <= upper_workspace[row])
                                ).item()
                            ):
                                result[row, :2] = candidate
                                break
                        else:
                            raise ValueError(
                                "Could not sample a base goal satisfying "
                                f"base_goal_minimum_distance={minimum_distance:.3f}; "
                                "increase the base goal range or reduce the minimum."
                            )
                result[:, 2] = relative[2, 0] + torch.rand(
                    env_ids.numel(), generator=generator, device=args.device
                ) * (relative[2, 1] - relative[2, 0])
            else:
                result[:, :2] = torch.clamp(root_pose[:, :2], lower_workspace, upper_workspace)
            base_goals[env_ids] = result
            base_goal_offsets[env_ids] = result
            base_goal_offsets[env_ids, :2] -= root_pose[:, :2]
            direction = torch.randn(env_ids.numel(), 2, generator=generator, device=args.device)
            base_goal_directions[env_ids] = direction / torch.linalg.vector_norm(
                direction, dim=1, keepdim=True
            ).clamp_min(1.0e-8)

        all_goal_env_ids = torch.arange(args.num_envs, device=args.device)
        deterministic_scenario_sampling = not args.shared_obstacle_environment
        if not deterministic_scenario_sampling:
            sample_base_goal_rows(all_goal_env_ids, initial_root_pose)

        upper_body_target = agent.default_body_pos[:, 12:].clone()
        left_ee_targets = torch.eye(4, device=args.device, dtype=torch.float32).repeat(
            args.num_envs, 1, 1
        )
        right_ee_targets = left_ee_targets.clone()
        arm_random_states = [
            np.random.RandomState(args.seed + 100_003 * env_id) for env_id in range(args.num_envs)
        ]

        class _CpuDualArmIKAdapter:
            """Expose the numeric SPARK CPU IK through the tensor IK contract."""

            def __init__(self):
                self.kinematics = UnitreeG1DualArmKinematics(
                    UnitreeG1DualArmDynamic1Config(),
                    load_collision_geometry=False,
                )
                # The task samples positions only. Regularizing every joint
                # against the seed can create a false local compromise several
                # centimeters from an otherwise exactly reachable point.
                self.solver = BoundedLeastSquaresIK(
                    self.kinematics.reduced_fixed_base_model,
                    IKSolverConfig(
                        regularization_weight=0.0,
                        smoothness_weight=0.0,
                        max_evaluations=100,
                        position_tolerance=float(args.arm_goal_tolerance),
                    ),
                )

            def forward(self, configuration):
                rows = torch.as_tensor(configuration).detach().cpu().numpy().reshape(-1, 14)
                left, right = [], []
                for row in rows:
                    frames = self.kinematics.forward_kinematics(row)
                    left.append(frames[self.kinematics.robot_cfg.Frames.L_ee])
                    right.append(frames[self.kinematics.robot_cfg.Frames.R_ee])
                return (
                    torch.as_tensor(np.asarray(left), device=args.device, dtype=torch.float32),
                    torch.as_tensor(np.asarray(right), device=args.device, dtype=torch.float32),
                )

            def solve(self, left_ee, right_ee, current_configuration):
                left = torch.as_tensor(left_ee).detach().cpu().numpy().reshape(-1, 4, 4)
                right = torch.as_tensor(right_ee).detach().cpu().numpy().reshape(-1, 4, 4)
                current = (
                    torch.as_tensor(current_configuration).detach().cpu().numpy().reshape(-1, 14)
                )
                configurations, successes = [], []
                position_errors, rotation_errors = [], []
                solve_time = 0.0
                for left_target, right_target, seed in zip(left, right, current):
                    # Benchmark arm goals are Cartesian positions represented
                    # by spheres; they do not prescribe an orientation.  Do
                    # not inherit the legacy fixed-base API's identity-rotation
                    # objective, which rejects reachable position targets.
                    result = self.solver.solve(
                        IKProblem(
                            targets=(
                                IKTarget(
                                    self.kinematics.L_hand_id,
                                    left_target,
                                    orientation_mask=(False, False, False),
                                ),
                                IKTarget(
                                    self.kinematics.R_hand_id,
                                    right_target,
                                    orientation_mask=(False, False, False),
                                ),
                            ),
                            initial_configuration=seed,
                            previous_configuration=seed,
                        )
                    )
                    solution = result.configuration
                    configurations.append(solution if result.success else seed)
                    successes.append(result.success)
                    position_errors.append(result.position_error)
                    rotation_errors.append(result.orientation_error)
                    solve_time += result.solve_time
                return {
                    "configuration": torch.as_tensor(
                        np.asarray(configurations),
                        device=args.device,
                        dtype=torch.float32,
                    ),
                    "success": torch.as_tensor(
                        successes,
                        device=args.device,
                        dtype=torch.bool,
                    ),
                    "position_error": torch.as_tensor(
                        position_errors,
                        device=args.device,
                        dtype=torch.float32,
                    ),
                    "rotation_error": torch.as_tensor(
                        rotation_errors,
                        device=args.device,
                        dtype=torch.float32,
                    ),
                    "solve_time": solve_time,
                    "backend": "cpu_bounded_least_squares_unitree_g1_dual_arm",
                }

        class _SonicFixedBaseIKAdapter:
            """Use the same 17-DoF waist-and-arms IK contract as scalar Sonic."""

            includes_waist = True

            def __init__(self):
                self.kinematics = UnitreeG1FixedBaseKinematics(UnitreeG1FixedBaseDynamic1Config())

            def forward(self, configuration):
                rows = torch.as_tensor(configuration).detach().cpu().numpy().reshape(-1, 17)
                left, right = [], []
                for row in rows:
                    frames = self.kinematics.forward_kinematics(row)
                    left.append(frames[self.kinematics.robot_cfg.Frames.L_ee])
                    right.append(frames[self.kinematics.robot_cfg.Frames.R_ee])
                return (
                    torch.as_tensor(np.asarray(left), device=args.device, dtype=torch.float32),
                    torch.as_tensor(np.asarray(right), device=args.device, dtype=torch.float32),
                )

            def solve(self, left_ee, right_ee, current_configuration):
                left = torch.as_tensor(left_ee).detach().cpu().numpy().reshape(-1, 4, 4)
                right = torch.as_tensor(right_ee).detach().cpu().numpy().reshape(-1, 4, 4)
                current = (
                    torch.as_tensor(current_configuration).detach().cpu().numpy().reshape(-1, 17)
                )
                configurations, successes = [], []
                position_errors, rotation_errors = [], []
                solve_time = 0.0
                for left_target, right_target, seed in zip(left, right, current):
                    solution = seed
                    info = {}
                    result = None
                    row_solve_time = 0.0
                    # Scalar Sonic resolves this same static target every
                    # control cycle and warm-starts from the previous answer.
                    # Collapse those first few refinement calls into reset so
                    # tensor inference remains fully batched afterward.
                    for _ in range(8):
                        started = time.perf_counter()
                        solution, info = self.kinematics.inverse_kinematics(
                            [right_target, left_target], solution, None
                        )
                        elapsed = time.perf_counter() - started
                        result = info.get("ik_result")
                        row_solve_time += float(getattr(result, "solve_time", elapsed))
                        if bool(info.get("success", False)):
                            break
                    configurations.append(np.asarray(solution, dtype=float))
                    successes.append(bool(info.get("success", True)))
                    position_errors.append(float(getattr(result, "position_error", 0.0)))
                    rotation_errors.append(float(getattr(result, "orientation_error", 0.0)))
                    solve_time += row_solve_time
                return {
                    "configuration": torch.as_tensor(
                        np.asarray(configurations),
                        device=args.device,
                        dtype=torch.float32,
                    ),
                    "success": torch.as_tensor(successes, device=args.device, dtype=torch.bool),
                    "position_error": torch.as_tensor(
                        position_errors, device=args.device, dtype=torch.float32
                    ),
                    "rotation_error": torch.as_tensor(
                        rotation_errors, device=args.device, dtype=torch.float32
                    ),
                    "solve_time": solve_time,
                    "backend": "scalar_sonic_fixed_base_waist_dual_arm",
                }

        # Base-only tasks do not need arm kinematics. Sonic must use the same
        # fixed-base waist-and-arms target generator as its scalar adapter;
        # other CUDA policies retain the batched cuRobo solver.
        arm_kinematics_enabled = args.arm_goal_mode != "default"
        ik_solver = None
        ik_backend_name = "disabled (base-only task)"
        if arm_kinematics_enabled:
            if args.policy == "sonic":
                ik_solver = _SonicFixedBaseIKAdapter()
                ik_backend_name = "scalar_sonic_fixed_base_waist_dual_arm"
            elif str(args.device).startswith("cuda"):
                ik_solver = UnitreeG1CuroboDualArmIK(
                    max_batch_size=args.num_envs,
                    device=args.device,
                    num_seeds=args.ik_num_seeds,
                    self_collision_check=args.enable_self_collision,
                )
                ik_backend_name = "curobo_gpu_unitree_g1_dual_arm"
            else:
                ik_solver = _CpuDualArmIKAdapter()
                ik_backend_name = "cpu_bounded_least_squares_unitree_g1_dual_arm"
        ik_includes_waist = bool(getattr(ik_solver, "includes_waist", False))
        latest_ik_success = torch.ones(args.num_envs, device=args.device, dtype=torch.bool)
        total_ik_time = 0.0
        total_ik_queries = 0

        def sample_and_solve_arm_goals(
            env_ids,
            feedback,
            *,
            sampled_left_frames=None,
            sampled_right_frames=None,
        ):
            """Use BenchmarkTask Cartesian sampling and solve only selected rows."""
            nonlocal total_ik_time, total_ik_queries
            if env_ids.numel() == 0:
                return
            if not arm_kinematics_enabled:
                upper_body_target[env_ids] = feedback["body_joint_pos"][env_ids, 12:]
                latest_ik_success[env_ids] = True
                return
            ids = env_ids.detach().cpu().tolist()
            left_goal_range = (
                DEFAULT_LEFT_ARM_GOAL_RANGE
                if args.left_arm_goal_range is None
                else np.asarray(args.left_arm_goal_range, dtype=float).reshape(3, 2)
            )
            right_goal_range = (
                DEFAULT_RIGHT_ARM_GOAL_RANGE
                if args.right_arm_goal_range is None
                else np.asarray(args.right_arm_goal_range, dtype=float).reshape(3, 2)
            )
            supplied_scenario = sampled_left_frames is not None
            if supplied_scenario:
                left_frames = np.asarray(sampled_left_frames, dtype=float).reshape(-1, 4, 4)
                right_frames = np.asarray(sampled_right_frames, dtype=float).reshape(-1, 4, 4)
            else:
                left_frames = np.repeat(np.eye(4)[None], len(ids), axis=0)
                right_frames = np.repeat(np.eye(4)[None], len(ids), axis=0)
                for row, env_id in enumerate(ids):
                    if args.arm_goal_mode == "fixed":
                        left_frames[row, :3, 3] = (0.3, 0.25, 0.0)
                        right_frames[row, :3, 3] = (0.3, -0.25, 0.0)
                    elif args.arm_goal_mode == "random":
                        left_frames[row, :3, 3] = sample_bounded_position(
                            arm_random_states[env_id], left_goal_range
                        )
                        right_frames[row, :3, 3] = sample_bounded_position(
                            arm_random_states[env_id], right_goal_range
                        )
                    else:
                        # Default mode retains the reset configuration and its FK
                        # pose, avoiding a synthetic Cartesian motion request.
                        current_arm = feedback["body_joint_pos"][
                            env_ids, 12 if ik_includes_waist else 15 :
                        ]
                        left_default, right_default = ik_solver.forward(current_arm)
                        left_ee_targets[env_ids] = left_default
                        right_ee_targets[env_ids] = right_default
                        upper_body_target[env_ids] = feedback["body_joint_pos"][env_ids, 12:]
                        latest_ik_success[env_ids] = True
                        return

            left_tensor = torch.as_tensor(left_frames, device=args.device, dtype=torch.float32)
            right_tensor = torch.as_tensor(right_frames, device=args.device, dtype=torch.float32)
            current_arm = feedback["body_joint_pos"][env_ids, 12 if ik_includes_waist else 15 :]
            solution = current_arm.clone()
            success = torch.zeros(len(ids), device=args.device, dtype=torch.bool)
            pending = torch.arange(len(ids), device=args.device)
            max_attempts = 1 if supplied_scenario else (20 if args.arm_goal_mode == "random" else 1)
            for _ in range(max_attempts):
                result = ik_solver.solve(
                    left_tensor[pending], right_tensor[pending], current_arm[pending]
                )
                solved_rows = pending[result["success"]]
                solution[solved_rows] = result["configuration"][result["success"]]
                success[solved_rows] = True
                total_ik_time += float(result["solve_time"])
                total_ik_queries += int(pending.numel())
                pending = pending[~result["success"]]
                if pending.numel() == 0 or args.arm_goal_mode != "random" or supplied_scenario:
                    break
                # Keep the task distribution identical to BenchmarkTask, but
                # reject poses for which the selected IK backend cannot
                # produce a valid target. This prevents impossible episodes.
                for row in pending.detach().cpu().tolist():
                    env_id = ids[row]
                    left_tensor[row, :3, 3] = torch.as_tensor(
                        sample_bounded_position(arm_random_states[env_id], left_goal_range),
                        device=args.device,
                        dtype=torch.float32,
                    )
                    right_tensor[row, :3, 3] = torch.as_tensor(
                        sample_bounded_position(arm_random_states[env_id], right_goal_range),
                        device=args.device,
                        dtype=torch.float32,
                    )
            left_ee_targets[env_ids] = left_tensor
            right_ee_targets[env_ids] = right_tensor
            latest_ik_success[env_ids] = success
            if ik_includes_waist:
                upper_body_target[env_ids] = solution
            else:
                upper_body_target[env_ids, :3] = agent.default_body_pos[env_ids, 12:15]
                upper_body_target[env_ids, 3:] = solution
            if not bool(torch.all(success).item()):
                failed = [ids[row] for row in torch.nonzero(~success).flatten().cpu().tolist()]
                print(f"[SPARK] IK failed after resampling for environments {failed}", flush=True)

        all_env_ids = torch.arange(args.num_envs, device=args.device)
        if not deterministic_scenario_sampling:
            sample_and_solve_arm_goals(all_env_ids, initial_feedback)

        def refresh_environment_geometry(env_ids=None, root_positions=None):
            """Update the selected distance representation from obstacle poses."""
            if not safety_enabled or not args.num_obstacles:
                return
            if env_ids is None:
                env_ids = all_env_ids
            if args.shared_obstacle_environment:
                if root_positions is None:
                    root_positions = agent.get_feedback()["root_pose_w"][env_ids, :3]
                else:
                    root_positions = root_positions[env_ids]
                shared = obstacle_positions[0]
                distance = torch.linalg.vector_norm(
                    root_positions[:, None, :] - shared[None, :, :], dim=-1
                )
                capacity = min(local_obstacle_capacity, shared.shape[0])
                nearest_distance, nearest_ids = torch.topk(
                    distance, k=capacity, dim=1, largest=False
                )
                selected = shared[nearest_ids]
                selected_valid = nearest_distance <= float(args.obstacle_query_range)
                if args.environment_representation == "sphere":
                    obstacle_cloud.positions[env_ids] = selected
                    obstacle_cloud.valid_mask[env_ids] = selected_valid
                elif args.environment_representation == "point_cloud":
                    sampled = selected[:, :, None, :] + point_surface_offsets[None, None]
                    obstacle_cloud.positions[env_ids] = sampled.reshape(env_ids.numel(), -1, 3)
                    point_valid = selected_valid[:, :, None].expand(
                        -1, -1, int(args.points_per_obstacle)
                    )
                    if args.dynamic_point_count:
                        minimum = min(
                            max(1, int(args.minimum_points_per_obstacle)),
                            int(args.points_per_obstacle),
                        )
                        counts = torch.randint(
                            minimum,
                            int(args.points_per_obstacle) + 1,
                            (env_ids.numel(), capacity),
                            generator=generator,
                            device=args.device,
                        )
                        point_index = torch.arange(args.points_per_obstacle, device=args.device)[
                            None, None
                        ]
                        # ``expand`` returns a zero-stride view; build a fresh
                        # mask instead of performing an aliased in-place write.
                        point_valid = point_valid & (point_index < counts[:, :, None])
                    obstacle_cloud.valid_mask[env_ids] = point_valid.reshape(env_ids.numel(), -1)
                return
            origins = agent.scene.env_origins[env_ids, None, :]
            centers = obstacle_positions[env_ids] + origins
            if args.environment_representation == "sphere":
                obstacle_cloud.positions[env_ids] = centers
            elif args.environment_representation == "point_cloud":
                sampled = centers[:, :, None, :] + point_surface_offsets[None, None]
                obstacle_cloud.positions[env_ids] = sampled.reshape(env_ids.numel(), -1, 3)
                if args.dynamic_point_count:
                    minimum = min(
                        max(1, int(args.minimum_points_per_obstacle)),
                        int(args.points_per_obstacle),
                    )
                    counts = torch.randint(
                        minimum,
                        int(args.points_per_obstacle) + 1,
                        (env_ids.numel(), obstacle_count),
                        generator=generator,
                        device=args.device,
                    )
                    point_index = torch.arange(args.points_per_obstacle, device=args.device)[
                        None, None
                    ]
                    obstacle_cloud.valid_mask[env_ids] = (point_index < counts[:, :, None]).reshape(
                        env_ids.numel(), -1
                    )
                else:
                    obstacle_cloud.valid_mask[env_ids] = True
            else:
                sampled = centers[:, :, None, :] + mesh_surface_offsets[None, None]
                obstacle_mesh.vertices[env_ids] = sampled.reshape(env_ids.numel(), -1, 3)

        def sample_obstacles(env_ids, feedback):
            """Sample obstacles while keeping task goals collision-free."""
            if not args.num_obstacles or env_ids.numel() == 0:
                return
            if args.shared_obstacle_environment:
                # A shared world persists across individual environment
                # resets. Only the initial all-environment call samples it.
                if bool(torch.any(obstacle_directions).item()):
                    return
                world_lower = shared_workspace_bounds[:, 0]
                world_upper = shared_workspace_bounds[:, 1]
                root = feedback["root_pose_w"]
                qx, qy, qz, qw = root[:, 3:7].unbind(dim=-1)
                yaw = torch.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy.square() + qz.square()),
                )
                c, s = torch.cos(yaw), torch.sin(yaw)

                def arm_goal_world(target):
                    offset = target[:, :3, 3]
                    result = root[:, :3].clone()
                    result[:, 0] += c * offset[:, 0] - s * offset[:, 1]
                    result[:, 1] += s * offset[:, 0] + c * offset[:, 1]
                    result[:, 2] += offset[:, 2]
                    return result

                base_goal_world = torch.cat((base_goals[:, :2], root[:, 2:3]), dim=1)
                protected = [
                    base_goal_world,
                    arm_goal_world(left_ee_targets),
                    arm_goal_world(right_ee_targets),
                ]
                protected_radii = [
                    torch.full(
                        (args.num_envs,),
                        float(
                            args.base_goal_tolerance
                            + args.obstacle_goal_keepaway
                            + args.obstacle_radius
                        ),
                        device=args.device,
                    ),
                    torch.full(
                        (args.num_envs,),
                        float(
                            args.arm_goal_tolerance
                            + args.obstacle_goal_keepaway
                            + args.obstacle_radius
                        ),
                        device=args.device,
                    ),
                    torch.full(
                        (args.num_envs,),
                        float(
                            args.arm_goal_tolerance
                            + args.obstacle_goal_keepaway
                            + args.obstacle_radius
                        ),
                        device=args.device,
                    ),
                ]
                if collision_model is not None:
                    robot_centers, _ = sphere_centers_and_offsets()
                    environment_mask = collision_model.environment_mask
                    protected.append(robot_centers[:, environment_mask].reshape(-1, 3))
                    protected_radii.append(
                        (
                            collision_model.radii[environment_mask]
                            + float(args.obstacle_radius + args.obstacle_robot_keepaway)
                        ).repeat(args.num_envs)
                    )
                protected = torch.cat([item.reshape(-1, 3) for item in protected], dim=0)
                protected_radii = torch.cat(protected_radii, dim=0)
                selected = obstacle_positions.clone()
                valid = torch.zeros(1, obstacle_count, device=args.device, dtype=torch.bool)
                for _ in range(100):
                    pending = ~valid
                    if not bool(torch.any(pending).item()):
                        break
                    candidates = world_lower + torch.rand(
                        1,
                        obstacle_count,
                        3,
                        generator=generator,
                        device=args.device,
                    ) * (world_upper - world_lower)
                    accepted = pending & torch.all(
                        torch.linalg.vector_norm(
                            candidates[:, :, None] - protected[None, None], dim=-1
                        )
                        >= protected_radii[None, None],
                        dim=-1,
                    )
                    selected[accepted] = candidates[accepted]
                    valid |= accepted
                if not bool(torch.all(valid).item()):
                    raise RuntimeError(
                        "Could not sample the shared obstacle environment with "
                        "the requested robot/goal keepaway distances"
                    )
                obstacle_positions[0] = selected[0]
                direction = torch.randn(
                    1,
                    obstacle_count,
                    3,
                    generator=generator,
                    device=args.device,
                )
                obstacle_directions[:] = direction / torch.linalg.vector_norm(
                    direction, dim=-1, keepdim=True
                ).clamp_min(1.0e-8)
                refresh_environment_geometry(root_positions=feedback["root_pose_w"][:, :3])
                if render_enabled and args.environment_representation == "sphere":
                    agent.set_visual_shared_obstacles(
                        obstacle_positions[0].detach().cpu().numpy(),
                        args.obstacle_radius,
                    )
                return
            origins = agent.scene.env_origins[env_ids]
            root = feedback["root_pose_w"][env_ids]
            qx, qy, qz, qw = root[:, 3:7].unbind(dim=-1)
            yaw = torch.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy.square() + qz.square()),
            )
            c, s = torch.cos(yaw), torch.sin(yaw)

            def arm_goal_local(target):
                offset = target[env_ids, :3, 3]
                goal = root[:, :3] - origins
                goal = goal.clone()
                goal[:, 0] += c * offset[:, 0] - s * offset[:, 1]
                goal[:, 1] += s * offset[:, 0] + c * offset[:, 1]
                goal[:, 2] += offset[:, 2]
                return goal

            base_goal_local = torch.cat(
                (
                    base_goals[env_ids, :2] - origins[:, :2],
                    (root[:, 2:3] - origins[:, 2:3]),
                ),
                dim=1,
            )
            goals = torch.stack(
                (
                    base_goal_local,
                    arm_goal_local(left_ee_targets),
                    arm_goal_local(right_ee_targets),
                ),
                dim=1,
            )
            goal_radii = torch.tensor(
                (
                    args.base_goal_tolerance,
                    args.arm_goal_tolerance,
                    args.arm_goal_tolerance,
                ),
                device=args.device,
            )
            required = goal_radii + float(args.obstacle_goal_keepaway + args.obstacle_radius)
            robot_centers = robot_required = None
            if collision_model is not None:
                robot_centers, _ = sphere_centers_and_offsets()
                robot_centers = robot_centers[env_ids] - origins[:, None]
                robot_radii = collision_model.radii[collision_model.environment_mask]
                robot_centers = robot_centers[:, collision_model.environment_mask]
                robot_required = robot_radii + float(
                    args.obstacle_radius + args.obstacle_robot_keepaway
                )
            selected = obstacle_positions[env_ids].clone()
            valid = torch.zeros(
                env_ids.numel(),
                obstacle_count,
                device=args.device,
                dtype=torch.bool,
            )
            for _ in range(100):
                pending = ~valid
                if not bool(torch.any(pending).item()):
                    break
                candidates = obstacle_bounds[:, 0] + torch.rand(
                    env_ids.numel(),
                    obstacle_count,
                    3,
                    generator=generator,
                    device=args.device,
                ) * (obstacle_bounds[:, 1] - obstacle_bounds[:, 0])
                accepted = pending & torch.all(
                    torch.linalg.vector_norm(candidates[:, :, None] - goals[:, None], dim=-1)
                    >= required[None, None],
                    dim=-1,
                )
                if robot_centers is not None:
                    accepted &= torch.all(
                        torch.linalg.vector_norm(
                            candidates[:, :, None] - robot_centers[:, None], dim=-1
                        )
                        >= robot_required[None, None],
                        dim=-1,
                    )
                selected[accepted] = candidates[accepted]
                valid |= accepted
            if not bool(torch.all(valid).item()):
                raise RuntimeError(
                    "Could not sample obstacles with the requested goal keepaway; "
                    "increase --obstacle-range or reduce --obstacle-goal-keepaway"
                )
            obstacle_positions[env_ids] = selected
            direction = torch.randn(
                env_ids.numel(),
                obstacle_count,
                3,
                generator=generator,
                device=args.device,
            )
            obstacle_directions[env_ids] = direction / torch.linalg.vector_norm(
                direction, dim=-1, keepdim=True
            ).clamp_min(1.0e-8)
            refresh_environment_geometry(env_ids)
            if render_enabled and args.environment_representation == "sphere":
                if args.shared_obstacle_environment:
                    agent.set_visual_shared_obstacles(
                        obstacle_positions[0].detach().cpu().numpy(),
                        args.obstacle_radius,
                    )
                else:
                    agent.set_visual_obstacles(
                        obstacle_positions.detach().cpu().numpy(), args.obstacle_radius
                    )

        scenario_episode_indices = np.zeros(args.num_envs, dtype=np.int64)
        latest_scenario_fingerprints = [None] * args.num_envs
        canonical_kinematics = UnitreeG1WholeBodyKinematics(robot_cfg)
        canonical_dof = np.asarray(
            [robot_cfg.DefaultDoFVal[dof] for dof in robot_cfg.DoFs], dtype=float
        )
        canonical_frames = canonical_kinematics.forward_kinematics(canonical_dof)
        canonical_root_position = canonical_dof[:3].copy()
        canonical_root_yaw = 0.0
        canonical_collision_centers = np.asarray(
            [canonical_frames[int(frame_id), :3, 3] for frame_id in robot_cfg.CollisionVol],
            dtype=float,
        )
        canonical_collision_radii = np.asarray(
            [float(geometry.size[0]) for geometry in robot_cfg.CollisionVol.values()],
            dtype=float,
        )
        canonical_right_start = canonical_frames[robot_cfg.Frames.R_ee, :3, 3]
        canonical_left_start = canonical_frames[robot_cfg.Frames.L_ee, :3, 3]

        def sample_deterministic_scenario_rows(env_ids, feedback):
            """Apply the same NumPy reset contract used by scalar MuJoCo."""

            ids = env_ids.detach().cpu().tolist()
            root_pose = feedback["root_pose_w"][env_ids].detach().cpu().numpy()
            origins = agent.scene.env_origins[env_ids].detach().cpu().numpy()
            left_frames = np.repeat(np.eye(4)[None], len(ids), axis=0)
            right_frames = np.repeat(np.eye(4)[None], len(ids), axis=0)
            for row, env_id in enumerate(ids):
                local_root = canonical_root_position
                root_yaw = canonical_root_yaw
                cosine, sine = np.cos(root_yaw), np.sin(root_yaw)

                def start_world(local):
                    return local_root + np.array(
                        [
                            cosine * local[0] - sine * local[1],
                            sine * local[0] + cosine * local[1],
                            local[2],
                        ]
                    )

                scenario = sample_benchmark_scenario(
                    seed=benchmark_scenario_seed(
                        args.seed,
                        environment_index=env_id,
                        episode_index=int(scenario_episode_indices[env_id]),
                    ),
                    root_position=local_root,
                    root_yaw=root_yaw,
                    arm_goal_enabled=arm_kinematics_enabled,
                    dual_arm=True,
                    right_arm_goal_range=(
                        DEFAULT_RIGHT_ARM_GOAL_RANGE
                        if args.right_arm_goal_range is None
                        else np.asarray(args.right_arm_goal_range).reshape(3, 2)
                    ),
                    left_arm_goal_range=(
                        DEFAULT_LEFT_ARM_GOAL_RANGE
                        if args.left_arm_goal_range is None
                        else np.asarray(args.left_arm_goal_range).reshape(3, 2)
                    ),
                    right_arm_start=start_world(canonical_right_start),
                    left_arm_start=start_world(canonical_left_start),
                    arm_goal_minimum_distance=args.arm_goal_minimum_distance,
                    arm_goal_pair_keepout=args.arm_goal_pair_keepout,
                    base_goal_enabled=args.goal_mode != "velocity",
                    base_goal_range=(
                        tuple(args.base_goal_range[:2]),
                        tuple(args.base_goal_range[2:4]),
                        tuple(args.base_goal_height_range),
                    ),
                    base_goal_rot_range=tuple(args.base_goal_range[4:6]),
                    base_goal_relative_to_current=args.base_goal_relative_to_current,
                    base_goal_workspace_range=(
                        None
                        if args.base_goal_workspace_range is None
                        else np.asarray(args.base_goal_workspace_range).reshape(2, 2)
                    ),
                    base_goal_minimum_distance=args.base_goal_minimum_distance,
                    num_obstacles=args.num_obstacles,
                    obstacle_range=np.asarray(args.obstacle_range).reshape(3, 2),
                    obstacle_radius=args.obstacle_radius,
                    obstacle_keepout=args.obstacle_keepout,
                    obstacle_robot_keepaway=args.obstacle_robot_keepaway,
                    obstacle_goal_keepaway=args.obstacle_goal_keepaway,
                    arm_goal_radius=args.arm_goal_tolerance,
                    base_goal_radius=args.base_goal_tolerance,
                    robot_collision_centers=canonical_collision_centers,
                    robot_collision_radii=canonical_collision_radii,
                )
                latest_scenario_fingerprints[env_id] = benchmark_scenario_fingerprint(scenario)
                if scenario.base_goal is not None:
                    base_goals[env_id] = torch.as_tensor(
                        (
                            scenario.base_goal[0] + origins[row, 0],
                            scenario.base_goal[1] + origins[row, 1],
                            scenario.base_goal[3],
                        ),
                        device=args.device,
                    )
                    base_goal_offsets[env_id] = base_goals[env_id]
                    base_goal_offsets[env_id, :2] -= feedback["root_pose_w"][env_id, :2]
                if scenario.right_arm_goal is not None:
                    right_frames[row, :3, 3] = scenario.right_arm_goal
                    left_frames[row, :3, 3] = scenario.left_arm_goal
                if args.num_obstacles:
                    obstacle_positions[env_id] = torch.as_tensor(
                        scenario.obstacle_positions, device=args.device, dtype=torch.float32
                    )
                    direction_rng = np.random.RandomState(scenario.seed + 17)
                    direction = direction_rng.normal(size=(args.num_obstacles, 3))
                    direction /= np.maximum(np.linalg.norm(direction, axis=1, keepdims=True), 1e-8)
                    obstacle_directions[env_id] = torch.as_tensor(
                        direction, device=args.device, dtype=torch.float32
                    )
                print(
                    f"[SPARK] benchmark scenario env={env_id} seed={scenario.seed} "
                    f"fingerprint={latest_scenario_fingerprints[env_id]}",
                    flush=True,
                )
            if arm_kinematics_enabled:
                sample_and_solve_arm_goals(
                    env_ids,
                    feedback,
                    sampled_left_frames=left_frames,
                    sampled_right_frames=right_frames,
                )
            else:
                sample_and_solve_arm_goals(env_ids, feedback)
            if args.num_obstacles:
                refresh_environment_geometry(env_ids)

        if deterministic_scenario_sampling:
            sample_deterministic_scenario_rows(all_env_ids, initial_feedback)
        else:
            sample_obstacles(all_env_ids, initial_feedback)

        def update_base_goals():
            if args.base_goal_velocity <= 0.0 or args.goal_mode != "random":
                return
            distance = float(args.base_goal_velocity * args.dt * args.control_decimation)
            base_goals[:, :2].add_(base_goal_directions * distance)
            env_ids = torch.arange(args.num_envs, device=args.device)
            lower, upper = workspace_bounds(env_ids)
            for axis in range(2):
                below = base_goals[:, axis] < lower[:, axis]
                above = base_goals[:, axis] > upper[:, axis]
                reflected = below | above
                base_goal_directions[:, axis] = torch.where(
                    reflected,
                    -base_goal_directions[:, axis],
                    base_goal_directions[:, axis],
                )
                base_goals[:, axis] = torch.maximum(
                    torch.minimum(base_goals[:, axis], upper[:, axis]),
                    lower[:, axis],
                )

        def update_obstacles():
            if not args.num_obstacles or args.obstacle_velocity <= 0.0:
                return
            step_distance = float(args.obstacle_velocity * args.dt * args.control_decimation)
            obstacle_positions.add_(obstacle_directions * step_distance)
            for axis in range(3):
                if args.shared_obstacle_environment:
                    lower = shared_workspace_bounds[axis, 0]
                    upper = shared_workspace_bounds[axis, 1]
                else:
                    lower, upper = obstacle_bounds[axis]
                below = obstacle_positions[..., axis] < lower
                above = obstacle_positions[..., axis] > upper
                reflected = below | above
                obstacle_directions[..., axis] = torch.where(
                    reflected,
                    -obstacle_directions[..., axis],
                    obstacle_directions[..., axis],
                )
                obstacle_positions[..., axis].clamp_(lower, upper)
            if not args.shared_obstacle_environment:
                refresh_environment_geometry()

        episode_steps = torch.zeros(args.num_envs, device=args.device, dtype=torch.long)
        # Unlike task episode time, this clock resets only with simulator
        # state. A newly sampled goal in a continuous scene must not put WBT
        # back through its cold-start standing warmup.
        physical_motion_steps = torch.zeros_like(episode_steps)
        episode_counts = torch.zeros_like(episode_steps)
        successful_resets = 0
        fall_resets = 0
        timeout_resets = 0
        workspace_exit_resets = 0

        update_visual_goals = None
        if render_enabled:
            env_origins = agent.scene.env_origins.detach().cpu().numpy()

            def update_visual_goals(feedback):
                """Render current tensor goals using one consistent local frame."""
                root_pose = feedback["root_pose_w"].detach().cpu().numpy()
                root_local = root_pose[:, :3] - env_origins
                qx, qy, qz, qw = root_pose[:, 3:7].T
                yaw = np.arctan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz),
                )

                def to_world_relative(offsets):
                    c, s = np.cos(yaw), np.sin(yaw)
                    positions = root_local.copy()
                    positions[:, 0] += c * offsets[:, 0] - s * offsets[:, 1]
                    positions[:, 1] += s * offsets[:, 0] + c * offsets[:, 1]
                    positions[:, 2] += offsets[:, 2]
                    return positions

                # base_goals is world-space and changes independently for
                # each row after success/reset. Do not cache its startup
                # offset: that left completed environments displaying the old
                # marker indefinitely.
                base_markers = None
                if args.goal_mode != "velocity":
                    base_markers = np.empty((args.num_envs, 3), dtype=float)
                    base_markers[:, :2] = (
                        base_goals[:, :2].detach().cpu().numpy() - env_origins[:, :2]
                    )
                    base_markers[:, 2] = base_goal_marker_heights
                left_arm_markers = None
                right_arm_markers = None
                if arm_kinematics_enabled:
                    left_arm_markers = to_world_relative(
                        left_ee_targets[:, :3, 3].detach().cpu().numpy()
                    )
                    right_arm_markers = to_world_relative(
                        right_ee_targets[:, :3, 3].detach().cpu().numpy()
                    )
                agent.set_visual_goals(
                    base_markers,
                    left_arm_markers,
                    right_arm_markers,
                    base_radius=args.base_goal_tolerance,
                    arm_radius=args.arm_goal_tolerance,
                )

            update_visual_goals(initial_feedback)

        latest_position_error = torch.zeros(args.num_envs, device=args.device)
        latest_yaw_error = torch.zeros_like(latest_position_error)
        # A learned gait must be allowed to brake before handing control to the
        # stationary squat expert.  Pose-only switching can occur mid-step and
        # was the main source of late falls near a base goal in batched WBT.
        base_settle_count = torch.zeros(args.num_envs, device=args.device, dtype=torch.long)
        base_settled = torch.zeros(args.num_envs, device=args.device, dtype=torch.bool)
        base_translation_hold = torch.zeros_like(base_settled)
        base_best_position_error = torch.full_like(latest_position_error, float("inf"))
        base_no_progress_count = torch.zeros_like(base_settle_count)
        base_recovery_steps = torch.zeros_like(base_settle_count)

        def update_goal_command(feedback):
            nonlocal velocity_command, latest_position_error, latest_yaw_error
            if args.goal_mode == "velocity":
                return
            pose = feedback["root_pose_w"]
            qx, qy, qz, qw = pose[:, 3:7].unbind(dim=-1)
            yaw = torch.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy.square() + qz.square()),
            )
            error_world = base_goals[:, :2] - pose[:, :2]
            c, s = torch.cos(yaw), torch.sin(yaw)
            error_body = torch.stack(
                (
                    c * error_world[:, 0] + s * error_world[:, 1],
                    -s * error_world[:, 0] + c * error_world[:, 1],
                ),
                dim=-1,
            )
            velocity_world = feedback["root_velocity_w"][:, :2]
            velocity_body = torch.stack(
                (
                    c * velocity_world[:, 0] + s * velocity_world[:, 1],
                    -s * velocity_world[:, 0] + c * velocity_world[:, 1],
                ),
                dim=-1,
            )
            yaw_error = torch.atan2(
                torch.sin(base_goals[:, 2] - yaw),
                torch.cos(base_goals[:, 2] - yaw),
            )
            latest_position_error = torch.linalg.vector_norm(error_world, dim=-1)
            latest_yaw_error = yaw_error.abs()
            if args.policy in {"wbt", "sonic", "sport"}:
                linear_limit = torch.tensor(
                    (args.wbt_max_forward_speed, args.wbt_max_lateral_speed),
                    device=args.device,
                )
                # The learned locomotion plant has tracking lag. Linearly
                # reduce its admissible command over the final two goal radii
                # so it can enter squat/stand mode without overshooting.
                approach_scale = torch.clamp(
                    latest_position_error / max(2.0 * args.base_goal_tolerance, 1.0e-6),
                    0.0,
                    1.0,
                )
                limit = approach_scale[:, None] * linear_limit[None]
                velocity_command[:, :2] = torch.maximum(
                    torch.minimum(
                        args.base_position_kp * error_body - args.base_velocity_kd * velocity_body,
                        limit,
                    ),
                    -limit,
                )
                if args.policy == "sonic":
                    # Scalar Sonic applies a radial planar speed limit after
                    # PID. An axis-wise box changes diagonal direction and can
                    # exceed the policy's 0.12 m/s planar contract.
                    planar_norm = torch.linalg.vector_norm(
                        velocity_command[:, :2], dim=-1, keepdim=True
                    )
                    radial_scale = torch.clamp(
                        float(args.wbt_max_forward_speed) / planar_norm.clamp_min(1.0e-8),
                        max=1.0,
                    )
                    velocity_command[:, :2] *= radial_scale
                command_norm = torch.linalg.vector_norm(velocity_command[:, :2], dim=-1)
                needs_translation = latest_position_error > args.base_goal_tolerance
                minimum_speed = torch.minimum(
                    torch.full_like(command_norm, float(args.wbt_min_translation_speed)),
                    torch.linalg.vector_norm(limit, dim=-1),
                )
                boost = needs_translation & (command_norm < minimum_speed)
                direction = error_body / torch.linalg.vector_norm(
                    error_body, dim=-1, keepdim=True
                ).clamp_min(1.0e-8)
                velocity_command[:, :2] = torch.where(
                    boost[:, None],
                    direction * minimum_speed[:, None],
                    velocity_command[:, :2],
                )
                yaw_limit = float(args.wbt_max_yaw_rate)
            else:
                velocity_command[:, :2] = torch.clamp(
                    args.base_position_kp * error_body,
                    min=torch.tensor((-0.24, -0.18), device=args.device),
                    max=torch.tensor((0.24, 0.18), device=args.device),
                )
                yaw_limit = 0.5
            velocity_command[:, 2] = torch.clamp(
                args.base_yaw_kp * yaw_error
                - args.base_yaw_velocity_kd * feedback["root_angular_velocity_b"][:, 2],
                -yaw_limit,
                yaw_limit,
            )
            if args.policy in {"wbt", "sonic", "sport"}:
                needs_yaw = latest_yaw_error > args.base_yaw_tolerance
                weak_yaw = velocity_command[:, 2].abs() < float(args.wbt_min_yaw_rate)
                minimum_yaw = min(float(args.wbt_min_yaw_rate), yaw_limit)
                velocity_command[:, 2] = torch.where(
                    needs_yaw & weak_yaw,
                    torch.sign(yaw_error) * minimum_yaw,
                    velocity_command[:, 2],
                )

        def update_base_arrival_state(feedback):
            """Latch WBT arrival after sustained SE(2) goal residency."""
            if args.goal_mode == "velocity":
                return
            # Use scalar Sonic's tighter control deadband while retaining the
            # task case's larger success radius. This avoids intentionally
            # parking at the outer edge of a 0.15 m goal sphere.
            control_deadband = min(float(args.base_goal_tolerance), 0.12)
            resume_distance = max(
                control_deadband + 0.02,
                float(args.base_goal_tolerance) + 0.005,
            )
            enter_translation_hold = latest_position_error <= control_deadband
            leave_translation_hold = latest_position_error > resume_distance
            base_translation_hold.copy_(
                (base_translation_hold | enter_translation_hold) & ~leave_translation_hold
            )
            velocity_command[base_translation_hold, :2] = 0.0

            yaw_reached = latest_yaw_error <= float(args.base_yaw_tolerance)
            velocity_command[yaw_reached, 2] = 0.0
            pose_arrived = (latest_position_error <= float(args.base_goal_tolerance)) & yaw_reached
            base_settle_count.copy_(
                torch.where(
                    pose_arrived,
                    base_settle_count + 1,
                    torch.zeros_like(base_settle_count),
                )
            )
            base_settled.copy_(base_settle_count >= max(1, int(args.wbt_base_settle_steps)))

        def update_base_progress_recovery():
            """Restart Sonic's gait shaper if commanded motion stops progressing."""
            if args.policy != "sonic" or args.goal_mode == "velocity":
                return
            planar_intent = torch.linalg.vector_norm(velocity_command[:, :2], dim=-1) > 0.015
            active = (
                (latest_position_error > float(args.base_goal_tolerance))
                & planar_intent
                & (base_recovery_steps <= 0)
                & (physical_motion_steps >= int(args.warmup_steps))
            )
            improved = latest_position_error < (base_best_position_error - 0.01)
            base_best_position_error.copy_(
                torch.where(
                    active & improved,
                    latest_position_error,
                    torch.where(active, base_best_position_error, latest_position_error),
                )
            )
            base_no_progress_count.copy_(
                torch.where(
                    active & ~improved,
                    base_no_progress_count + 1,
                    torch.zeros_like(base_no_progress_count),
                )
            )
            stalled = base_no_progress_count >= 100
            if bool(torch.any(stalled).item()):
                env_ids = torch.nonzero(stalled).flatten()
                policy.reset_base_tracking(env_ids)
                base_recovery_steps[env_ids] = 25
                base_no_progress_count[env_ids] = 0
                base_best_position_error[env_ids] = latest_position_error[env_ids]
                print(
                    "[SPARK] Sonic base tracking recovery: "
                    + ", ".join(
                        f"env {env_id} error={latest_position_error[env_id].item():.3f}m"
                        for env_id in env_ids.detach().cpu().tolist()
                    ),
                    flush=True,
                )

        min_root_height = float("inf")
        max_root_tilt = 0.0
        max_root_angular_speed = 0.0

        upper_velocity_limit = torch.tensor(
            # Dual-arm IK owns only the 14 arm joints. Waist motion remains
            # under the learned locomotion stabilizer.
            [0.0, 0.0, 0.0] + [1.5] * 14,
            device=args.device,
            dtype=torch.float32,
        )
        safe_base_limit = (
            torch.tensor(
                (
                    args.wbt_max_forward_speed,
                    args.wbt_max_lateral_speed,
                    args.wbt_max_yaw_rate,
                ),
                device=args.device,
            )
            if args.policy in {"wbt", "sonic", "sport"}
            else torch.tensor((0.3, 0.3, 0.5), device=args.device)
        )
        safe_control_lower = -torch.cat(
            (
                upper_velocity_limit,
                safe_base_limit,
            )
        )[None, :]
        safe_control_upper = -safe_control_lower
        last_safe_base_command = torch.zeros(
            args.num_envs, 3, device=args.device, dtype=torch.float32
        )
        last_commanded_upper_velocity = torch.zeros(
            args.num_envs, 17, device=args.device, dtype=torch.float32
        )
        previous_sphere_jacobian = None
        filtered_sphere_jacobian_dot = None
        previous_sphere_jacobian_valid = torch.zeros(
            args.num_envs, device=args.device, dtype=torch.bool
        )

        def build_sphere_control_jacobian(feedback):
            """Map upper-joint and planar base commands to sphere velocities."""
            kin = agent.get_body_kinematics()
            centers, world_offset = sphere_centers_and_offsets()
            link_jacobian = kin["body_jacobian_w"][:, collision_model.body_ids]
            joint_ids = kin["body_joint_ids"][12:]
            # Floating articulations expose six base columns before public
            # joint columns. Fixed-base layouts contain joint columns only.
            offset = link_jacobian.shape[-1] - len(kin["joint_names"])
            joint_columns = offset + joint_ids
            jacobian = centers.new_zeros(centers.shape[0], centers.shape[1], 3, 20)
            linear = link_jacobian[:, :, :3, joint_columns]
            angular = link_jacobian[:, :, 3:, joint_columns]
            jacobian[:, :, :, :17] = linear + torch.cross(
                angular.movedim(-1, -2),
                world_offset[:, :, None, :],
                dim=-1,
            ).movedim(-2, -1)

            root_pose = feedback["root_pose_w"]
            qx, qy, qz, qw = root_pose[:, 3:7].unbind(dim=-1)
            yaw = torch.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy.square() + qz.square()),
            )
            c, s = torch.cos(yaw), torch.sin(yaw)
            jacobian[:, :, 0, 17] = c[:, None]
            jacobian[:, :, 1, 17] = s[:, None]
            jacobian[:, :, 0, 18] = -s[:, None]
            jacobian[:, :, 1, 18] = c[:, None]
            relative = centers - root_pose[:, None, :3]
            jacobian[:, :, 0, 19] = -relative[:, :, 1]
            jacobian[:, :, 1, 19] = relative[:, :, 0]
            return centers, jacobian

        def gather_query_jacobian(sphere_jacobian, ids):
            batch_ids = torch.arange(sphere_jacobian.shape[0], device=args.device)[:, None]
            return sphere_jacobian[batch_ids, ids]

        def apply_tensor_safety(feedback, requested_upper_target, requested_velocity):
            """Filter WBT references and return per-environment diagnostics."""
            nonlocal safety_minimum_distance, safety_max_residual
            nonlocal safety_environment_minimum_distance
            nonlocal safety_self_minimum_distance
            nonlocal latest_collision_centers, latest_safety_visualization
            nonlocal latest_collision_mask
            nonlocal latest_collision_distance
            nonlocal previous_sphere_jacobian
            nonlocal filtered_sphere_jacobian_dot
            requested_upper_target = requested_upper_target.clone()
            requested_upper_target[:, :3] = agent.default_body_pos[:, 12:15]
            if not safety_enabled and args.policy in {"wbt", "sonic", "sport"}:
                # Collision safety is optional, but learned locomotion command
                # shaping is part of the plant adaptation.  Previously this
                # slew limiter lived only in the active-safety branch, so a v0
                # bypass run stepped from zero directly to its minimum walking
                # speed while the otherwise identical v1 run ramped smoothly.
                # That discontinuity could tip WBT and made behavior depend on
                # the benchmark case rather than environment count.
                control_period = float(args.dt * args.control_decimation)
                rate = torch.tensor(
                    (
                        args.wbt_command_acceleration,
                        args.wbt_command_acceleration,
                        args.wbt_yaw_acceleration,
                    ),
                    device=args.device,
                )
                maximum_change = rate * control_period
                limited_velocity = last_safe_base_command + torch.clamp(
                    requested_velocity - last_safe_base_command,
                    -maximum_change,
                    maximum_change,
                )
                last_safe_base_command.copy_(limited_velocity)
                requested_velocity = limited_velocity
            if not safety_enabled and not args.render_robot_collision_volumes:
                return requested_upper_target, requested_velocity, None
            centers, sphere_jacobian = build_sphere_control_jacobian(feedback)
            control_period = float(args.dt * args.control_decimation)
            if previous_sphere_jacobian is None:
                sphere_jacobian_dot = torch.zeros_like(sphere_jacobian)
            else:
                sphere_jacobian_dot = (sphere_jacobian - previous_sphere_jacobian) / control_period
                sphere_jacobian_dot = torch.where(
                    previous_sphere_jacobian_valid[:, None, None, None],
                    sphere_jacobian_dot,
                    torch.zeros_like(sphere_jacobian_dot),
                )
            rate_limit = float(args.second_order_jacobian_rate_limit)
            sphere_jacobian_dot = torch.clamp(sphere_jacobian_dot, -rate_limit, rate_limit)
            rate_filter = float(np.clip(args.second_order_jacobian_rate_filter, 0.0, 1.0))
            if filtered_sphere_jacobian_dot is None:
                filtered_sphere_jacobian_dot = sphere_jacobian_dot
            else:
                filtered_sphere_jacobian_dot = filtered_sphere_jacobian_dot + rate_filter * (
                    sphere_jacobian_dot - filtered_sphere_jacobian_dot
                )
            sphere_jacobian_dot = filtered_sphere_jacobian_dot
            previous_sphere_jacobian = sphere_jacobian.detach().clone()
            previous_sphere_jacobian_valid.fill_(True)
            latest_collision_centers = centers
            if not safety_enabled:
                return requested_upper_target, requested_velocity, None
            if args.shared_obstacle_environment and args.num_obstacles:
                # Build a compact per-robot obstacle tensor before any
                # robot-link distance query. Complexity is B*M for the cheap
                # root-level spatial filter plus B*L*K for collision distance,
                # instead of B*L*M for every robot link and global obstacle.
                refresh_environment_geometry(root_positions=feedback["root_pose_w"][:, :3])
            if args.profile_safety_stages and args.device.startswith("cuda"):
                torch.cuda.synchronize()
            profile_mark = time.perf_counter()
            root_pose = feedback["root_pose_w"]
            qx, qy, qz, qw = root_pose[:, 3:7].unbind(dim=-1)
            yaw = torch.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy.square() + qz.square()),
            )
            root_velocity_w = feedback["root_velocity_w"]
            c, s = torch.cos(yaw), torch.sin(yaw)
            base_velocity = torch.stack(
                (
                    c * root_velocity_w[:, 0] + s * root_velocity_w[:, 1],
                    -s * root_velocity_w[:, 0] + c * root_velocity_w[:, 1],
                    feedback["root_angular_velocity_b"][:, 2],
                ),
                dim=1,
            )
            current_control_velocity = torch.cat(
                (feedback["body_joint_vel"][:, 12:], base_velocity), dim=1
            )
            if args.dynamics_order == 2:
                # Acceleration commands are integrated into the command state.
                # Kinematically stepped mobile bases do not expose a useful
                # measured root velocity, and learned locomotion velocity can
                # lag the command substantially, so neither is a valid
                # integrator state here.
                current_control_velocity[:, :17] = last_commanded_upper_velocity
                current_control_velocity[:, 17:] = last_safe_base_command
            index_build_kwargs = (
                {"control_velocity": current_control_velocity} if args.dynamics_order == 2 else {}
            )
            constraint_sets = []
            query_sets = []
            distance_sets = []
            latest_collision_mask = torch.zeros(
                centers.shape[:2], device=args.device, dtype=torch.uint8
            )
            latest_collision_distance = torch.full(
                centers.shape[:2], torch.inf, device=args.device, dtype=torch.float32
            )
            if args.num_obstacles:
                if args.environment_representation == "mesh":
                    env_query = collision_backend.query(
                        centers,
                        collision_model.radii,
                        obstacle_mesh,
                    )
                    # Mesh queries return one closest triangle per robot
                    # sphere. Apply the same configured environment mask used
                    # by primitive and point-cloud representations.
                    env_query.valid_mask &= collision_model.environment_mask[None]
                else:
                    env_query = collision_backend.query_environment_nearest(
                        centers,
                        collision_model.radii,
                        obstacle_cloud,
                        collision_model.environment_mask,
                        nearest_k=args.safety_nearest_points_per_link,
                        chunk_size=args.safety_point_chunk_size,
                    )
                env_jacobian = gather_query_jacobian(sphere_jacobian, env_query.robot_geometry_id)
                penetrating = env_query.valid_mask & (env_query.distance <= 0.0)
                latest_collision_mask.scatter_reduce_(
                    1,
                    env_query.robot_geometry_id,
                    penetrating.to(torch.uint8),
                    reduce="amax",
                    include_self=True,
                )
                latest_collision_distance.scatter_reduce_(
                    1,
                    env_query.robot_geometry_id,
                    torch.where(
                        env_query.valid_mask,
                        env_query.distance,
                        torch.full_like(env_query.distance, torch.inf),
                    ),
                    reduce="amin",
                    include_self=True,
                )
                env_jacobian_dot = gather_query_jacobian(
                    sphere_jacobian_dot, env_query.robot_geometry_id
                )
                constraint_sets.append(
                    safety_index.build(
                        env_query,
                        env_jacobian,
                        point_jacobian_dot=env_jacobian_dot,
                        **index_build_kwargs,
                    )
                    if args.dynamics_order == 2
                    else safety_index.build(env_query, env_jacobian)
                )
                query_sets.append(env_query)
                distance_sets.append(
                    torch.where(
                        env_query.valid_mask,
                        env_query.distance,
                        torch.full_like(env_query.distance, torch.inf),
                    )
                )
                environment_minimum = distance_sets[-1].min(dim=1).values
                safety_environment_minimum_distance = torch.minimum(
                    safety_environment_minimum_distance, environment_minimum
                )
            if args.enable_self_collision and collision_model.self_pair_i.numel():
                self_query = self_collision_backend.query_self(
                    centers,
                    collision_model.radii,
                    collision_model.self_pair_i,
                    collision_model.self_pair_j,
                )
                self_penetrating = self_query.valid_mask & (self_query.distance <= 0.0)
                latest_collision_mask.scatter_reduce_(
                    1,
                    self_query.robot_geometry_id,
                    self_penetrating.to(torch.uint8),
                    reduce="amax",
                    include_self=True,
                )
                self_distance = torch.where(
                    self_query.valid_mask,
                    self_query.distance,
                    torch.full_like(self_query.distance, torch.inf),
                )
                latest_collision_distance.scatter_reduce_(
                    1,
                    self_query.robot_geometry_id,
                    self_distance,
                    reduce="amin",
                    include_self=True,
                )
                latest_collision_distance.scatter_reduce_(
                    1,
                    self_query.environment_geometry_id,
                    self_distance,
                    reduce="amin",
                    include_self=True,
                )
                latest_collision_mask.scatter_reduce_(
                    1,
                    self_query.environment_geometry_id,
                    self_penetrating.to(torch.uint8),
                    reduce="amax",
                    include_self=True,
                )
                first_jacobian = gather_query_jacobian(
                    sphere_jacobian, self_query.robot_geometry_id
                )
                second_jacobian = gather_query_jacobian(
                    sphere_jacobian, self_query.environment_geometry_id
                )
                first_jacobian_dot = gather_query_jacobian(
                    sphere_jacobian_dot, self_query.robot_geometry_id
                )
                second_jacobian_dot = gather_query_jacobian(
                    sphere_jacobian_dot, self_query.environment_geometry_id
                )
                constraint_sets.append(
                    self_safety_index.build(
                        self_query,
                        first_jacobian,
                        other_point_jacobian=second_jacobian,
                        point_jacobian_dot=first_jacobian_dot,
                        other_point_jacobian_dot=second_jacobian_dot,
                        **index_build_kwargs,
                    )
                    if args.dynamics_order == 2
                    else self_safety_index.build(
                        self_query,
                        first_jacobian,
                        other_point_jacobian=second_jacobian,
                    )
                )
                query_sets.append(self_query)
                distance_sets.append(self_query.distance)
                self_minimum = self_query.distance.min(dim=1).values
                safety_self_minimum_distance = torch.minimum(
                    safety_self_minimum_distance, self_minimum
                )
            if args.profile_safety_stages:
                if args.device.startswith("cuda"):
                    torch.cuda.synchronize()
                now = time.perf_counter()
                safety_profile["query"] += now - profile_mark
                profile_mark = now
            if not constraint_sets:
                return requested_upper_target, requested_velocity, None
            constraints = TensorSafetyConstraints(
                A=torch.cat([item.A for item in constraint_sets], dim=1),
                lower=torch.cat([item.lower for item in constraint_sets], dim=1),
                distance=torch.cat([item.distance for item in constraint_sets], dim=1),
                active_mask=torch.cat([item.active_mask for item in constraint_sets], dim=1),
                source="combined",
            )
            current_upper = feedback["body_joint_pos"][:, 12:]
            upper_velocity = torch.clamp(
                (requested_upper_target - current_upper) / control_period,
                -upper_velocity_limit,
                upper_velocity_limit,
            )
            requested_control_velocity = torch.cat((upper_velocity, requested_velocity), dim=1)
            if args.dynamics_order == 2:
                reference = (requested_control_velocity - current_control_velocity) / control_period
                acceleration_limit = torch.cat(
                    (
                        torch.full_like(
                            upper_velocity_limit,
                            float(args.second_order_upper_acceleration_limit),
                        ),
                        torch.tensor(
                            (
                                args.wbt_command_acceleration,
                                args.wbt_command_acceleration,
                                args.wbt_yaw_acceleration,
                            ),
                            device=args.device,
                        ),
                    )
                )[None, :]
                filter_lower = -acceleration_limit
                filter_upper = acceleration_limit
                reference = torch.maximum(torch.minimum(reference, filter_upper), filter_lower)
            else:
                reference = requested_control_velocity
                filter_lower = safe_control_lower
                filter_upper = safe_control_upper
            # Waist correction is intentionally unavailable to the tensor
            # safety controller. Remove it from the nominal reference before
            # filtering as well; otherwise measured waist velocity makes D2
            # report a safety intervention on every step.
            reference[:, :3] = 0.0
            if args.profile_safety_stages:
                if args.device.startswith("cuda"):
                    torch.cuda.synchronize()
                now = time.perf_counter()
                safety_profile["constraints"] += now - profile_mark
                profile_mark = now
            safe, info = safety_filter.filter(
                reference,
                constraints,
                lower_limit=filter_lower,
                upper_limit=filter_upper,
            )
            # The tensor safety controller shares a 17-DOF upper-body vector
            # with WBT for compatibility, but its first three waist entries
            # are deliberately disabled. Collision avoidance may use either
            # arm or locomotion, never an independent waist IK correction.
            safe[:, :3] = 0.0
            constraint_safe = safe
            nominal_d2_velocity = None
            if args.dynamics_order == 2:
                acceleration = safe
                nominal_acceleration = reference
                nominal_d2_velocity = (
                    current_control_velocity + nominal_acceleration * control_period
                )
                safe = current_control_velocity + acceleration * control_period
                safe = torch.maximum(torch.minimum(safe, safe_control_upper), safe_control_lower)
                last_commanded_upper_velocity.copy_(safe[:, :17])
            if args.policy in {"wbt", "sonic", "sport"}:
                # Learned locomotion cannot realize discontinuous QP outputs
                # like the ideal mobile base can. Rate-limit only the command
                # presented to the locomotion network; upper-body safety
                # references remain at the full control cadence.
                rate = torch.tensor(
                    (
                        args.wbt_command_acceleration,
                        args.wbt_command_acceleration,
                        args.wbt_yaw_acceleration,
                    ),
                    device=args.device,
                )
                maximum_change = rate * control_period
                blend = float(np.clip(args.wbt_safety_base_blend, 0.0, 1.0))
                if args.dynamics_order == 2:
                    nominal_base_target = nominal_d2_velocity[:, 17:]
                    feasible_base_target = nominal_base_target + blend * (
                        safe[:, 17:] - nominal_base_target
                    )
                    # Acceleration integration above already enforces the
                    # command slew bound; a second limiter would integrate it
                    # twice and make avoidance unnecessarily sluggish.
                    safe_base = feasible_base_target
                else:
                    feasible_base_target = requested_velocity + blend * (
                        safe[:, 17:] - requested_velocity
                    )
                    safe_base = last_safe_base_command + torch.clamp(
                        feasible_base_target - last_safe_base_command,
                        -maximum_change,
                        maximum_change,
                    )
                last_safe_base_command.copy_(safe_base)
                safe[:, 17:] = safe_base
                actual_residual = constraints.lower - torch.einsum(
                    "bcu,bu->bc", constraints.A, constraint_safe
                )
                positive = torch.where(
                    constraints.active_mask,
                    torch.clamp_min(actual_residual, 0.0),
                    torch.zeros_like(actual_residual),
                )
                info["max_violation"] = positive.max(dim=1).values
                info["converged"] = info["max_violation"] <= 1.0e-3
                info["triggered"] = (constraint_safe - reference).abs().amax(dim=1) > 1.0e-3
            elif args.dynamics_order == 2:
                # Ideal mobile-base and sport paths still need a persistent
                # velocity command state for acceleration integration.
                last_safe_base_command.copy_(safe[:, 17:])
            if args.render_safety_trigger_constraints or args.render_safety_violations:
                witness_robot = torch.cat([query.witness_robot for query in query_sets], dim=1)
                witness_other = torch.cat(
                    [query.witness_environment for query in query_sets], dim=1
                )
                nominal_residual = constraints.lower - torch.einsum(
                    "bcu,bu->bc", constraints.A, reference
                )
                safe_residual = constraints.lower - torch.einsum(
                    "bcu,bu->bc", constraints.A, constraint_safe
                )
                latest_safety_visualization = {
                    "start": witness_robot,
                    "end": witness_other,
                    "trigger_mask": constraints.active_mask & (nominal_residual > 1.0e-5),
                    "violation_mask": constraints.active_mask & (safe_residual > 1.0e-5),
                }
            if args.profile_safety_stages:
                if args.device.startswith("cuda"):
                    torch.cuda.synchronize()
                safety_profile["solve"] += time.perf_counter() - profile_mark
                safety_profile["calls"] += 1
            safety_trigger_count.add_(info["triggered"].long())
            safety_max_residual = torch.maximum(safety_max_residual, info["max_violation"])
            minimum = torch.cat(distance_sets, dim=1).min(dim=1).values
            safety_minimum_distance = torch.minimum(safety_minimum_distance, minimum)
            # Preserve the original far-horizon target when no correction is
            # required. During intervention, expose a safe one-step target.
            safe_upper = current_upper + safe[:, :17] * control_period
            if args.dynamics_order != 2:
                safe_upper = torch.where(
                    info["triggered"][:, None], safe_upper, requested_upper_target
                )
            safe_upper[:, :3] = agent.default_body_pos[:, 12:15]
            return safe_upper, safe[:, 17:], info

        def update_stability_metrics(feedback):
            nonlocal min_root_height, max_root_tilt, max_root_angular_speed
            root_pose = feedback["root_pose_w"]
            quaternion = root_pose[:, 3:7]
            # Isaac Lab root quaternions use XYZW. R_zz is the cosine of the
            # angle between the robot's and world's up axes.
            up_alignment = 1.0 - 2.0 * (quaternion[:, 0].square() + quaternion[:, 1].square())
            tilt = torch.acos(torch.clamp(up_alignment, -1.0, 1.0))
            angular_speed = torch.linalg.vector_norm(feedback["root_angular_velocity_b"], dim=-1)
            min_root_height = min(min_root_height, float(root_pose[:, 2].min().item()))
            max_root_tilt = max(max_root_tilt, float(tilt.max().item()))
            max_root_angular_speed = max(max_root_angular_speed, float(angular_speed.max().item()))

        def reset_environments(env_ids, *, reason):
            """Reset and fully resample selected tensor rows independently."""
            nonlocal successful_resets, fall_resets, timeout_resets, workspace_exit_resets
            if env_ids.numel() == 0:
                return
            # Every completed tensor episode owns a full row-local reset.  A
            # previous continuous transition changed arm/base goals while
            # retaining the locomotion RNN and contact phase, which could tip
            # an otherwise successful robot during its next episode.
            physical_reset = True
            if physical_reset:
                reset_kwargs = {}
                if args.shared_obstacle_environment:
                    reset_kwargs["root_pose_w"] = sample_shared_spawn_pose(env_ids)
                agent.reset(env_ids=env_ids, **reset_kwargs)
                if policy is not None:
                    policy.reset(env_ids=env_ids)
                if safety_filter is not None and hasattr(safety_filter, "reset"):
                    safety_filter.reset(env_ids)
            reset_feedback = agent.get_feedback()
            reset_root = reset_feedback["root_pose_w"][env_ids]
            if physical_reset:
                initial_root_pose[env_ids] = reset_root
                initial_body_joint_pos[env_ids] = reset_feedback["body_joint_pos"][env_ids]
                target[env_ids] = agent.default_body_pos[env_ids]
                velocity_command[env_ids] = 0.0
                last_safe_base_command[env_ids] = 0.0
                last_commanded_upper_velocity[env_ids] = 0.0
                previous_sphere_jacobian_valid[env_ids] = False
                base_settle_count[env_ids] = 0
                base_settled[env_ids] = False
                base_translation_hold[env_ids] = False
                base_best_position_error[env_ids] = float("inf")
                base_no_progress_count[env_ids] = 0
                base_recovery_steps[env_ids] = 0
                if filtered_sphere_jacobian_dot is not None:
                    filtered_sphere_jacobian_dot[env_ids] = 0.0
                physical_motion_steps[env_ids] = 0
            episode_steps[env_ids] = 0
            episode_counts[env_ids] += 1
            scenario_episode_indices[env_ids.detach().cpu().numpy()] += 1
            # A new task goal always starts a new arrival/settling decision,
            # including goal-to-goal transitions that preserve simulator state.
            base_settle_count[env_ids] = 0
            base_settled[env_ids] = False
            base_translation_hold[env_ids] = False
            base_best_position_error[env_ids] = float("inf")
            base_no_progress_count[env_ids] = 0
            base_recovery_steps[env_ids] = 0

            if deterministic_scenario_sampling:
                sample_deterministic_scenario_rows(env_ids, reset_feedback)
            else:
                sample_base_goal_rows(env_ids, reset_feedback["root_pose_w"])
                sample_and_solve_arm_goals(env_ids, reset_feedback)
                sample_obstacles(env_ids, reset_feedback)
            count = int(env_ids.numel())
            if reason == "goal_reached":
                successful_resets += count
            elif reason == "fallen":
                fall_resets += count
            elif reason == "timeout":
                timeout_resets += count
            elif reason == "workspace_exit":
                workspace_exit_resets += count
            print(
                f"[SPARK] {reason} environment reset: "
                + ", ".join(
                    f"env {env_id} -> episode {episode_counts[env_id].item()}"
                    for env_id in env_ids.detach().cpu().tolist()
                ),
                flush=True,
            )

        def step(index):
            nonlocal target
            action_info = {}
            profile_enabled = bool(args.profile_safety_stages)

            def profile_mark():
                if profile_enabled and args.device.startswith("cuda"):
                    torch.cuda.synchronize()
                return time.perf_counter()

            def profile_record(name, started):
                if not profile_enabled:
                    return
                if args.device.startswith("cuda"):
                    torch.cuda.synchronize()
                step_profile[name] += time.perf_counter() - started

            stage_started = profile_mark()
            update_base_goals()
            update_obstacles()
            profile_record("obstacles", stage_started)
            stage_started = profile_mark()
            if args.policy == "mobile_base":
                feedback = agent.get_feedback()
                update_goal_command(feedback)
                update_base_arrival_state(feedback)
                profile_record("feedback_goal", stage_started)
                stage_started = profile_mark()
                safe_upper, safe_velocity, _ = apply_tensor_safety(
                    feedback, upper_body_target, velocity_command
                )
                profile_record("safety_total", stage_started)
                # Keep the common benchmark state/metrics representative of
                # the command actually sent by the ideal mobile-base plant.
                target[:, 12:] = safe_upper
                stage_started = profile_mark()
                agent.step_kinematic_mobile_base(
                    safe_velocity,
                    safe_upper,
                    joint_velocity_limit=upper_velocity_limit,
                )
                profile_record("physics_step", stage_started)
            elif policy is None:
                phase = torch.tensor(index * 0.03, device=args.device)
                target[:, 15] = agent.default_body_pos[:, 15] + 0.08 * torch.sin(phase)
            else:
                feedback = agent.get_feedback()
                update_goal_command(feedback)
                profile_record("feedback_goal", stage_started)
                stage_started = profile_mark()
                # Learned walking has within-step velocity oscillations;
                # require sustained pose arrival rather than an instantaneous
                # threshold, just as the scalar WBT arrival latch does.
                update_base_arrival_state(feedback)
                update_base_progress_recovery()
                if args.policy == "sport":
                    safe_upper_body_target, safe_velocity_command, safety_info = (
                        apply_tensor_safety(feedback, upper_body_target, velocity_command)
                    )
                    profile_record("safety_total", stage_started)
                    stage_started = profile_mark()
                    target, policy_info = policy.infer_tensor(
                        body_joint_pos=feedback["body_joint_pos"],
                        body_joint_vel=feedback["body_joint_vel"],
                        root_quat_xyzw=feedback["root_pose_w"][:, 3:7],
                        root_angular_velocity=feedback["root_angular_velocity_b"],
                        velocity_command=safe_velocity_command,
                        upper_body_target=safe_upper_body_target,
                    )
                elif args.policy == "sonic":
                    safe_upper_body_target, safe_velocity_command, safety_info = (
                        apply_tensor_safety(feedback, upper_body_target, velocity_command)
                    )
                    profile_record("safety_total", stage_started)
                    stage_started = profile_mark()
                    warming = physical_motion_steps < int(args.warmup_steps)
                    sonic_command = safe_velocity_command.clone()
                    sonic_command[warming] = 0.0
                    planner_goal_distance = latest_position_error.clone()
                    recovering = base_recovery_steps > 0
                    planner_goal_distance[recovering] = max(
                        float(policy.slowdown_distance) + 0.1,
                        0.6,
                    )
                    target, policy_info = policy.infer_tensor(
                        body_joint_pos=feedback["body_joint_pos"],
                        body_joint_vel=feedback["body_joint_vel"],
                        root_pose_w=feedback["root_pose_w"],
                        root_angular_velocity=feedback["root_angular_velocity_b"],
                        command=sonic_command,
                        upper_body_target=safe_upper_body_target,
                        base_goal_distance=planner_goal_distance,
                    )
                    base_recovery_steps.copy_(torch.clamp(base_recovery_steps - 1, min=0))
                else:
                    warming = physical_motion_steps < int(args.warmup_steps)
                    if args.goal_mode == "velocity":
                        requested_loco = torch.full(
                            (args.num_envs,),
                            args.wbt_mode == "loco",
                            device=args.device,
                            dtype=torch.bool,
                        )
                    else:
                        # Continue running the locomotion expert with a zero
                        # command until the measured base motion has settled.
                        requested_loco = ~base_settled
                    loco_mask = requested_loco & ~warming

                    requested_upper_body_target = upper_body_target
                    if args.hold_arm_goals_during_locomotion:
                        # Match scalar WBT's hold_upper_body_during_locomotion
                        # contract.  Cartesian IK is sampled at episode start,
                        # but applying it while the learned gait is active can
                        # inject a large upper-body reaction into the floating
                        # base.  Retain each row's last commanded posture while
                        # walking; after that row settles, its same IK goal is
                        # released through the normal rate limiter below.
                        requested_upper_body_target = torch.where(
                            loco_mask[:, None],
                            target[:, 12:],
                            upper_body_target,
                        )
                    safe_upper_body_target, safe_velocity_command, safety_info = (
                        apply_tensor_safety(
                            feedback,
                            requested_upper_body_target,
                            velocity_command,
                        )
                    )
                    # Match scalar WBT's stateful upper-body target filter.
                    # Applying a freshly solved IK configuration in one PD
                    # step creates a large reaction torque during locomotion.
                    previous_upper_target = target[:, 12:].clone()
                    upper_target_step = torch.tensor(
                        [0.006, 0.006, 0.006] + [0.014] * 14,
                        device=args.device,
                    )
                    safe_upper_body_target = previous_upper_target + torch.clamp(
                        safe_upper_body_target - previous_upper_target,
                        -upper_target_step,
                        upper_target_step,
                    )
                    profile_record("safety_total", stage_started)
                    stage_started = profile_mark()
                    wbt_inputs = dict(
                        body_joint_pos=feedback["body_joint_pos"],
                        body_joint_vel=feedback["body_joint_vel"],
                        root_quat_xyzw=feedback["root_pose_w"][:, 3:7],
                        root_angular_velocity=feedback["root_angular_velocity_b"],
                        upper_body_target=safe_upper_body_target,
                        stance=args.wbt_stance,
                    )
                    blend = torch.clamp(
                        (physical_motion_steps - int(args.warmup_steps) + 1).float()
                        / max(1, int(args.wbt_transition_steps)),
                        0.0,
                        1.0,
                    )
                    all_loco = bool(torch.all(loco_mask).item())
                    all_squat = bool(torch.all(~loco_mask).item())
                    if all_loco and bool(torch.all(blend >= 1.0).item()):
                        target, policy_info = policy.infer_tensor(
                            **wbt_inputs,
                            command=safe_velocity_command,
                            mode="loco",
                        )
                    elif all_squat:
                        target, policy_info = policy.infer_tensor(
                            **wbt_inputs, command=None, mode="squat"
                        )
                    else:
                        # A mixed batch still owns independent recurrent
                        # state per environment. Evaluate both experts once
                        # and select each environment's appropriate row.
                        squat_target, squat_info = policy.infer_tensor(
                            **wbt_inputs, command=None, mode="squat"
                        )
                        loco_target, loco_info = policy.infer_tensor(
                            **wbt_inputs,
                            command=safe_velocity_command,
                            mode="loco",
                        )
                        blended_loco_target = (1.0 - blend[:, None]) * squat_target + blend[
                            :, None
                        ] * loco_target
                        target = torch.where(
                            loco_mask[:, None],
                            blended_loco_target,
                            squat_target,
                        )
                        policy_info = dict(squat_info)
                        for gain_name in ("motor_kps", "motor_kds"):
                            policy_info[gain_name] = torch.where(
                                loco_mask[:, None],
                                (1.0 - blend[:, None]) * squat_info[gain_name]
                                + blend[:, None] * loco_info[gain_name],
                                squat_info[gain_name],
                            )
                profile_record("policy_inference", stage_started)
                action_info.update(
                    {
                        key: value
                        for key, value in policy_info.items()
                        if key in ("motor_kps", "motor_kds")
                    }
                )
                if "motor_kps" in action_info:
                    action_info["motor_kps"] = action_info["motor_kps"] * motor_kp_scale
                if "motor_kds" in action_info:
                    action_info["motor_kds"] = action_info["motor_kds"] * motor_kd_scale
            if args.policy != "mobile_base":
                stage_started = profile_mark()
                agent.step(
                    command,
                    action_info={
                        **action_info,
                        "target_actuated_pos": target,
                        "left_gripper_goal": gripper_closed,
                        "right_gripper_goal": (
                            ~gripper_closed if args.alternate_grippers else gripper_closed
                        ),
                    },
                )
                profile_record("physics_step", stage_started)
            stage_started = profile_mark()
            feedback_after_step = agent.get_feedback()
            update_stability_metrics(feedback_after_step)
            episode_steps.add_(1)
            physical_motion_steps.add_(1)
            update_goal_command(feedback_after_step)
            if args.goal_mode == "velocity":
                base_done = torch.ones(args.num_envs, device=args.device, dtype=torch.bool)
            elif args.policy == "sport":
                # Sport's throughput benchmark is a base-goal task, whose
                # public completion contract is the current SE(2) tolerance.
                # Requiring WBT's 50-step locomotion-to-squat latch allowed a
                # row to visibly enter the goal and leave again without ever
                # resetting. A successful row is physically reset below, so
                # it does not need the standing-transition guard.
                base_done = (latest_position_error <= float(args.base_goal_tolerance)) & (
                    latest_yaw_error <= float(args.base_yaw_tolerance)
                )
            else:
                base_done = base_settled
            if arm_kinematics_enabled:
                current_left_ee, current_right_ee = ik_solver.forward(
                    feedback_after_step["body_joint_pos"][:, 12 if ik_includes_waist else 15 :]
                )
                left_arm_error = torch.linalg.vector_norm(
                    current_left_ee[:, :3, 3] - left_ee_targets[:, :3, 3], dim=1
                )
                right_arm_error = torch.linalg.vector_norm(
                    current_right_ee[:, :3, 3] - right_ee_targets[:, :3, 3], dim=1
                )
                arm_done = (
                    (left_arm_error < float(args.arm_goal_tolerance))
                    & (right_arm_error < float(args.arm_goal_tolerance))
                    & latest_ik_success
                )
            else:
                arm_done = torch.ones_like(base_done)
            if index > 0 and index % 100 == 0 and arm_kinematics_enabled:
                waiting_for_arms = base_done & ~arm_done
                if bool(torch.any(waiting_for_arms).item()):
                    rows = torch.nonzero(waiting_for_arms).flatten().tolist()
                    print(
                        "[SPARK] base reached; whole-goal reset waiting for arms: "
                        + ", ".join(
                            f"env {env_id} "
                            f"left={left_arm_error[env_id].item():.3f}m "
                            f"right={right_arm_error[env_id].item():.3f}m"
                            for env_id in rows
                        ),
                        flush=True,
                    )
                waiting_for_base = arm_done & ~base_done
                if bool(torch.any(waiting_for_base).item()):
                    rows = torch.nonzero(waiting_for_base).flatten().tolist()
                    print(
                        "[SPARK] arms reached; whole-goal reset waiting for base: "
                        + ", ".join(
                            f"env {env_id} xy={latest_position_error[env_id].item():.3f}m "
                            f"yaw={latest_yaw_error[env_id].item():.3f}rad"
                            for env_id in rows
                        ),
                        flush=True,
                    )
            profile_record("post_step", stage_started)
            stage_started = profile_mark()
            completed = base_done & arm_done & (physical_motion_steps >= int(args.warmup_steps))
            timed_out = episode_steps >= int(args.max_episode_length)
            root_pose = feedback_after_step["root_pose_w"]
            root_quaternion = root_pose[:, 3:7]
            up_alignment = 1.0 - 2.0 * (
                root_quaternion[:, 0].square() + root_quaternion[:, 1].square()
            )
            fallen = (root_pose[:, 2] < float(args.fall_height_threshold)) | (
                up_alignment < float(np.cos(np.deg2rad(args.fall_tilt_degrees)))
            )
            workspace_lower, workspace_upper = workspace_bounds(
                torch.arange(args.num_envs, device=args.device)
            )
            # The obstacle Z range describes where task objects may be
            # sampled; it is not a valid bound on the robot root height.
            # Cloned-environment escape detection is planar, matching base
            # goal sampling and the shared-workspace branch above.
            workspace_margin = torch.tensor(
                (args.workspace_exit_margin, args.workspace_exit_margin),
                device=args.device,
            )
            outside_workspace = torch.any(
                (root_pose[:, :2] < workspace_lower[:, :2] - workspace_margin)
                | (root_pose[:, :2] > workspace_upper[:, :2] + workspace_margin),
                dim=1,
            )
            if args.num_obstacles == 0 and not args.shared_obstacle_environment:
                # In v0 the bounds are only a goal-sampling region.  Isaac's
                # cloned environments are collision-isolated, so crossing a
                # visual tile boundary is not a task failure.
                outside_workspace.zero_()
            fall_rows = fallen & bool(args.reset_on_fall)
            workspace_rows = outside_workspace & ~fall_rows
            if bool(torch.any(workspace_rows).item()):
                for env_id in torch.nonzero(workspace_rows).flatten().tolist():
                    print(
                        "[SPARK] workspace diagnostic: "
                        f"env {env_id}, root={root_pose[env_id, :3].detach().cpu().tolist()}, "
                        f"lower={workspace_lower[env_id].detach().cpu().tolist()}, "
                        f"upper={workspace_upper[env_id].detach().cpu().tolist()}",
                        flush=True,
                    )
            success_rows = completed & bool(args.reset_on_success) & ~fall_rows & ~workspace_rows
            timeout_rows = (
                timed_out
                & bool(args.reset_on_timeout)
                & ~fall_rows
                & ~workspace_rows
                & ~success_rows
            )
            reset_occurred = False
            for rows, reason in (
                (fall_rows, "fallen"),
                (workspace_rows, "workspace_exit"),
                (success_rows, "goal_reached"),
                (timeout_rows, "timeout"),
            ):
                if bool(torch.any(rows).item()):
                    reset_environments(torch.nonzero(rows).flatten(), reason=reason)
                    reset_occurred = True
            if reset_occurred:
                # All visualization data below must come from the post-reset
                # simulator state. Reusing feedback_after_step here left the
                # articulation/goal markers one frame (or indefinitely, with
                # sparse rendering) at the pre-reset base pose.
                feedback_after_step = agent.get_feedback()
                update_goal_command(feedback_after_step)
            profile_record("reset_checks", stage_started)
            stage_started = profile_mark()
            if update_visual_goals is not None and index % max(1, int(args.render_every)) == 0:
                update_visual_goals(feedback_after_step)
                if args.num_obstacles and args.environment_representation == "sphere":
                    if args.shared_obstacle_environment:
                        agent.set_visual_shared_obstacles(
                            obstacle_positions[0].detach().cpu().numpy(),
                            args.obstacle_radius,
                        )
                    else:
                        agent.set_visual_obstacles(
                            obstacle_positions.detach().cpu().numpy(),
                            args.obstacle_radius,
                        )
                elif args.num_obstacles and args.environment_representation == "point_cloud":
                    if args.shared_obstacle_environment:
                        # Render the complete shared scene. Safety tensors stay
                        # local and fixed-capacity per robot, but visualization
                        # must not look as though distant global obstacles
                        # disappeared from the environment.
                        visual_points = (
                            obstacle_positions[0, :, None, :] + point_surface_offsets[None, :, :]
                        ).reshape(-1, 3)
                    else:
                        visible = obstacle_cloud.valid_mask[0]
                        visual_points = obstacle_cloud.positions[0, visible]
                    agent.set_visual_environment_point_cloud(
                        visual_points.detach().cpu().numpy(),
                        args.point_visual_size,
                    )
                elif args.num_obstacles and args.environment_representation == "mesh":
                    agent.set_visual_environment_mesh(
                        obstacle_mesh.vertices[0].detach().cpu().numpy(),
                        obstacle_mesh.faces.detach().cpu().numpy(),
                    )
                if args.render_robot_collision_volumes and collision_model is not None:
                    # Render from the post-step articulation state. Reusing
                    # pre-control safety centers makes markers visibly trail
                    # fast links by one complete control period.
                    render_centers, _ = build_sphere_control_jacobian(feedback_after_step)
                    agent.set_visual_robot_collision_spheres(
                        render_centers.detach().cpu().numpy(),
                        collision_model.radii.detach().cpu().numpy(),
                        collision_model.environment_mask.detach().cpu().numpy(),
                        collision_mask=(
                            latest_collision_mask.bool().detach().cpu().numpy()
                            if latest_collision_mask is not None
                            else None
                        ),
                        closest_distance=(
                            latest_collision_distance.detach().cpu().numpy()
                            if latest_collision_distance is not None
                            else None
                        ),
                        minimum_distance=args.minimum_distance,
                    )
                if latest_safety_visualization is not None:
                    visual = latest_safety_visualization
                    agent.set_visual_safety_constraints(
                        visual["start"].detach().cpu().numpy(),
                        visual["end"].detach().cpu().numpy(),
                        trigger_mask=(
                            visual["trigger_mask"].detach().cpu().numpy()
                            if args.render_safety_trigger_constraints
                            else None
                        ),
                        violation_mask=(
                            visual["violation_mask"].detach().cpu().numpy()
                            if args.render_safety_violations
                            else None
                        ),
                    )
                # Present exactly once after the articulation, goal markers,
                # collision volumes, and constraints all represent the same
                # post-step/post-reset simulator state.
                if video_recorder is not None and index >= args.warmup_steps:
                    video_recorder.schedule(index - args.warmup_steps)
                agent.render_frame()
            profile_record("visualization", stage_started)
            if profile_enabled:
                step_profile["calls"] += 1

        for index in range(args.warmup_steps):
            step(index)
        if args.device.startswith("cuda"):
            torch.cuda.synchronize()
        started = time.perf_counter()
        for index in range(args.num_steps):
            step(index + args.warmup_steps)
        if args.device.startswith("cuda"):
            torch.cuda.synchronize()
        elapsed = time.perf_counter() - started

        feedback = agent.get_feedback()
        update_goal_command(feedback)
        final_base_offset = feedback["root_pose_w"][:, :2] - initial_root_pose[:, :2]
        tracking_rms = torch.sqrt(torch.mean((feedback["body_joint_pos"] - target) ** 2)).item()
        arm_motion = torch.abs(feedback["body_joint_pos"][:, 12:] - initial_body_joint_pos[:, 12:])
        arm_target_error = torch.abs(feedback["body_joint_pos"][:, 12:] - upper_body_target)
        if arm_kinematics_enabled:
            final_left_ee, final_right_ee = ik_solver.forward(
                feedback["body_joint_pos"][:, 12 if ik_includes_waist else 15 :]
            )
            final_left_cartesian_error = torch.linalg.vector_norm(
                final_left_ee[:, :3, 3] - left_ee_targets[:, :3, 3], dim=1
            )
            final_right_cartesian_error = torch.linalg.vector_norm(
                final_right_ee[:, :3, 3] - right_ee_targets[:, :3, 3], dim=1
            )
        else:
            final_left_cartesian_error = torch.zeros(args.num_envs, device=args.device)
            final_right_cartesian_error = torch.zeros_like(final_left_cartesian_error)
        control_hz = args.num_steps / elapsed
        env_steps_per_second = args.num_envs * control_hz
        stability_height_threshold = float(args.fall_height_threshold)
        stability_passed = (
            fall_resets == 0
            and min_root_height >= stability_height_threshold
            and max_root_tilt <= float(torch.deg2rad(torch.tensor(30.0)).item())
            and torch.isfinite(feedback["body_joint_pos"]).all().item()
        )
        print(
            "SPARK Isaac parallel benchmark:\n"
            f"  environments: {args.num_envs}\n"
            f"  policy: {args.policy}\n"
            f"  control steps: {args.num_steps}\n"
            f"  control rate: {control_hz:.2f} Hz\n"
            f"  aggregate env steps/s: {env_steps_per_second:.2f}\n"
            f"  body target RMS error: {tracking_rms:.6f} rad\n"
            f"  arm motion mean/max: "
            f"{arm_motion.mean().item():.4f}/{arm_motion.max().item():.4f} rad\n"
            f"  arm target error mean/max: "
            f"{arm_target_error.mean().item():.4f}/{arm_target_error.max().item():.4f} rad\n"
            f"  Cartesian left arm error mean/max: "
            f"{final_left_cartesian_error.mean().item():.4f}/"
            f"{final_left_cartesian_error.max().item():.4f} m\n"
            f"  Cartesian right arm error mean/max: "
            f"{final_right_cartesian_error.mean().item():.4f}/"
            f"{final_right_cartesian_error.max().item():.4f} m\n"
            f"  IK backend: {ik_backend_name}\n"
            f"  IK queries/total solve time: {total_ik_queries}/{total_ik_time:.4f} s\n"
            f"  simulated control period: {args.dt * args.control_decimation:.4f} s\n"
            f"  final base position error mean/max: "
            f"{latest_position_error.mean().item():.4f}/{latest_position_error.max().item():.4f} m\n"
            f"  final base yaw error mean/max: "
            f"{latest_yaw_error.mean().item():.4f}/{latest_yaw_error.max().item():.4f} rad\n"
            f"  final base XY offset mean: "
            f"({final_base_offset[:, 0].mean().item():+.4f}, "
            f"{final_base_offset[:, 1].mean().item():+.4f}) m\n"
            f"  final requested base command mean: "
            f"({velocity_command[:, 0].mean().item():+.4f}, "
            f"{velocity_command[:, 1].mean().item():+.4f}, "
            f"{velocity_command[:, 2].mean().item():+.4f})\n"
            f"  final adapted base command mean: "
            f"({last_safe_base_command[:, 0].mean().item():+.4f}, "
            f"{last_safe_base_command[:, 1].mean().item():+.4f}, "
            f"{last_safe_base_command[:, 2].mean().item():+.4f})\n"
            f"  minimum root height: {min_root_height:.4f} m "
            f"(fall threshold {stability_height_threshold:.4f} m)\n"
            f"  maximum root tilt: {torch.rad2deg(torch.tensor(max_root_tilt)).item():.2f} deg\n"
            f"  maximum root angular speed: {max_root_angular_speed:.3f} rad/s\n"
            f"  stability check: {'PASS' if stability_passed else 'FAIL'}"
            f"\n  successful episode resets: {successful_resets}"
            f"\n  fall resets: {fall_resets}"
            f"\n  timeout resets: {timeout_resets}"
            f"\n  workspace-exit resets: {workspace_exit_resets}"
        )
        if safety_enabled:
            print(
                "Tensor safety:\n"
                f"  algorithm: {args.safe_algo}\n"
                f"  trigger steps per env: {safety_trigger_count.detach().cpu().tolist()}\n"
                f"  combined minimum distance per env: "
                f"{safety_minimum_distance.detach().cpu().tolist()}\n"
                f"  environment minimum distance per env: "
                f"{safety_environment_minimum_distance.detach().cpu().tolist()}\n"
                f"  self minimum distance per env: "
                f"{safety_self_minimum_distance.detach().cpu().tolist()}\n"
                f"  maximum residual per env: {safety_max_residual.detach().cpu().tolist()}"
            )
            if safety_profile["calls"]:
                calls = safety_profile["calls"]
                print("  synchronized stage latency:")
                for stage in ("query", "constraints", "solve"):
                    print(f"    {stage}: {1.0e3 * safety_profile[stage] / calls:.3f} ms/call")
        if step_profile["calls"]:
            calls = step_profile["calls"]
            print("Synchronized full-step stage latency:")
            accounted = 0.0
            for stage in (
                "obstacles",
                "feedback_goal",
                "safety_total",
                "policy_inference",
                "physics_step",
                "post_step",
                "reset_checks",
                "visualization",
            ):
                mean = step_profile[stage] / calls
                accounted += mean
                print(f"  {stage:16s}: {1.0e3 * mean:8.3f} ms/step")
            print(f"  accounted total : {1.0e3 * accounted:8.3f} ms/step")
        if args.num_envs > 1:
            print(
                "  sampled base goal offsets:\n    "
                + "\n    ".join(
                    f"env {index}: ({goal[0]:+.3f}, {goal[1]:+.3f}, {goal[2]:+.3f})"
                    for index, goal in enumerate(base_goal_offsets.detach().cpu().tolist())
                )
            )
            print(
                "  per-env target diversity (mean joint std): "
                f"{target.std(dim=0).mean().item():.6f} rad"
            )
        if args.with_hand and all(
            key in feedback for key in ("left_gripper_pos_fbk", "right_gripper_pos_fbk")
        ):
            print(
                "  hand feedback shapes: "
                f"left={tuple(feedback['left_gripper_pos_fbk'].shape)}, "
                f"right={tuple(feedback['right_gripper_pos_fbk'].shape)}"
            )
    except BaseException as exc:
        print(f"[SPARK] parallel benchmark failed: {type(exc).__name__}: {exc}", flush=True)
        import traceback

        traceback.print_exc()
        raise
    finally:
        if video_recorder is not None:
            video_recorder.close()
        if policy is not None and hasattr(policy, "close"):
            policy.close()
        if agent is not None:
            agent.close()
        simulation_app.close(wait_for_replicator=False)


if __name__ == "__main__":
    main()
