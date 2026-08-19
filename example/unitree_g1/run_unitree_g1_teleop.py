import argparse
from pathlib import Path

from spark_pipeline import TeleopPipeline as Pipeline
from spark_pipeline import UnitreeG1TeleopPipelineConfig as PipelineConfig
from spark_pipeline.simulation_runtime import configure_simulation_presentation
from spark_pipeline.teleop.camera import (
    add_teleop_camera_arguments,
    add_teleop_viewer_arguments,
    configure_teleop_camera,
    configure_teleop_viewer,
)
from spark_pipeline.teleop.unitree_g1_wbt_whole_body_teleop_pipeline_config import (
    UNITREE_G1_HEAD_CAMERA_CONFIG,
)


USE_REAL = False

CONTROL_WEIGHT = [
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
    1.0,
]


def _set_if_not_none(obj, name, value):
    if value is not None:
        setattr(obj, name, value)


def _value_or_default(value, default):
    return default if value is None else value


def _parse_bool(value):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "t", "yes", "y", "on"):
        return True
    if normalized in ("0", "false", "f", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"Expected a boolean value, got {value!r}")


def _has_hand_robot_config(robot_cfg):
    return robot_cfg is not None and "WithHand" in robot_cfg


def _default_sim_agent(robot_cfg):
    from spark_robot import get_agent_class_name

    return get_agent_class_name(robot_cfg, backend="mujoco")


def _is_fixed_base_robot(robot_cfg):
    return any(name in (robot_cfg or "") for name in ("FixedBase", "RightArm", "DualArm"))


def _normalize_min_distance(value):
    if value is None:
        return None
    if isinstance(value, dict):
        return value
    return {"environment": float(value), "self": 0.01}


def _configure_safe_algo(safe_algo_cfg, safe_algo_name, **kwargs):
    if safe_algo_name == "bypass":
        safe_algo_cfg.class_name = "ByPassSafeControl"
    elif safe_algo_name == "ssa":
        safe_algo_cfg.class_name = "BasicSafeSetAlgorithm"
        safe_algo_cfg.eta_ssa = _value_or_default(kwargs.get("eta_ssa"), 0.1)
    elif safe_algo_name == "rssa":
        safe_algo_cfg.class_name = "RelaxedSafeSetAlgorithm"
        safe_algo_cfg.eta_ssa = _value_or_default(kwargs.get("eta_ssa"), 0.3)
        safe_algo_cfg.slack_weight = _value_or_default(kwargs.get("slack_weight"), 1e3)
    elif safe_algo_name == "sss":
        safe_algo_cfg.class_name = "BasicSublevelSafeSetAlgorithm"
        safe_algo_cfg.lambda_sss = _value_or_default(kwargs.get("lambda_sss"), 10.0)
    elif safe_algo_name == "rsss":
        safe_algo_cfg.class_name = "RelaxedSublevelSafeSetAlgorithm"
        safe_algo_cfg.lambda_sss = _value_or_default(kwargs.get("lambda_sss"), 10.0)
        safe_algo_cfg.slack_weight = _value_or_default(kwargs.get("slack_weight"), 1e3)
    elif safe_algo_name == "cbf":
        safe_algo_cfg.class_name = "BasicControlBarrierFunction"
        safe_algo_cfg.lambda_cbf = _value_or_default(kwargs.get("lambda_cbf"), 10.0)
    elif safe_algo_name == "rcbf":
        safe_algo_cfg.class_name = "RelaxedControlBarrierFunction"
        safe_algo_cfg.lambda_cbf = _value_or_default(kwargs.get("lambda_cbf"), 10.0)
        safe_algo_cfg.slack_weight = _value_or_default(kwargs.get("slack_weight"), 1e3)
    elif safe_algo_name == "pfm":
        safe_algo_cfg.class_name = "BasicPotentialFieldMethod"
        safe_algo_cfg.c_pfm = _value_or_default(kwargs.get("c_pfm"), 1.0)
    elif safe_algo_name == "sma":
        safe_algo_cfg.class_name = "BasicSlidingModeAlgorithm"
        safe_algo_cfg.c_sma = _value_or_default(kwargs.get("c_sma"), 1.0)
    else:
        raise ValueError(f"Unsupported safe_algo: {safe_algo_name}")


def _configure_safety_index(safety_index_cfg, safety_index_name, **kwargs):
    if safety_index_name == "si1":
        safety_index_cfg.class_name = "FirstOrderCollisionSafetyIndex"
    elif safety_index_name == "si1a":
        safety_index_cfg.class_name = "FirstOrderCollisionSafetyIndexApprox"
    elif safety_index_name == "si2":
        safety_index_cfg.class_name = "SecondOrderCollisionSafetyIndex"
        safety_index_cfg.phi_n = 1.0
        safety_index_cfg.phi_k = 1.0
    elif safety_index_name == "si2a":
        safety_index_cfg.class_name = "SecondOrderCollisionSafetyIndexApprox"
        safety_index_cfg.phi_n = 1.0
        safety_index_cfg.phi_k = 10.0
    elif safety_index_name == "si2nn":
        safety_index_cfg.class_name = "SecondOrderNNCollisionSafetyIndex"
        safety_index_cfg.phi_n = 2.0
        safety_index_cfg.phi_k = 1.0
        safety_index_cfg.phi_nn_path = "n_2_scalar.onnx"
    else:
        raise ValueError(f"Unsupported safety_index: {safety_index_name}")

    _set_if_not_none(
        safety_index_cfg, "enable_self_collision", kwargs.get("enable_self_collision", None)
    )
    min_distance = _normalize_min_distance(kwargs.get("min_distance", None))
    if min_distance is not None:
        safety_index_cfg.min_distance = min_distance


def config_task_module(cfg: PipelineConfig, **kwargs):
    """Configure task-related settings."""
    robot_cfg = kwargs.get("robot_cfg") or cfg.robot.cfg.class_name
    cfg.env.task.enable_ros = _value_or_default(kwargs.get("enable_ros", None), False)
    cfg.env.task.mode = "Velocity"
    cfg.env.task.num_obstacle_task = _value_or_default(kwargs.get("num_obstacle_task", None), 2)
    obstacle_range = kwargs.get("obstacle_range")
    if obstacle_range is not None:
        cfg.env.task.obstacle_range = [
            tuple(obstacle_range[index : index + 2]) for index in range(0, 6, 2)
        ]
    cfg.env.task.environment_representation = kwargs.get("environment_representation", "sphere")
    cfg.env.task.points_per_obstacle = int(kwargs.get("points_per_obstacle", 64))
    cfg.env.task.max_visualized_points = int(kwargs.get("max_visualized_points", 200))
    cfg.env.task.object_mesh_directory = kwargs.get("object_mesh_directory")
    cfg.env.task.object_mesh_scale = float(kwargs.get("object_mesh_scale", 1.0))
    _set_if_not_none(cfg.env.task, "dt", kwargs.get("task_dt", None))
    _set_if_not_none(cfg.env.task, "ros_version", kwargs.get("ros_version", None))
    if "RightArm" in robot_cfg:
        cfg.env.task.use_dual_arm = False
        cfg.env.task.base_goal_enable = False
    if "FixedBase" in robot_cfg or "DualArm" in robot_cfg:
        cfg.env.task.base_goal_enable = False
    return cfg


def config_agent_module(cfg: PipelineConfig, **kwargs):
    """Configure agent-related settings."""
    import spark_robot

    use_real = kwargs.get("use_real", USE_REAL)
    backend = kwargs.get("backend", "mujoco")
    robot_cfg = kwargs.get("robot_cfg") or cfg.robot.cfg.class_name
    has_hand = _has_hand_robot_config(cfg.robot.cfg.class_name)
    configure_teleop_viewer(cfg.env.agent, kwargs)
    if use_real and backend != "mujoco":
        raise ValueError(
            "--backend selects the primary simulator and cannot select a real-agent twin; "
            "omit it in real mode and use --digital-twin-backend instead"
        )
    if use_real:
        cfg.env.agent.class_name = kwargs.get("agent_cfg") or "UnitreeG1RealAgent"
        cfg.env.agent.enable_viewer = False
        cfg.env.agent.enable_keyboard_control = _value_or_default(
            kwargs.get("enable_keyboard_control", None), True
        )
        cfg.env.agent.dt = _value_or_default(kwargs.get("dt", None), 0.02)
        cfg.env.agent.control_decimation = _value_or_default(
            kwargs.get("control_decimation", None), 1
        )
        cfg.env.agent.send_cmd = _value_or_default(kwargs.get("send_cmd", None), True)
        cfg.env.agent.net = kwargs.get("net", "")
        cfg.env.agent.unitree_model = _value_or_default(kwargs.get("unitree_model", None), "g1")
        cfg.env.agent.level = _value_or_default(kwargs.get("level", None), "high")
        cfg.env.agent.enable_hand_control = _value_or_default(
            kwargs.get("enable_hand_control", None), has_hand
        )
        cfg.env.agent.enable_digital_twin = bool(kwargs.get("enable_digital_twin", True))
        cfg.env.agent.digital_twin_backend = kwargs.get("digital_twin_backend", "mujoco")
        cfg.env.agent.digital_twin_sync_hz = float(kwargs.get("digital_twin_sync_hz", 500.0))
        if kwargs.get("task_dt") is None:
            cfg.env.task.dt = cfg.env.agent.dt * cfg.env.agent.control_decimation
        return cfg

    timing = getattr(spark_robot, robot_cfg)().simulator_dynamics
    cfg.env.agent.dt = _value_or_default(kwargs.get("dt", None), timing.physics_dt)
    cfg.env.agent.control_decimation = _value_or_default(
        kwargs.get("control_decimation", None), timing.control_decimation
    )

    if backend == "isaac":
        use_sim_dynamics = bool(kwargs.get("use_sim_dynamics", False))
        cfg.env.agent.class_name = kwargs.get("agent_cfg") or (
            "UnitreeG1IsaacAgent" if use_sim_dynamics else "UnitreeG1DynamicsVisualizationAgent"
        )
        cfg.env.agent.use_sim_dynamics = use_sim_dynamics
        cfg.env.agent.dynamics_backend = "simulator" if use_sim_dynamics else "model"
        cfg.env.agent.fixed_base = _is_fixed_base_robot(robot_cfg)
        cfg.env.agent.render = False
        cfg.env.agent.render_on_step = False
        cfg.env.agent.device = kwargs.get("isaac_device") or "cpu"
    else:
        cfg.env.agent.class_name = kwargs.get("agent_cfg") or _default_sim_agent(robot_cfg)
        cfg.env.agent.use_sim_dynamics = _value_or_default(
            kwargs.get("use_sim_dynamics", None), True
        )
    cfg.env.agent.obstacle_debug["manual_movement_step_size"] = _value_or_default(
        kwargs.get("manual_movement_step_size", None),
        0.02,
    )
    cfg.env.agent.obstacle_debug["num_obstacle"] = _value_or_default(
        kwargs.get("num_obstacle_debug", None), 1
    )
    cfg.env.agent.enable_viewer = _value_or_default(kwargs.get("enable_viewer", None), True)
    cfg.env.agent.enable_keyboard_control = _value_or_default(
        kwargs.get("enable_keyboard_control", None), False
    )
    cfg.env.agent.enable_hand_control = _value_or_default(
        kwargs.get("enable_hand_control", None), has_hand
    )
    configure_teleop_camera(cfg.env.agent, kwargs)
    cfg.env.agent.camera_config = UNITREE_G1_HEAD_CAMERA_CONFIG
    viewer_config = dict(cfg.env.agent.viewer_config)
    viewer_overrides = {
        "camera_lookat": kwargs.get("viewer_lookat"),
        "camera_distance": kwargs.get("viewer_distance"),
        "camera_azimuth": kwargs.get("viewer_azimuth"),
        "camera_elevation": kwargs.get("viewer_elevation"),
        "camera_vertical_fov": kwargs.get("viewer_vertical_fov"),
    }
    viewer_config.update(
        {name: value for name, value in viewer_overrides.items() if value is not None}
    )
    cfg.env.agent.viewer_config = viewer_config
    if kwargs.get("task_dt") is None:
        cfg.env.task.dt = cfg.env.agent.dt * cfg.env.agent.control_decimation
    return cfg


def config_policy_module(cfg: PipelineConfig, **kwargs):
    """Configure policy-related settings."""
    _set_if_not_none(
        cfg.policy.nominal_controller,
        "class_name",
        kwargs.get("policy_config", None),
    )
    return cfg


def config_safety_module(cfg: PipelineConfig, **kwargs):
    """Configure safety-related settings."""
    safe_algo = kwargs.get("safe_algo", "bypass")
    _configure_safe_algo(cfg.policy.safe_controller.safe_algo, safe_algo, **kwargs)

    if cfg.policy.safe_controller.safe_algo.class_name != "ByPassSafeControl":
        cfg.policy.safe_controller.safe_algo.control_weight = CONTROL_WEIGHT.copy()
        if "FixedBase" in cfg.robot.cfg.class_name:
            cfg.policy.safe_controller.safe_algo.control_weight = (
                cfg.policy.safe_controller.safe_algo.control_weight[:17]
            )
        elif "RightArm" in cfg.robot.cfg.class_name:
            cfg.policy.safe_controller.safe_algo.control_weight = (
                cfg.policy.safe_controller.safe_algo.control_weight[3:10]
            )
        elif "DualArm" in cfg.robot.cfg.class_name:
            cfg.policy.safe_controller.safe_algo.control_weight = (
                cfg.policy.safe_controller.safe_algo.control_weight[3:17]
            )
        else:
            cfg.policy.safe_controller.safe_algo.control_weight = (
                cfg.policy.safe_controller.safe_algo.control_weight[:20]
            )
        if kwargs.get("control_weight", None) is not None:
            cfg.policy.safe_controller.safe_algo.control_weight = list(kwargs["control_weight"])

    _configure_safety_index(
        cfg.policy.safe_controller.safety_index,
        kwargs.get("safety_index", "si1"),
        **kwargs,
    )
    cfg.policy.safe_controller.safety_index.enable_self_collision = bool(
        kwargs.get("enable_self_collision", False)
    )
    return cfg


def config_pipeline(cfg: PipelineConfig, **kwargs):
    """Configure pipeline settings."""
    cfg.robot.cfg.class_name = kwargs.get("robot_cfg") or "UnitreeG1FixedBaseDynamic1Config"
    # Teleoperation safety uses SPARK's frame-based collision volumes. The
    # optional Pinocchio geometry model is intended for motion planners and is
    # expensive (and parser-version-sensitive), so do not build it here unless
    # explicitly requested.
    cfg.robot.kinematics.load_collision_geometry = bool(
        kwargs.get("load_collision_geometry", False)
    )
    cfg.enable_logger = _value_or_default(kwargs.get("enable_logger", None), False)
    cfg.enable_plotter = _value_or_default(kwargs.get("enable_plotter", None), False)
    cfg.metric_selection.dof_pos = True
    cfg.metric_selection.dof_vel = True
    cfg.metric_selection.dist_goal_base = False
    cfg.metric_selection.trigger_safe_controller = False
    _set_if_not_none(cfg, "max_num_steps", kwargs.get("max_num_steps", None))
    cfg.profile_frequency = bool(kwargs.get("profile_frequency", False))
    cfg.render_robot_collision_volumes = _value_or_default(
        kwargs.get("render_robot_collision_volumes", None), True
    )
    # Isaac's link-mounted RGB-D sensor represents perception input, not the
    # operator debug viewport. Do not place synthetic safety/goal geometry in
    # sensor frames; physical robot links and obstacle geometry remain visible.
    if kwargs.get("backend", "mujoco") == "isaac" and kwargs.get("enable_camera", False):
        cfg.render_robot_collision_volumes = False
        cfg.render_goal_frames = False
        cfg.render_task_debug_frames = False
        cfg.render_action_debug_frames = False
        cfg.render_robot_reference_frames = False
        cfg.enable_safe_zone_render = False
    if kwargs.get("max_num_steps", None) is not None and hasattr(
        cfg.env.task, "max_episode_length"
    ):
        cfg.env.task.max_episode_length = kwargs["max_num_steps"] + 1
    return cfg


def build_config(**kwargs):
    """Build the backend-neutral Unitree G1 teleop configuration."""
    cfg = PipelineConfig()
    cfg = config_pipeline(cfg, **kwargs)
    cfg = config_task_module(cfg, **kwargs)
    cfg = config_agent_module(cfg, **kwargs)
    cfg = config_policy_module(cfg, **kwargs)
    cfg = config_safety_module(cfg, **kwargs)
    return cfg


def run(**kwargs):
    """Main execution block to run the teleop pipeline."""
    num_envs = int(kwargs.get("num_envs", 1))
    if num_envs != 1:
        raise ValueError(
            "Unitree teleoperation is interactive and requires --num-envs 1; "
            "use run_unitree_g1_benchmark.py for batched environments"
        )
    policy_config = kwargs.get("policy_config", "TeleopPIDPolicy")
    if policy_config == "UnitreeG1SonicPolicy":
        try:
            from example.unitree_g1.sonic_support import run as run_sonic
        except ModuleNotFoundError as exc:
            if exc.name != "example":
                raise
            # Direct execution puts this script's directory, rather than the
            # repository root, on sys.path.
            from sonic_support import run as run_sonic

        sonic_kwargs = dict(kwargs)
        sonic_kwargs.pop("policy_config", None)
        if not sonic_kwargs.get("robot_cfg") or "WholeBody" not in sonic_kwargs["robot_cfg"]:
            # SONIC's released policy and reference Isaac/MuJoCo mechanism are
            # 29-DoF. Hands remain an explicit opt-in instead of silently
            # changing the articulation selected by the unified launcher.
            sonic_kwargs["robot_cfg"] = "UnitreeG1WholeBodyDynamic1Config"
        return run_sonic(**sonic_kwargs)
    if policy_config == "UnitreeG1SportSafePolicy":
        from spark_policy.composed_policy.unitree_g1.sport_safe import (
            UnitreeG1SportSafePolicyConfig,
        )
        from spark_pipeline.teleop.unitree_g1_whole_body_config import run as run_unitree

        sport_kwargs = dict(kwargs)
        sport_kwargs.pop("policy_config", None)
        sport_kwargs["composed_policy_class"] = "UnitreeG1SportSafePolicy"
        sport_kwargs["executor_policy_class"] = "UnitreeG1SportExecutorAdapter"
        sport_kwargs["use_isaac_tensor_backend"] = True
        # Policy-specific timing, tracker limits, and gain selection live with
        # the policy. CLI values remain explicit user overrides.
        for name, value in UnitreeG1SportSafePolicyConfig().teleop_overrides().items():
            if sport_kwargs.get(name) is None:
                sport_kwargs[name] = value
        if not sport_kwargs.get("robot_cfg") or "WholeBody" not in sport_kwargs["robot_cfg"]:
            sport_kwargs["robot_cfg"] = "UnitreeG1WholeBodyWithHandDynamic1Config"
        return run_unitree(**sport_kwargs)
    if policy_config == "UnitreeG1WBTPolicy":
        from spark_pipeline.teleop.unitree_g1_whole_body_config import run as run_unitree

        wbt_kwargs = dict(kwargs)
        wbt_kwargs.pop("policy_config", None)
        # WBT uses one IsaacLab plant and native-drive contract for both one
        # and many environments. The one-environment adapter only restores
        # the ordinary scalar feedback/action API expected by teleoperation.
        wbt_kwargs["use_isaac_tensor_backend"] = True
        if wbt_kwargs.get("obstacle_range") is not None:
            values = wbt_kwargs["obstacle_range"]
            wbt_kwargs["obstacle_range"] = [
                tuple(values[index : index + 2]) for index in range(0, 6, 2)
            ]
        if not wbt_kwargs.get("robot_cfg") or "WholeBody" not in wbt_kwargs["robot_cfg"]:
            wbt_kwargs["robot_cfg"] = "UnitreeG1WholeBodyWithHandDynamic1Config"
        return run_unitree(**wbt_kwargs)
    cfg = build_config(**kwargs)
    backend = kwargs.get("backend", "mujoco")
    configure_simulation_presentation(cfg, backend=backend)
    isaac_sim_dynamics = bool(backend == "isaac" and cfg.env.agent.use_sim_dynamics)
    simulation_app = None
    pipeline = None
    renderer_process = None
    renderer_attached = False
    try:
        if backend == "isaac" and cfg.env.agent.enable_viewer and not isaac_sim_dynamics:
            from spark_agent.simulation.isaac import IsaacRendererProcess

            # Modeled dynamics has no local Kit application, so visualization
            # remains fully isolated in the existing renderer process.
            renderer_process = IsaacRendererProcess(
                robot_cfg_class_name=cfg.robot.cfg.class_name,
                fixed_base=_is_fixed_base_robot(cfg.robot.cfg.class_name),
                enable_keyboard_control=cfg.env.agent.enable_keyboard_control,
                enable_hand_control=cfg.env.agent.enable_hand_control,
                viewer_show_simulation_info=cfg.env.agent.viewer_show_simulation_info,
                num_obstacle_debug=cfg.env.agent.obstacle_debug["num_obstacle"],
                manual_movement_step_size=cfg.env.agent.obstacle_debug["manual_movement_step_size"],
                viewer_hz=kwargs.get("viewer_hz", 10.0),
                render_quality=kwargs.get("isaac_render_quality", "performance"),
                renderer=kwargs.get("isaac_renderer"),
                anti_aliasing=kwargs.get("isaac_anti_aliasing"),
                width=kwargs.get("viewer_width"),
                height=kwargs.get("viewer_height"),
                limit_cpu_threads=kwargs.get("isaac_cpu_threads", 8),
                device=kwargs.get("isaac_renderer_device", "cpu"),
                viewer_config=cfg.env.agent.viewer_config,
                enable_camera=cfg.env.agent.enable_camera,
                camera_width=cfg.env.agent.camera_width,
                camera_height=cfg.env.agent.camera_height,
                camera_rate_hz=cfg.env.agent.camera_rate_hz,
                camera_display=cfg.env.agent.camera_display,
                camera_config=cfg.env.agent.camera_config,
            )

        if isaac_sim_dynamics:
            # PhysX owns the authoritative state. Isaac Sim 6 cannot reliably
            # initialize two Kit applications concurrently on one workstation,
            # so simulator-dynamics mode uses this same Kit instance for the
            # optional viewport. Rendering remains after env.step/policy.act.
            from isaacsim import SimulationApp
            from spark_agent.simulation.isaac.render_quality import isaac_render_preset

            enable_viewer = bool(cfg.env.agent.enable_viewer)
            render_config = isaac_render_preset(
                kwargs.get("isaac_render_quality", "performance"),
                renderer=kwargs.get("isaac_renderer"),
                anti_aliasing=kwargs.get("isaac_anti_aliasing"),
                width=kwargs.get("viewer_width"),
                height=kwargs.get("viewer_height"),
            )
            launch_config = {
                "headless": not enable_viewer,
                "hide_ui": False if enable_viewer else True,
                "width": render_config["width"],
                "height": render_config["height"],
                "window_width": render_config["width"],
                "window_height": render_config["height"],
                "renderer": render_config["renderer"],
                "anti_aliasing": render_config["anti_aliasing"],
                "samples_per_pixel_per_frame": render_config["samples_per_pixel_per_frame"],
                "denoiser": render_config["denoiser"],
                "max_bounces": render_config["max_bounces"],
                "limit_cpu_threads": kwargs.get("isaac_cpu_threads", 8),
                "disable_viewport_updates": not enable_viewer,
                # SPARK owns keyboard teleoperation. Disable Kit editor
                # hotkeys so Space cannot pause the timeline, and configure
                # the viewport as a presentation-only window.
                "extra_args": [
                    "--/exts/omni.kit.hotkeys.core/hotkeys_enabled=false",
                    "--/app/viewport/defaults/noTitleBar=true",
                    "--/persistent/app/viewport/noPadding=true",
                    "--/exts/omni.kit.viewport.window/startup/dockTabInvisible=true",
                ],
            }
            simulation_app = SimulationApp(launch_config)
            cfg.env.agent.render = enable_viewer
            cfg.env.agent.viewer_hz = kwargs.get("viewer_hz", 10.0)

        pipeline = Pipeline(cfg)
        if simulation_app is not None:
            pipeline.env.agent.attach_simulation_app(simulation_app)
        if renderer_process is not None:
            pipeline.env.agent.attach_renderer_process(renderer_process)
            renderer_attached = True

        pipeline.run(save_path=kwargs.get("save_path", None))
    finally:
        # BaseGoalPipeline closes the viewer normally; this also covers Ctrl-C
        # and policy exceptions without leaving an Isaac Kit process behind.
        if pipeline is not None:
            pipeline.env.agent.close_viewer()
            if isaac_sim_dynamics:
                pipeline.env.agent.close()
        if renderer_process is not None and not renderer_attached:
            renderer_process.close(force=True)
        if simulation_app is not None:
            simulation_app.close(wait_for_replicator=False)

    if backend == "isaac" and pipeline is not None:
        renderer_status = getattr(pipeline.env.agent, "renderer_process_status", None)
        if renderer_status:
            print(f"Isaac renderer process profile: {renderer_status}")
            if "error" in renderer_status:
                raise RuntimeError(renderer_status["error"])
    return pipeline


def _parse_cli_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Run one Unitree G1 safe-teleop policy with a MuJoCo or Isaac viewer."
    )
    parser.add_argument(
        "--backend",
        choices=("mujoco", "isaac"),
        default="mujoco",
        help=(
            "Simulation backend when --use-real=false. Keep mujoco for the "
            "current real-agent workflow; configure its optional viewer mirror "
            "with --enable-digital-twin and --digital-twin-backend."
        ),
    )
    parser.add_argument(
        "--num-envs",
        type=int,
        default=1,
        help="Teleoperation uses exactly one interactive environment.",
    )
    parser.add_argument(
        "--policy-config",
        choices=(
            "TeleopPIDPolicy",
            "UnitreeG1WBTPolicy",
            "UnitreeG1SonicPolicy",
            "UnitreeG1SportSafePolicy",
        ),
        default="TeleopPIDPolicy",
        help="Registered spark_policy class used by teleoperation.",
    )
    parser.add_argument("--use-real", type=_parse_bool, nargs="?", const=True, default=USE_REAL)
    parser.add_argument(
        "--robot-config",
        "--robot-cfg",
        dest="robot_cfg",
        default="UnitreeG1DualArmWithHandDynamic1Config",
        help="Robot configuration class (both flag spellings are accepted).",
    )
    parser.add_argument("--agent-cfg", default=None)
    parser.add_argument(
        "--safe-algo",
        choices=("bypass", "ssa", "rssa", "sss", "rsss", "cbf", "rcbf", "pfm", "sma"),
        default="rssa",
    )
    parser.add_argument(
        "--enable-self-collision",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=True,
    )
    parser.add_argument(
        "--safety-index", choices=("si1", "si1a", "si2", "si2a", "si2nn"), default="si1"
    )
    parser.add_argument("--min-distance", type=float, default=None)
    parser.add_argument("--num-obstacle-task", type=int, default=0)
    parser.add_argument("--obstacle-init", type=float, nargs=3, default=None)
    parser.add_argument("--obstacle-velocity", type=float, default=None)
    parser.add_argument("--obstacle-direction", type=float, nargs=3, default=None)
    parser.add_argument(
        "--arm-goal-enable",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=None,
        help=(
            "Enable Cartesian arm-goal tracking. SONIC and WBT enable it by "
            "default; pass false to hold the policy's default upper body."
        ),
    )
    parser.add_argument("--obstacle-range", type=float, nargs=6, default=None)
    parser.add_argument("--num-obstacle-debug", type=int, default=None)
    parser.add_argument("--control-weight", type=float, nargs="+", default=None)
    parser.add_argument("--locomotion-control-weight", type=float, default=None)
    parser.add_argument("--upper-body-control-weight", type=float, default=None)
    parser.add_argument("--waist-control-weight", type=float, default=None)
    parser.add_argument("--height-control-weight", type=float, default=None)
    wbt = parser.add_argument_group("WBT policy and safety envelope")
    wbt.add_argument("--base-position-kp", type=float, nargs=3, default=None)
    wbt.add_argument("--base-orientation-kp", type=float, nargs=3, default=None)
    wbt.add_argument("--base-xy-yaw-limit", type=float, nargs=3, default=None)
    wbt.add_argument("--base-command-rate-limit", type=float, nargs=5, default=None)
    wbt.add_argument("--base-command-deadband", type=float, nargs=5, default=None)
    wbt.add_argument("--loco-start-threshold", type=float, default=None)
    wbt.add_argument("--safe-loco-start-threshold", type=float, default=None)
    wbt.add_argument("--stop-threshold", type=float, nargs=3, default=None)
    wbt.add_argument("--base-goal-xy-deadband", type=float, default=None)
    wbt.add_argument("--base-goal-resume-distance", type=float, default=None)
    wbt.add_argument("--base-goal-yaw-deadband", type=float, default=None)
    wbt.add_argument("--base-goal-yaw-resume-distance", type=float, default=None)
    wbt.add_argument("--pre-safe-loco-command-limit", type=float, nargs=3, default=None)
    wbt.add_argument("--pre-safe-loco-command-rate-limit", type=float, nargs=3, default=None)
    wbt.add_argument("--pre-safe-height-command-limit", type=float, default=None)
    wbt.add_argument("--eta-ssa", type=float, default=None)
    wbt.add_argument("--slack-weight", type=float, default=None)
    parser.add_argument("--manual-movement-step-size", type=float, default=None)
    parser.add_argument(
        "--goal-tracking-type",
        choices=("legged", "pid"),
        default=None,
        help="Nominal base-goal tracker used by SonicSafe/WBTSafe.",
    )
    parser.add_argument("--enable-ros", type=_parse_bool, nargs="?", const=True, default=None)
    parser.add_argument("--ros-version", choices=("ros1", "ros2"), default=None)
    parser.add_argument(
        "--headless",
        dest="enable_viewer",
        action="store_false",
        default=True,
        help="Disable the viewer; the window is enabled by default.",
    )
    parser.add_argument(
        "--enable-keyboard-control", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument("--enable-logger", type=_parse_bool, nargs="?", const=True, default=None)
    parser.add_argument("--enable-plotter", type=_parse_bool, nargs="?", const=True, default=None)
    parser.add_argument("--send-cmd", type=_parse_bool, nargs="?", const=True, default=None)
    parser.add_argument("--net", default="")
    parser.add_argument("--unitree-model", default="g1")
    parser.add_argument("--level", choices=("low", "high"), default=None)
    parser.add_argument(
        "--enable-digital-twin",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=True,
        help="Mirror the real Unitree state in a MuJoCo viewer; pass false to disable it.",
    )
    parser.add_argument(
        "--digital-twin-backend",
        choices=("mujoco",),
        default="mujoco",
        help="Viewer backend for the optional real-agent twin (currently MuJoCo only).",
    )
    parser.add_argument("--digital-twin-sync-hz", type=float, default=500.0)
    parser.add_argument(
        "--enable-hand-control", type=_parse_bool, nargs="?", const=True, default=None
    )
    add_teleop_camera_arguments(parser)
    add_teleop_viewer_arguments(parser)
    parser.add_argument("--dt", type=float, default=None)
    parser.add_argument("--control-decimation", type=int, default=None)
    parser.add_argument(
        "--use-sim-dynamics",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=True,
        help="Use backend physics; false uses SPARK modeled dynamics for backend-parity checks",
    )
    parser.add_argument("--task-dt", type=float, default=None)
    parser.add_argument("--max-num-steps", type=int, default=None)
    parser.add_argument("--profile-frequency", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--environment-representation",
        choices=("sphere", "point_cloud"),
        default="sphere",
    )
    parser.add_argument("--points-per-obstacle", type=int, default=64)
    parser.add_argument("--max-visualized-points", type=int, default=200)
    parser.add_argument("--object-mesh-directory", default=None)
    parser.add_argument("--object-mesh-scale", type=float, default=1.0)
    parser.add_argument("--safety-nearest-points-per-link", type=int, default=4)
    parser.add_argument(
        "--load-collision-geometry",
        action="store_true",
        help="Build Pinocchio collision meshes for policies that explicitly need them",
    )
    parser.add_argument("--viewer-hz", type=float, default=10.0)
    parser.add_argument("--viewer-lookat", type=float, nargs=3, default=None)
    parser.add_argument("--viewer-distance", type=float, default=None)
    parser.add_argument("--viewer-azimuth", type=float, default=None)
    parser.add_argument("--viewer-elevation", type=float, default=None)
    parser.add_argument("--viewer-vertical-fov", type=float, default=None)
    parser.add_argument(
        "--render-robot-collision-volumes",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=True,
        help="Show or hide the robot's collision-volume spheres/boxes",
    )
    parser.add_argument(
        "--isaac-render-quality",
        choices=("performance", "quality", "cinematic"),
        default="performance",
        help="Shared Isaac rendering preset; explicit rendering options override it.",
    )
    parser.add_argument("--viewer-width", type=int, default=None)
    parser.add_argument("--viewer-height", type=int, default=None)
    parser.add_argument(
        "--isaac-renderer",
        choices=("RaytracedLighting", "RealTimePathTracing", "PathTracing"),
        default=None,
        help=(
            "Isaac renderer. RaytracedLighting is the interactive default; full RTX modes "
            "may perform a long first-run shader compilation."
        ),
    )
    parser.add_argument("--isaac-anti-aliasing", type=int, default=None)
    parser.add_argument("--isaac-cpu-threads", type=int, default=4)
    parser.add_argument(
        "--isaac-device",
        default=None,
        help=(
            "Isaac PhysX tensor device; defaults to CPU for one environment. "
            "Use cuda:0 when benchmarking future batched environments"
        ),
    )
    parser.add_argument(
        "--isaac-renderer-device",
        default="cpu",
        help="Tensor device used by the independent Isaac renderer articulation",
    )
    parser.add_argument("--save-path", default=None)
    sonic = parser.add_argument_group("SONIC policy server")
    sonic.add_argument("--sonic-endpoint", default="tcp://127.0.0.1:5560")
    sonic.add_argument(
        "--auto-launch-sonic-server",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=None,
    )
    sonic.add_argument(
        "--sonic-deploy-root",
        default=None,
        help=(
            "Path to GR00T-WholeBodyControl/gear_sonic_deploy. Defaults to "
            "SPARK_SONIC_DEPLOY_ROOT or the standard sibling checkout."
        ),
    )
    sonic.add_argument("--sonic-timeout-ms", type=int, default=None)
    sonic.add_argument("--sonic-fallback-mode", choices=("hold", "default", "raise"), default=None)
    sonic.add_argument(
        "--sonic-locomotion-mode",
        choices=("slow", "walk", "run", "hybrid"),
        default=None,
        help=(
            "SONIC gait mode. By default PID uses WALK far from the goal and "
            "SLOW_WALK near it; legged tracking keeps its configured mode."
        ),
    )
    sonic.add_argument("--sonic-policy-precision", choices=("16", "32"), default=None)
    sonic.add_argument("--sonic-planner-precision", choices=("16", "32"), default=None)
    sonic.add_argument("--sonic-server-startup-timeout", type=float, default=None)
    return vars(parser.parse_args(argv))


if __name__ == "__main__":
    run(**_parse_cli_args())
