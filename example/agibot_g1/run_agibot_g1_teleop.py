import argparse

from spark_pipeline import TeleopPipeline as Pipeline
from spark_pipeline import AgiBotG1TeleopPipelineConfig as PipelineConfig
from spark_pipeline import run_simulation_pipeline
from spark_pipeline.teleop.camera import (
    add_teleop_camera_arguments,
    add_teleop_viewer_arguments,
    configure_teleop_camera,
    configure_teleop_viewer,
)


ROBOT_CONFIGS = (
    "AgiBotG1FixedBaseDynamic1Config",
    "AgiBotG1DualArmDynamic1Config",
    "AgiBotG1RightArmDynamic1Config",
    "AgiBotG1MobileBaseDynamic1Config",
    "AgiBotG1MobileBaseDynamic2Config",
    "AgiBotG1MobileBaseUnicycleDynamic1Config",
    "AgiBotG1MobileBaseBicycleDynamic2Config",
)


def _parse_bool(value):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if normalized in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Expected a boolean value, got {value!r}")


def _control_dim(robot_cfg_name: str) -> int:
    import spark_robot

    return len(getattr(spark_robot, robot_cfg_name)().Control)


def _uses_base_goal(robot_cfg_name: str) -> bool:
    return "FixedBase" in robot_cfg_name or "MobileBase" in robot_cfg_name


def config_task_module(cfg: PipelineConfig, **kwargs):
    cfg.env.task.enable_ros = False
    cfg.env.task.mode = "Velocity"
    cfg.env.task.num_obstacle_task = 0
    robot_cfg_name = kwargs.get("robot_cfg", "AgiBotG1FixedBaseDynamic1Config")
    cfg.env.task.use_dual_arm = "RightArm" not in robot_cfg_name
    cfg.env.task.base_goal_enable = _uses_base_goal(robot_cfg_name)
    cfg.env.task.base_goal_init = kwargs.get("base_goal_init", [0.0, 0.0, 0.793])
    return cfg


def config_agent_module(cfg: PipelineConfig, **kwargs):
    import spark_robot

    cfg.env.agent.class_name = kwargs.get("agent_cfg")
    cfg.env.agent.obstacle_debug["manual_movement_step_size"] = kwargs.get(
        "manual_movement_step_size", 0.02
    )
    cfg.env.agent.obstacle_debug["num_obstacle"] = kwargs.get("num_obstacle_debug", 1)
    cfg.env.agent.dynamics_backend = kwargs.get("dynamics_backend", "simulator")
    cfg.env.agent.use_sim_dynamics = cfg.env.agent.dynamics_backend == "simulator"
    robot_cfg_name = kwargs.get("robot_cfg", "AgiBotG1FixedBaseDynamic1Config")
    simulator_dynamics = getattr(spark_robot, robot_cfg_name)().simulator_dynamics
    dt = kwargs.get("dt")
    if dt is None:
        dt = kwargs.get("physics_dt")
    cfg.env.agent.dt = float(simulator_dynamics.physics_dt if dt is None else dt)
    control_decimation = kwargs.get("control_decimation")
    cfg.env.agent.control_decimation = int(
        simulator_dynamics.control_decimation if control_decimation is None else control_decimation
    )
    cfg.env.task.dt = cfg.env.agent.dt * cfg.env.agent.control_decimation
    cfg.env.agent.enable_viewer = bool(kwargs.get("enable_viewer", True))
    keyboard_control = kwargs.get("enable_keyboard_control")
    cfg.env.agent.enable_keyboard_control = (
        cfg.env.agent.enable_viewer if keyboard_control is None else bool(keyboard_control)
    )
    configure_teleop_camera(cfg.env.agent, kwargs)
    configure_teleop_viewer(cfg.env.agent, kwargs)
    cfg.env.agent.real_time = bool(kwargs.get("real_time", True))
    cfg.env.agent.device = kwargs.get("device", "cpu")
    return cfg


def config_policy_module(cfg: PipelineConfig, **kwargs):
    return cfg


def config_safety_module(cfg: PipelineConfig, **kwargs):
    safe_algo = kwargs.get("safe_algo", "bypass")
    control_dim = _control_dim(kwargs.get("robot_cfg", "AgiBotG1FixedBaseDynamic1Config"))
    if safe_algo == "bypass":
        cfg.policy.safe_controller.safe_algo.class_name = "ByPassSafeControl"
    elif safe_algo == "ssa":
        cfg.policy.safe_controller.safe_algo.class_name = "BasicSafeSetAlgorithm"
        cfg.policy.safe_controller.safe_algo.eta_ssa = 0.1
        cfg.policy.safe_controller.safe_algo.control_weight = [1.0] * control_dim
    elif safe_algo == "rssa":
        cfg.policy.safe_controller.safe_algo.class_name = "RelaxedSafeSetAlgorithm"
        cfg.policy.safe_controller.safe_algo.eta_ssa = 0.1
        cfg.policy.safe_controller.safe_algo.slack_weight = 1e3
        cfg.policy.safe_controller.safe_algo.control_weight = [1.0] * control_dim
    elif safe_algo == "sss":
        cfg.policy.safe_controller.safe_algo.class_name = "BasicSublevelSafeSetAlgorithm"
        cfg.policy.safe_controller.safe_algo.lambda_sss = 10.0
        cfg.policy.safe_controller.safe_algo.control_weight = [1.0] * control_dim
    elif safe_algo == "rsss":
        cfg.policy.safe_controller.safe_algo.class_name = "RelaxedSublevelSafeSetAlgorithm"
        cfg.policy.safe_controller.safe_algo.lambda_sss = 10.0
        cfg.policy.safe_controller.safe_algo.slack_weight = 1e3
        cfg.policy.safe_controller.safe_algo.control_weight = [1.0] * control_dim
    elif safe_algo == "cbf":
        cfg.policy.safe_controller.safe_algo.class_name = "BasicControlBarrierFunction"
        cfg.policy.safe_controller.safe_algo.lambda_cbf = 10.0
        cfg.policy.safe_controller.safe_algo.control_weight = [1.0] * control_dim
    elif safe_algo == "rcbf":
        cfg.policy.safe_controller.safe_algo.class_name = "RelaxedControlBarrierFunction"
        cfg.policy.safe_controller.safe_algo.lambda_cbf = 10.0
        cfg.policy.safe_controller.safe_algo.slack_weight = 1e3
        cfg.policy.safe_controller.safe_algo.control_weight = [1.0] * control_dim
    elif safe_algo == "pfm":
        cfg.policy.safe_controller.safe_algo.class_name = "BasicPotentialFieldMethod"
        cfg.policy.safe_controller.safe_algo.c_pfm = 1.0
    elif safe_algo == "sma":
        cfg.policy.safe_controller.safe_algo.class_name = "BasicSlidingModeAlgorithm"
        cfg.policy.safe_controller.safe_algo.c_sma = 1.0

    safety_index = kwargs.get("safety_index")
    if safety_index is None:
        import spark_robot

        dynamics_order = getattr(spark_robot, kwargs["robot_cfg"])().dynamics_order
        safety_index = "si1" if dynamics_order == 1 else "si2"
    if safety_index == "si1":
        cfg.policy.safe_controller.safety_index.class_name = "FirstOrderCollisionSafetyIndex"
    elif safety_index == "si1a":
        cfg.policy.safe_controller.safety_index.class_name = "FirstOrderCollisionSafetyIndexApprox"
    elif safety_index == "si2":
        cfg.policy.safe_controller.safety_index.class_name = "SecondOrderCollisionSafetyIndex"
        cfg.policy.safe_controller.safety_index.phi_n = 1.0
        cfg.policy.safe_controller.safety_index.phi_k = 1.0
    elif safety_index == "si2a":
        cfg.policy.safe_controller.safety_index.class_name = "SecondOrderCollisionSafetyIndexApprox"
        cfg.policy.safe_controller.safety_index.phi_n = 1.0
        cfg.policy.safe_controller.safety_index.phi_k = 1.0
    elif safety_index == "si2nn":
        cfg.policy.safe_controller.safety_index.class_name = "SecondOrderNNCollisionSafetyIndex"
        cfg.policy.safe_controller.safety_index.phi_n = 2.0
        cfg.policy.safe_controller.safety_index.phi_k = 1.0
        cfg.policy.safe_controller.safety_index.phi_nn_path = "n_2_scalar.onnx"

    cfg.policy.safe_controller.safety_index.enable_self_collision = False
    cfg.policy.safe_controller.safety_index.min_distance["environment"] = float(
        kwargs.get("minimum_distance", 0.05)
    )
    cfg.policy.safe_controller.safety_index.min_distance["self"] = 0.0
    return cfg


def config_pipeline(cfg: PipelineConfig, **kwargs):
    cfg.robot.cfg.class_name = kwargs.get("robot_cfg")
    cfg.max_num_steps = kwargs.get("max_num_steps", cfg.max_num_steps)
    cfg.max_num_reset = kwargs.get("max_num_reset", cfg.max_num_reset)
    cfg.profile_frequency = bool(kwargs.get("profile_frequency", False))
    cfg.enable_logger = False
    cfg.enable_plotter = False
    cfg.enable_safe_zone_render = False
    cfg.render_robot_collision_volumes = bool(kwargs.get("render_robot_collision_volumes", True))
    cfg.metric_selection.dof_pos = True
    cfg.metric_selection.dof_vel = True
    cfg.metric_selection.dist_goal_base = False
    cfg.metric_selection.trigger_safe_controller = False
    return cfg


def run(**kwargs):
    from spark_robot import get_agent_class_name

    backend = kwargs.get("backend", "mujoco")
    if int(kwargs.get("num_envs", 1)) != 1:
        raise ValueError("AgiBot safe teleoperation supports exactly one environment")
    robot_cfg_name = kwargs.get("robot_cfg")
    if not robot_cfg_name:
        raise ValueError("robot_cfg must select an AgiBot G1 configuration")
    kwargs = dict(kwargs)
    kwargs["agent_cfg"] = kwargs.get("agent_cfg") or get_agent_class_name(
        robot_cfg_name, backend=backend
    )

    cfg = PipelineConfig()
    cfg = config_pipeline(cfg, **kwargs)
    cfg = config_task_module(cfg, **kwargs)
    cfg = config_agent_module(cfg, **kwargs)
    cfg = config_policy_module(cfg, **kwargs)
    cfg = config_safety_module(cfg, **kwargs)

    return run_simulation_pipeline(
        cfg,
        backend=backend,
        pipeline_class=Pipeline,
        save_path=kwargs.get("save_path"),
    )


def _parse_cli_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Run single-environment AgiBot G1 safe teleoperation."
    )
    parser.add_argument("--backend", choices=("mujoco", "isaac"), default="mujoco")
    parser.add_argument(
        "--robot-config",
        "--robot-cfg",
        dest="robot_cfg",
        choices=ROBOT_CONFIGS,
        default="AgiBotG1FixedBaseDynamic1Config",
    )
    parser.add_argument("--agent-cfg", default=None)
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument(
        "--device",
        default="cpu",
        help="Isaac physics device; CPU is faster for one interactive environment.",
    )
    parser.add_argument("--dynamics-backend", choices=("simulator", "model"), default="simulator")
    parser.add_argument(
        "--safe-algo",
        choices=("bypass", "ssa", "rssa", "sss", "rsss", "cbf", "rcbf", "pfm", "sma"),
        default="rssa",
    )
    parser.add_argument(
        "--safety-index",
        choices=("si1", "si1a", "si2", "si2a", "si2nn"),
        default=None,
        help="Safety-index model; defaults to si1 or si2 according to dynamics order.",
    )
    parser.add_argument(
        "--minimum-distance",
        type=float,
        default=0.05,
        help="Required signed robot-obstacle surface clearance in metres.",
    )
    parser.add_argument("--num-obstacle-task", type=int, default=0)
    parser.add_argument("--num-obstacle-debug", type=int, default=1)
    parser.add_argument(
        "--render-robot-collision-volumes",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=True,
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        help="Override the selected AgiBot configuration's physics step.",
    )
    parser.add_argument(
        "--control-decimation",
        type=int,
        default=None,
        help="Override the selected AgiBot configuration's physics substeps.",
    )
    parser.add_argument("--max-num-steps", type=int, default=-1)
    parser.add_argument("--profile-frequency", action="store_true")
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
    add_teleop_camera_arguments(parser)
    add_teleop_viewer_arguments(parser)
    parser.add_argument("--real-time", type=_parse_bool, nargs="?", const=True, default=True)
    parser.add_argument("--save-path", default=None)
    return vars(parser.parse_args(argv))


if __name__ == "__main__":
    run(**_parse_cli_args())
