from copy import deepcopy
import traceback

import numpy as np
from scipy.spatial.transform import Rotation as R

from spark_pipeline import (
    TeleopPipeline,
    UNITREE_G1_WBT_PRE_SAFE_CONTROL_NAMES,
    UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig,
    UnitreeG1WBTCascadeWholeBodyWithHandTeleopPipelineConfig,
)
from spark_pipeline.teleop.unitree_g1_wbt_whole_body_teleop_pipeline_config import (
    UNITREE_G1_WBT_SAFE_TELEOP_DEFAULTS,
)
from spark_pipeline.simulation_runtime import (
    ISAAC_INTERACTIVE_VIEWER_HZ,
    configure_simulation_presentation,
)


USE_REAL = False
WBT_RUN_DEFAULTS = deepcopy(UNITREE_G1_WBT_SAFE_TELEOP_DEFAULTS)
WBT_RUN_DEFAULTS["use_real"] = USE_REAL


def _set_if_not_none(obj, name, value):
    if value is not None:
        setattr(obj, name, value)


def _value_or_default(value, default):
    return default if value is None else value


def _with_run_defaults(kwargs):
    merged = deepcopy(WBT_RUN_DEFAULTS)
    for key, value in kwargs.items():
        if value is not None:
            merged[key] = value
    return merged


def _control_weight(value):
    if value is None:
        return None
    weight = list(value)
    expected = len(UNITREE_G1_WBT_PRE_SAFE_CONTROL_NAMES)
    if len(weight) != expected:
        raise ValueError(f"control_weight must have {expected} entries, got {len(weight)}")
    return weight


def _configure_wbt_arm_kinematics(cfg, **kwargs):
    arm_robot_cfg = (
        kwargs.get("wbt_arm_robot_cfg")
        or kwargs.get("arm_robot_cfg")
        or "UnitreeG1DualArmDynamic1Config"
    )
    arm_kinematics_class_name = (
        kwargs.get("wbt_arm_kinematics_class_name")
        or kwargs.get("arm_kinematics_class_name")
        or "UnitreeG1DualArmKinematics"
    )
    if hasattr(cfg.policy, "wbt_arm_robot_cfg"):
        _set_if_not_none(cfg.policy.wbt_arm_robot_cfg, "class_name", arm_robot_cfg)
    if hasattr(cfg.policy, "wbt_arm_kinematics"):
        _set_if_not_none(cfg.policy.wbt_arm_kinematics, "class_name", arm_kinematics_class_name)
    return cfg


def _cfg_class(robot_cfg):
    if robot_cfg is not None and "WithHand" in robot_cfg:
        return UnitreeG1WBTCascadeWholeBodyWithHandTeleopPipelineConfig
    return UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig


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


def _normalize_min_distance(value):
    if value is None:
        return {"environment": 0.05, "self": 0.0}
    if isinstance(value, dict):
        normalized = dict(value)
        normalized.setdefault("self", 0.0)
        return normalized
    return {"environment": float(value), "self": 0.0}


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

    safety_index_cfg.enable_self_collision = kwargs.get("enable_self_collision", True)
    safety_index_cfg.min_distance = _normalize_min_distance(kwargs.get("min_distance", None))


def config_task_module(cfg, **kwargs):
    _set_if_not_none(cfg.env.task, "class_name", kwargs.get("task_class_name", None))
    _set_if_not_none(cfg.env.task, "task_name", kwargs.get("task_name", None))
    cfg.env.task.enable_ros = _value_or_default(kwargs.get("enable_ros", None), False)
    cfg.env.task.mode = "Velocity"
    cfg.env.task.num_obstacle_task = _value_or_default(
        kwargs.get("num_obstacle_task", None), WBT_RUN_DEFAULTS["num_obstacle_task"]
    )
    cfg.env.task.obstacle_direction = _value_or_default(
        kwargs.get("obstacle_direction", None), [0.0, 1.0, 0.0]
    )
    cfg.env.task.obstacle_init = _value_or_default(
        kwargs.get("obstacle_init", None), [1.3, -0.7, 0.1]
    )
    cfg.env.task.obstacle_velocity = _value_or_default(kwargs.get("obstacle_velocity", None), 0.0)
    cfg.env.task.goal_right_velocity = _value_or_default(
        kwargs.get("goal_right_velocity", None), 0.0
    )
    cfg.env.task.goal_left_velocity = _value_or_default(kwargs.get("goal_left_velocity", None), 0.0)
    cfg.env.task.dt = _value_or_default(kwargs.get("task_dt", None), 0.02)
    cfg.env.task.base_goal_enable = _value_or_default(kwargs.get("base_goal_enable", None), True)
    cfg.env.task.gripper_debug_print = _value_or_default(
        kwargs.get("gripper_debug_print", None), False
    )
    _set_if_not_none(
        cfg.env.task, "teleop_upper_body_mode", kwargs.get("teleop_upper_body_mode", None)
    )
    _set_if_not_none(cfg.env.task, "ros_version", kwargs.get("ros_version", None))
    _set_if_not_none(cfg.env.task, "arm_goal_enable", kwargs.get("arm_goal_enable", None))
    _set_if_not_none(
        cfg.env.task,
        "arm_goal_init_from_current_ee",
        kwargs.get("arm_goal_init_from_current_ee", None),
    )
    _set_if_not_none(cfg.env.task, "goal_left_init", kwargs.get("goal_left_init", None))
    _set_if_not_none(cfg.env.task, "goal_right_init", kwargs.get("goal_right_init", None))
    _set_if_not_none(cfg.env.task, "base_goal_init", kwargs.get("base_goal_init", None))
    for name in (
        "mode",
        "obstacle_mode",
        "arm_goal_mode",
        "base_goal_mode",
        "obstacle_range",
        "obstacle_size",
        "obstacle_keepout",
        "robot_keepout",
        "obstacle_goal_keepaway",
        "deterministic_scenario_sampling",
        "left_arm_goal_range",
        "right_arm_goal_range",
        "base_goal_range",
        "base_goal_minimum_distance",
        "base_goal_relative_to_current",
        "base_goal_workspace_range",
        "base_goal_rot_range",
        "base_goal_velocity",
        "arm_goal_size",
        "arm_goal_minimum_distance",
        "arm_goal_pair_keepout",
        "base_goal_size",
        "base_goal_yaw_size",
        "arm_goal_reach_done",
        "base_goal_reach_done",
        "max_episode_length",
        "fall_height_threshold",
        "reset_on_success",
        "reset_on_timeout",
        "completion_mode",
        "resample_arm_goals_on_reset",
        "resample_base_goal_on_reset",
        "resample_obstacles_on_reset",
        "seed",
        "environment_representation",
        "points_per_obstacle",
        "minimum_points_per_obstacle",
        "dynamic_point_count",
        "regenerate_point_cloud_every_step",
        "point_radius",
        "object_mesh_path",
        "object_mesh_directory",
        "object_mesh_scale",
        "max_visualized_points",
    ):
        _set_if_not_none(cfg.env.task, name, kwargs.get(name, None))
    if hasattr(cfg.env.task, "ros_params"):
        cfg.env.task.ros_params["real_robot_location_topic"] = kwargs.get(
            "real_robot_location_topic",
            "/real_robot_location",
        )
        cfg.env.task.ros_params["robot_teleop_mode_topic"] = kwargs.get(
            "robot_teleop_mode_topic",
            "robot_teleop_mode",
        )
    return cfg


def config_agent_module(cfg, **kwargs):
    use_real = kwargs.get("use_real", USE_REAL)
    backend = kwargs.get("backend", "mujoco")
    cfg.env.agent.viewer_show_simulation_info = bool(
        kwargs.get("viewer_show_simulation_info", False)
    )
    if use_real and backend != "mujoco":
        raise ValueError("--backend isaac is currently a simulation workflow")
    if not use_real:
        import spark_robot

        timing = getattr(spark_robot, cfg.robot.cfg.class_name)().simulator_dynamics
        # WBT has a MobileBase public control contract but requires an
        # articulated whole-body simulator realization.
        default_agent = {
            "mujoco": "UnitreeG1WholeBodyMujocoAgent",
            "isaac": "UnitreeG1WholeBodyIsaacAgent",
        }[backend]
        cfg.env.agent.class_name = kwargs.get("agent_cfg") or default_agent
        cfg.env.agent.obstacle_debug["num_obstacle"] = _value_or_default(
            kwargs.get("num_obstacle_debug", None), 1
        )
        cfg.env.agent.obstacle_debug["manual_movement_step_size"] = _value_or_default(
            kwargs.get("manual_movement_step_size", None),
            0.02,
        )
        cfg.env.agent.enable_viewer = _value_or_default(kwargs.get("enable_viewer", None), True)
        cfg.env.agent.enable_keyboard_control = _value_or_default(
            kwargs.get("enable_keyboard_control", None), True
        )
        cfg.env.agent.real_time = _value_or_default(kwargs.get("real_time", None), True)
        cfg.env.agent.use_sim_dynamics = True
        cfg.env.agent.dynamics_backend = "simulator"
        cfg.env.agent.fixed_base = False
        cfg.env.agent.render = False
        cfg.env.agent.render_on_step = False
        cfg.env.agent.device = kwargs.get("isaac_device") or "cpu"
        if str(cfg.env.agent.device).startswith("cuda"):
            print(
                "[performance] Single-environment Isaac WBT is faster on CPU; "
                "CUDA is intended for batched environments. Use --isaac-device cpu "
                "for real-time teleoperation."
            )
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
        for name in (
            "record_video_path",
            "record_gif_path",
            "record_fps",
            "record_duration",
            "record_width",
            "record_height",
        ):
            _set_if_not_none(cfg.env.agent, name, kwargs.get(name))
        cfg.env.agent.enable_contact_visualization = bool(
            backend == "isaac"
            and _value_or_default(kwargs.get("enable_contact_visualization", None), True)
        )
        cfg.env.agent.contact_force_scale = _value_or_default(
            kwargs.get("contact_force_scale", None), 5.0e-4
        )
        cfg.env.agent.contact_torque_scale = _value_or_default(
            kwargs.get("contact_torque_scale", None), 5.0e-3
        )
        cfg.env.agent.contact_force_threshold = _value_or_default(
            kwargs.get("contact_force_threshold", None), 1.0
        )
        cfg.env.agent.show_contact_forces = bool(kwargs.get("show_contact_forces", False))
        cfg.env.agent.show_contact_wrench = bool(kwargs.get("show_contact_wrench", False))
        cfg.env.agent.enable_hand_control = _value_or_default(
            kwargs.get("enable_hand_control", None),
            "WithHand" in cfg.robot.cfg.class_name,
        )
        if backend == "isaac":
            # One and many Isaac environments share WBT's exact plant and
            # native-drive contract. Only tensor shape, scene layout, device,
            # and the scalar NumPy boundary differ for interactive use.
            # SPORT has its own stable tensor plant contract and the benchmark
            # intentionally uses the backend defaults. Do not silently apply
            # WBT's native implicit gains to the scalar SPORT adapter.
            if kwargs.get("composed_policy_class") != "UnitreeG1SportSafePolicy":
                from spark_policy.control.whole_body.unitree_g1.wbt import (
                    wbt_isaac_actuation_config,
                )

                for name, value in wbt_isaac_actuation_config().items():
                    setattr(cfg.env.agent, name, value)
            if kwargs.get("isaac_usd_path") is not None:
                cfg.env.agent.usd_path = kwargs["isaac_usd_path"]
            cfg.env.agent.num_envs = 1
            cfg.env.agent.scalar_api = True
        cfg.env.agent.dt = _value_or_default(kwargs.get("dt", None), timing.physics_dt)
        cfg.env.agent.control_decimation = _value_or_default(
            kwargs.get("control_decimation", None), timing.control_decimation
        )
        cfg.env.agent.viewer_hz = _value_or_default(
            kwargs.get("viewer_hz", None),
            ISAAC_INTERACTIVE_VIEWER_HZ,
        )
        _set_if_not_none(cfg.env.agent, "enable_camera", kwargs.get("enable_camera", None))
        _set_if_not_none(cfg.env.agent, "camera_width", kwargs.get("camera_width", None))
        _set_if_not_none(cfg.env.agent, "camera_height", kwargs.get("camera_height", None))
        _set_if_not_none(cfg.env.agent, "camera_rate_hz", kwargs.get("camera_rate_hz", None))
        _set_if_not_none(cfg.env.agent, "camera_display", kwargs.get("camera_display", None))
        return cfg

    cfg.env.agent.class_name = kwargs.get("agent_cfg") or "UnitreeG1RealAgent"
    cfg.env.agent.enable_viewer = False
    cfg.env.agent.enable_keyboard_control = _value_or_default(
        kwargs.get("enable_keyboard_control", None), True
    )
    cfg.env.agent.dt = _value_or_default(kwargs.get("dt", None), 0.02)
    cfg.env.agent.control_decimation = _value_or_default(kwargs.get("control_decimation", None), 1)
    cfg.env.agent.send_cmd = _value_or_default(kwargs.get("send_cmd", None), False)
    cfg.env.agent.net = kwargs.get("net", "")
    cfg.env.agent.unitree_model = _value_or_default(kwargs.get("unitree_model", None), "g1")
    cfg.env.agent.level = _value_or_default(kwargs.get("level", None), "low")
    cfg.env.agent.enable_hand_control = _value_or_default(
        kwargs.get("enable_hand_control", None), False
    )
    cfg.env.agent.enable_digital_twin = bool(kwargs.get("enable_digital_twin", True))
    cfg.env.agent.digital_twin_backend = kwargs.get("digital_twin_backend", "mujoco")
    cfg.env.agent.digital_twin_sync_hz = float(kwargs.get("digital_twin_sync_hz", 500.0))
    cfg.env.agent.hand_state_debug_print = _value_or_default(
        kwargs.get("hand_state_debug_print", None), False
    )
    cfg.env.agent.hand_state_zero_invalid = _value_or_default(
        kwargs.get("hand_state_zero_invalid", None), True
    )
    cfg.env.agent.real_robot_location_topic = _value_or_default(
        kwargs.get("real_robot_location_topic", None),
        "/real_robot_location",
    )
    cfg.env.agent.real_robot_location_enabled = _value_or_default(
        kwargs.get("real_robot_location_enabled", None),
        bool(kwargs.get("enable_ros", False)),
    )
    cfg.env.agent.real_robot_location_timeout = _value_or_default(
        kwargs.get("real_robot_location_timeout", None), 0.5
    )
    cfg.env.agent.real_robot_location_orientation_source = _value_or_default(
        kwargs.get("real_robot_location_orientation_source", None),
        "topic",
    )
    cfg.env.agent.real_robot_location_align_imu_to_pose = _value_or_default(
        kwargs.get("real_robot_location_align_imu_to_pose", None),
        False,
    )
    cfg.env.agent.real_robot_location_filter_alpha = _value_or_default(
        kwargs.get("real_robot_location_filter_alpha", None),
        1.0,
    )
    if kwargs.get("dt", None) is not None or kwargs.get("control_decimation", None) is not None:
        period = float(cfg.env.agent.dt) * int(cfg.env.agent.control_decimation)
        if kwargs.get("task_dt", None) is None:
            cfg.env.task.dt = period
        if hasattr(cfg.policy, "pre_safe_target_dt"):
            cfg.policy.pre_safe_target_dt = period
        if hasattr(cfg.policy, "executor") and hasattr(cfg.policy.executor, "safe_target_dt"):
            cfg.policy.executor.safe_target_dt = period
    return cfg


def config_policy_module(cfg, **kwargs):
    _set_if_not_none(cfg.policy, "goal_tracking_type", kwargs.get("goal_tracking_type", None))
    _set_if_not_none(cfg.policy, "class_name", kwargs.get("composed_policy_class", None))
    _set_if_not_none(cfg.policy.executor, "class_name", kwargs.get("executor_policy_class", None))
    cfg = _configure_wbt_arm_kinematics(cfg, **kwargs)
    _set_if_not_none(
        cfg.policy.goal_tracking_policy, "class_name", kwargs.get("goal_tracking_policy", None)
    )
    for name in (
        "max_planar_speed",
        "min_planar_speed",
        "max_yaw_rate",
        "max_acceleration",
        "max_yaw_acceleration",
    ):
        _set_if_not_none(cfg.policy.goal_tracking_policy, name, kwargs.get(name, None))
    if hasattr(cfg.policy, "pid_goal_tracking_policy"):
        for name in (
            "base_goal_xy_deadband",
            "base_goal_resume_distance",
            "base_goal_yaw_deadband",
            "base_goal_yaw_resume_distance",
        ):
            _set_if_not_none(
                cfg.policy.pid_goal_tracking_policy,
                name,
                kwargs.get(name, None),
            )
    if hasattr(cfg.policy, "executor"):
        for name in (
            "control_dt",
            "zero_squat_command",
            "use_policy_motor_gains",
            "use_wbt_motor_gains",
            "motor_kp_scale",
            "motor_kd_scale",
            "disable_upper_body_target_rate_limit",
            "upper_body_target_filter_gain",
            "upper_body_gravity_compensation",
            "upper_body_target_rate_limit",
            "hold_upper_body_during_locomotion",
            "base_position_kp",
            "base_orientation_kp",
            "base_velocity_kd",
            "base_arrival_velocity",
            "base_settle_steps",
            "hold_loco_stance_after_motion",
            "base_xy_yaw_limit",
            "base_z_pitch_limit",
            "base_command_rate_limit",
            "base_command_deadband",
            "squat_command_scale",
            "loco_command_scale",
            "loco_start_threshold",
            "safe_loco_start_threshold",
            "stop_threshold",
            "base_goal_xy_deadband",
            "base_goal_resume_distance",
            "base_goal_yaw_deadband",
            "base_goal_yaw_resume_distance",
        ):
            _set_if_not_none(cfg.policy.executor, name, kwargs.get(name, None))
        if (
            kwargs.get("backend", "mujoco") == "isaac"
            and kwargs.get("upper_body_gravity_compensation", None) is None
        ):
            # Generalized gravity-force ordering is asset-internal for nested
            # USD articulations.  The hybrid implicit drive supplies stable
            # arm holding without injecting a potentially mismapped vector.
            cfg.policy.executor.upper_body_gravity_compensation = False
        # Backend-specific motor/contact parameters belong to the agent.  The
        # WBT command map and arrival thresholds deliberately remain identical
        # so a given policy command has the same meaning in both simulators.
    for name in (
        "pre_safe_height_recovery_enabled",
        "pre_safe_height_default",
        "pre_safe_height_recovery_kp",
        "pre_safe_height_recovery_deadband",
        "pre_safe_height_command_limit",
        "pre_safe_height_target_lookahead",
        "pre_safe_height_min",
        "pre_safe_height_max",
        "pre_safe_height_priority",
        "pre_safe_height_priority_deadband",
        "pre_safe_height_priority_xy_deadband",
        "pre_safe_height_priority_yaw_deadband",
        "pre_safe_loco_command_limit",
        "pre_safe_loco_command_rate_limit",
        "pre_safe_upper_body_velocity_limit",
        "pre_safe_upper_body_position_error_limit",
    ):
        _set_if_not_none(cfg.policy, name, kwargs.get(name, None))
    return cfg


def config_safety_module(cfg, **kwargs):
    safe_algo = kwargs.get("safe_algo", "bypass")
    _configure_safe_algo(cfg.policy.pre_safe_controller.safe_algo, safe_algo, **kwargs)
    control_weight = _control_weight(kwargs.get("control_weight", None))
    if (
        control_weight is not None
        and cfg.policy.pre_safe_controller.safe_algo.class_name != "ByPassSafeControl"
    ):
        cfg.policy.pre_safe_controller.safe_algo.control_weight = control_weight

    safety_index = kwargs.get("safety_index", "si1")
    _configure_safety_index(
        cfg.policy.safe_controller.safety_index,
        safety_index,
        **kwargs,
    )
    if kwargs.get("environment_representation", "sphere") != "sphere":
        nearest = int(kwargs.get("safety_nearest_points_per_link", 2))
        cfg.policy.safe_controller.safety_index.nearest_points_per_link = nearest
        cfg.policy.pre_safe_controller.safety_index.nearest_points_per_link = nearest
    _configure_safety_index(
        cfg.policy.pre_safe_controller.safety_index,
        safety_index,
        **kwargs,
    )
    cfg.metric_selection.trigger_safe_controller = (
        cfg.policy.pre_safe_controller.safe_algo.class_name != "ByPassSafeControl"
    )
    return cfg


def config_pipeline(cfg, **kwargs):
    _set_if_not_none(cfg.robot.cfg, "class_name", kwargs.get("robot_cfg", None))
    robot_cfg_name = kwargs.get("robot_cfg", cfg.robot.cfg.class_name)
    if hasattr(cfg.policy, "safe_robot_cfg"):
        dynamics = "Dynamic2" if "Dynamic2" in robot_cfg_name else "Dynamic1"
        cfg.policy.safe_robot_cfg.class_name = f"UnitreeG1MobileBase{dynamics}Config"
    cfg.robot.kinematics.load_collision_geometry = bool(
        kwargs.get("load_collision_geometry", False)
    )
    cfg.enable_logger = _value_or_default(kwargs.get("enable_logger", None), False)
    cfg.enable_plotter = _value_or_default(kwargs.get("enable_plotter", None), False)
    cfg.metric_selection.dof_pos = True
    cfg.metric_selection.dof_vel = True
    cfg.metric_selection.dist_goal_base = False
    cfg.metric_selection.trigger_safe_controller = kwargs.get("safe_algo", "bypass") != "bypass"
    cfg.profile_frequency = bool(kwargs.get("profile_frequency", False))
    cfg.continuous_goal_transition = bool(kwargs.get("continuous_goal_transition", False))
    cfg.render_robot_collision_volumes = _value_or_default(
        kwargs.get("render_robot_collision_volumes", None), True
    )
    # Keep the link-mounted Isaac RGB-D observation free of visualization-only
    # safety volumes, goals, frames, and constraint regions.
    if kwargs.get("backend", "mujoco") == "isaac" and kwargs.get("enable_camera", False):
        cfg.render_robot_collision_volumes = False
        cfg.render_goal_frames = False
        cfg.render_task_debug_frames = False
        cfg.render_action_debug_frames = False
        cfg.render_robot_reference_frames = False
        cfg.enable_safe_zone_render = False
    _set_if_not_none(cfg, "max_num_steps", kwargs.get("max_num_steps", None))
    _set_if_not_none(cfg, "max_num_reset", kwargs.get("max_num_reset", None))
    if (
        kwargs.get("max_num_steps", None) is not None
        and hasattr(cfg.env.task, "max_episode_length")
        and getattr(cfg.env.task, "class_name", None) != "BenchmarkTask"
    ):
        cfg.env.task.max_episode_length = kwargs["max_num_steps"] + 1
    return cfg


def build_config(**kwargs):
    """Build one WBT configuration whose policy/task are backend-neutral."""
    kwargs = _with_run_defaults(kwargs)
    robot_cfg = kwargs["robot_cfg"]
    cfg = _cfg_class(robot_cfg)()
    cfg = config_pipeline(cfg, **kwargs)
    cfg = config_task_module(cfg, **kwargs)
    cfg = config_agent_module(cfg, **kwargs)
    cfg = config_policy_module(cfg, **kwargs)
    cfg = config_safety_module(cfg, **kwargs)
    physics_period = float(cfg.env.agent.dt) * int(cfg.env.agent.control_decimation)
    task_period = float(cfg.env.task.dt)
    if not np.isclose(physics_period, task_period, rtol=0.0, atol=1.0e-9):
        raise ValueError(
            "Unitree control clocks must match: "
            f"physics dt*decimation={physics_period:.9f}s, task dt={task_period:.9f}s"
        )
    # Policy timing belongs to the executor configuration. The agent exposes
    # only its plant integration clock; this bridge makes their contract
    # explicit and lets each policy reject an incompatible checkpoint clock.
    cfg.policy.executor.control_dt = physics_period
    cfg.policy.pre_safe_target_dt = physics_period
    if hasattr(cfg.policy, "goal_tracking_policy"):
        cfg.policy.goal_tracking_policy.tracking_dt = physics_period
    return cfg


def run(**kwargs):
    kwargs = _with_run_defaults(kwargs)
    cfg = build_config(**kwargs)

    return run_configured_pipeline(cfg, kwargs)


def run_configured_pipeline(
    cfg,
    kwargs,
    *,
    pipeline_class=TeleopPipeline,
    error_label="WBT",
    before_pipeline=None,
    after_pipeline=None,
):
    """Execute a configured whole-body policy with shared MuJoCo/Isaac lifecycle."""

    backend = kwargs.get("backend", "mujoco")
    configure_simulation_presentation(cfg, backend=backend)
    simulation_app = None
    pipeline = None
    try:
        if backend == "isaac":
            enable_viewer = bool(cfg.env.agent.enable_viewer)
            enable_camera = bool(getattr(cfg.env.agent, "enable_camera", False))
            from isaacsim import SimulationApp
            from spark_agent.simulation.isaac.render_quality import isaac_render_preset

            render_config = isaac_render_preset(
                kwargs.get("isaac_render_quality", "performance"),
                renderer=kwargs.get("isaac_renderer"),
                anti_aliasing=kwargs.get("isaac_anti_aliasing"),
                width=kwargs.get("viewer_width"),
                height=kwargs.get("viewer_height"),
            )

            simulation_app = SimulationApp(
                {
                    "headless": not enable_viewer,
                    "hide_ui": not enable_viewer,
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
                    "disable_viewport_updates": not (enable_viewer or enable_camera),
                    "extra_args": [
                        "--/exts/omni.kit.hotkeys.core/hotkeys_enabled=false",
                        "--/app/viewport/defaults/noTitleBar=true",
                        "--/persistent/app/viewport/noPadding=true",
                        "--/exts/omni.kit.viewport.window/startup/dockTabInvisible=true",
                    ],
                }
            )
            # The authoritative Isaac application owns the visible viewport
            # and receives the exact state used by the policy loop.
            cfg.env.agent.render = enable_viewer or enable_camera

        # Preserve the original single-viewer lifecycle: construct the Isaac
        # agent (and therefore its stage, articulation, ground, and camera)
        # immediately after SimulationApp.  A policy backend may take minutes
        # to initialize, but must not delay creation of the visible robot.
        pipeline = pipeline_class(cfg)
        if simulation_app is not None:
            pipeline.env.agent.attach_simulation_app(simulation_app)
        if before_pipeline is not None:
            if backend == "isaac" and enable_viewer:
                pipeline.env.agent.render()
                simulation_app.update()
                print(
                    "[SPARK] Isaac robot ready; initializing policy backend...",
                    flush=True,
                )
            before_pipeline(pipeline)
        _apply_base_goal_pitch(pipeline, **kwargs)
        pipeline.run(save_path=kwargs.get("save_path", None))
    except Exception as exc:
        print(f"{error_label} pipeline failed before shutdown: {type(exc).__name__}: {exc}")
        traceback.print_exc()
        raise
    finally:
        # External policy backends must stop before SimulationApp.close().
        # Isaac may terminate its Python host as part of Kit shutdown, which
        # would otherwise skip an outer caller's finally block and orphan the
        # backend process (and its inherited stdout pipe).
        if after_pipeline is not None:
            after_pipeline()
        if pipeline is not None:
            pipeline.env.agent.close_viewer()
            if backend == "isaac":
                pipeline.env.agent.close()
        if simulation_app is not None:
            simulation_app.close(wait_for_replicator=False)
    return pipeline


def _apply_base_goal_pitch(pipeline, **kwargs):
    base_goal_pitch = kwargs.get("base_goal_pitch", 0.0)
    if abs(base_goal_pitch) > 0.0:
        pipeline.env.agent.base_goal_debug_frame[:3, :3] = R.from_euler(
            "xyz",
            [0.0, base_goal_pitch, 0.0],
        ).as_matrix()
