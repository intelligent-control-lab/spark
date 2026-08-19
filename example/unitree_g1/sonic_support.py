import argparse
import os
import shlex
import socket
import subprocess
import time
from pathlib import Path
from urllib.parse import urlparse

import numpy as np
from scipy.spatial.transform import Rotation as R

from spark_pipeline import (
    TeleopPipeline as Pipeline,
    UNITREE_G1_SONIC_PRE_SAFE_CONTROL_NAMES,
    UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig,
    UnitreeG1SonicCascadeWholeBodyWithHandTeleopPipelineConfig,
)
from spark_pipeline.teleop.camera import add_teleop_viewer_arguments
from spark_policy.composed_policy.unitree_g1.sonic_safe import (
    UnitreeG1SonicSafePolicyConfig,
)


USE_REAL = False

SONIC_WAIST_CONTROL_WEIGHT = 25.0
SONIC_UPPER_BODY_CONTROL_WEIGHT = 1.0
# Lower cost means the safety QP prefers this control group.  Collision
# avoidance should first modify the walking path, then the constrained arm,
# and use waist motion only when its geometry actually requires it.
SONIC_LOCOMOTION_CONTROL_WEIGHT = 1.0
SONIC_HEIGHT_CONTROL_WEIGHT = 0.1


def _default_sonic_control_weight(
    waist=SONIC_WAIST_CONTROL_WEIGHT,
    upper_body=SONIC_UPPER_BODY_CONTROL_WEIGHT,
    locomotion=SONIC_LOCOMOTION_CONTROL_WEIGHT,
    height=SONIC_HEIGHT_CONTROL_WEIGHT,
):
    # Pre-safe SONIC controls are 17 upper-body joint velocities plus virtual
    # base x/y/yaw commands; configurations with height control add z last.
    expected = len(UNITREE_G1_SONIC_PRE_SAFE_CONTROL_NAMES)
    weight = [float(waist)] * 3 + [float(upper_body)] * 14 + [float(locomotion)] * 3
    if expected == 21:
        weight += [float(height)]
    if len(weight) != expected:
        raise ValueError(
            "Unexpected SONIC pre-safe control layout: "
            f"expected {expected} controls, grouped weights define {len(weight)}"
        )
    return weight


SONIC_CONTROL_WEIGHT = _default_sonic_control_weight()

SONIC_BACKEND_BINARY_REL = Path("target/release/g1_deploy_onnx_ref")
SONIC_POLICY_FILE_REL = Path("policy/release/model_decoder.onnx")
SONIC_ENCODER_FILE_REL = Path("policy/release/model_encoder.onnx")
SONIC_OBS_CONFIG_REL = Path("policy/release/observation_config.yaml")
SONIC_PLANNER_FILE_REL = Path("planner/target_vel/V2/planner_sonic.onnx")
SONIC_MOTION_DATA_REL = Path("reference/example")


def _with_sonic_runtime_defaults(kwargs):
    """Fill policy-owned server defaults without overriding explicit input."""
    resolved = dict(kwargs)
    defaults = UnitreeG1SonicSafePolicyConfig()
    values = {
        "goal_tracking_type": defaults.goal_tracking_type,
        "auto_launch_sonic_server": defaults.auto_launch_server,
        "sonic_policy_precision": defaults.policy_precision,
        "sonic_planner_precision": defaults.planner_precision,
        "sonic_server_startup_timeout": defaults.server_startup_timeout,
    }
    for name, value in values.items():
        if resolved.get(name) is None:
            resolved[name] = value
    return resolved


def _weight_or_default(kwargs, name, default):
    value = kwargs.get(name, default)
    return default if value is None else value


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


def _pipeline_config_class(robot_cfg):
    if robot_cfg is not None and "WithHand" in robot_cfg:
        return UnitreeG1SonicCascadeWholeBodyWithHandTeleopPipelineConfig
    return UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig


def _control_period(cfg):
    return float(cfg.env.agent.dt) * int(cfg.env.agent.control_decimation)


def _normalize_min_distance(value):
    if value is None:
        return None
    if isinstance(value, dict):
        return value
    return {"environment": float(value), "self": 0.01}


def _control_weight(value):
    if value is None:
        return None

    weight = np.asarray(value, dtype=float).reshape(-1)
    expected = len(UNITREE_G1_SONIC_PRE_SAFE_CONTROL_NAMES)
    if weight.shape[0] != expected:
        raise ValueError(f"control_weight must have {expected} entries, got {weight.shape[0]}")
    return weight.tolist()


def _active_safety_controller_cfg(cfg):
    return getattr(cfg.policy, "pre_safe_controller", cfg.policy.safe_controller)


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


def config_task_module(cfg, **kwargs):
    _set_if_not_none(cfg.env.task, "class_name", kwargs.get("task_class_name", None))
    _set_if_not_none(cfg.env.task, "task_name", kwargs.get("task_name", None))
    _set_if_not_none(cfg.env.task, "enable_ros", kwargs.get("enable_ros", None))
    _set_if_not_none(cfg.env.task, "ros_version", kwargs.get("ros_version", None))
    _set_if_not_none(cfg.env.task, "dt", kwargs.get("task_dt", None))
    _set_if_not_none(cfg.env.task, "num_obstacle_task", kwargs.get("num_obstacle_task", None))
    _set_if_not_none(cfg.env.task, "obstacle_init", kwargs.get("obstacle_init", None))
    _set_if_not_none(cfg.env.task, "obstacle_velocity", kwargs.get("obstacle_velocity", None))
    _set_if_not_none(cfg.env.task, "obstacle_direction", kwargs.get("obstacle_direction", None))
    _set_if_not_none(cfg.env.task, "arm_goal_enable", kwargs.get("arm_goal_enable", None))
    _set_if_not_none(cfg.env.task, "base_goal_enable", kwargs.get("base_goal_enable", None))
    _set_if_not_none(cfg.env.task, "base_goal_init", kwargs.get("base_goal_init", None))
    _set_if_not_none(cfg.env.task, "base_goal_velocity", kwargs.get("base_goal_velocity", None))
    _set_if_not_none(cfg.env.task, "goal_left_init", kwargs.get("goal_left_init", None))
    _set_if_not_none(cfg.env.task, "goal_right_init", kwargs.get("goal_right_init", None))
    _set_if_not_none(cfg.env.task, "goal_left_velocity", kwargs.get("goal_left_velocity", None))
    _set_if_not_none(cfg.env.task, "goal_right_velocity", kwargs.get("goal_right_velocity", None))
    _set_if_not_none(
        cfg.env.task,
        "arm_goal_init_from_current_ee",
        kwargs.get("arm_goal_init_from_current_ee", None),
    )
    # Benchmark cases are backend/robot independent. Ground their task and
    # reset fields here exactly as the WBT runner does.
    for name in (
        "mode",
        "obstacle_mode",
        "arm_goal_mode",
        "base_goal_mode",
        "obstacle_range",
        "obstacle_size",
        "obstacle_keepout",
        "robot_keepout",
        "left_arm_goal_range",
        "right_arm_goal_range",
        "base_goal_range",
        "base_goal_minimum_distance",
        "base_goal_relative_to_current",
        "base_goal_workspace_range",
        "base_goal_rot_range",
        "arm_goal_size",
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
    ):
        value = kwargs.get(name, None)
        if name == "obstacle_range" and value is not None:
            flat = np.asarray(value, dtype=float).reshape(-1)
            if flat.shape[0] == 6:
                value = [tuple(flat[index : index + 2]) for index in range(0, 6, 2)]
        _set_if_not_none(cfg.env.task, name, value)
    return cfg


def config_agent_module(cfg, **kwargs):
    use_real = kwargs.get("use_real", USE_REAL)
    backend = kwargs.get("backend", "mujoco")
    cfg.env.agent.viewer_show_simulation_info = bool(
        kwargs.get("viewer_show_simulation_info", False)
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
    else:
        # SONIC and WBT both emit whole-body joint targets, so they must use
        # the same simulator agent, timing, viewer, camera, and actuator
        # configuration. Policy selection must not alter backend semantics.
        from spark_pipeline.teleop.unitree_g1_whole_body_config import (
            config_agent_module as configure_whole_body_agent,
        )

        cfg = configure_whole_body_agent(cfg, **kwargs)
        if backend == "isaac":
            # Keep asset selection policy-agnostic. The selected robot config
            # owns the Isaac URDF, including articulated hands and materials.
            cfg.env.agent.usd_path = kwargs.get("isaac_usd_path", None)
            # Apply the plant contract owned by the SONIC policy package.
            from spark_policy.control.whole_body.unitree_g1.sonic.config import (
                sonic_isaac_actuation_config,
            )

            for name, value in sonic_isaac_actuation_config().items():
                setattr(cfg.env.agent, name, value)
            cfg.env.agent.num_envs = 1
            cfg.env.agent.scalar_api = True
            cfg.env.agent.sim_position_error_limit = _value_or_default(
                kwargs.get("sim_position_error_limit", None), None
            )

    if kwargs.get("num_obstacle_debug", None) is not None:
        cfg.env.agent.obstacle_debug["num_obstacle"] = kwargs["num_obstacle_debug"]
    if kwargs.get("manual_movement_step_size", None) is not None:
        cfg.env.agent.obstacle_debug["manual_movement_step_size"] = kwargs[
            "manual_movement_step_size"
        ]

    if (
        use_real
        or kwargs.get("dt", None) is not None
        or kwargs.get("control_decimation", None) is not None
    ):
        period = _control_period(cfg)
        if kwargs.get("task_dt", None) is None:
            cfg.env.task.dt = period
        cfg.policy.executor.safe_target_dt = period
        cfg.policy.executor.control_dt = period
    return cfg


def config_policy_module(cfg, **kwargs):
    _set_if_not_none(cfg.policy, "goal_tracking_type", kwargs.get("goal_tracking_type", None))
    _set_if_not_none(cfg.policy.executor, "sonic_endpoint", kwargs.get("sonic_endpoint", None))
    _set_if_not_none(cfg.policy.executor, "sonic_timeout_ms", kwargs.get("sonic_timeout_ms", None))
    _set_if_not_none(
        cfg.policy.executor, "sonic_fallback_mode", kwargs.get("sonic_fallback_mode", None)
    )
    _set_if_not_none(
        cfg.policy.executor,
        "sonic_planar_speed_scale",
        kwargs.get("sonic_planar_speed_scale", None),
    )
    _set_if_not_none(
        cfg.policy.executor, "sonic_slow_min_speed", kwargs.get("sonic_slow_min_speed", None)
    )
    _set_if_not_none(
        cfg.policy.executor,
        "sonic_slow_walk_max_speed",
        kwargs.get("sonic_slow_walk_max_speed", None),
    )
    _set_if_not_none(
        cfg.policy.executor, "sonic_walk_min_speed", kwargs.get("sonic_walk_min_speed", None)
    )
    _set_if_not_none(
        cfg.policy.executor, "sonic_walk_max_speed", kwargs.get("sonic_walk_max_speed", None)
    )
    _set_if_not_none(
        cfg.policy.executor, "sonic_run_min_speed", kwargs.get("sonic_run_min_speed", None)
    )
    _set_if_not_none(cfg.policy.executor, "sonic_max_speed", kwargs.get("sonic_max_speed", None))
    _set_if_not_none(
        cfg.policy.executor, "sonic_motion_deadband", kwargs.get("sonic_motion_deadband", None)
    )
    sonic_locomotion_mode = kwargs.get("sonic_locomotion_mode", None)
    if (
        sonic_locomotion_mode is None
        and str(getattr(cfg.policy, "goal_tracking_type", "legged")).lower() == "pid"
    ):
        sonic_locomotion_mode = getattr(cfg.policy, "pid_sonic_locomotion_mode", "walk")
    _set_if_not_none(cfg.policy.executor, "sonic_locomotion_mode", sonic_locomotion_mode)
    _set_if_not_none(
        cfg.policy.executor, "sonic_facing_mode", kwargs.get("sonic_facing_mode", None)
    )
    _set_if_not_none(
        cfg.policy.executor,
        "sonic_height_control_enabled",
        kwargs.get("sonic_height_control_enabled", None),
    )
    _set_if_not_none(
        cfg.policy.executor, "sonic_default_height", kwargs.get("sonic_default_height", None)
    )
    _set_if_not_none(cfg.policy.executor, "sonic_height_min", kwargs.get("sonic_height_min", None))
    _set_if_not_none(cfg.policy.executor, "sonic_height_max", kwargs.get("sonic_height_max", None))
    _set_if_not_none(
        cfg.policy.executor, "sonic_height_deadband", kwargs.get("sonic_height_deadband", None)
    )
    _set_if_not_none(
        cfg.policy.executor,
        "sonic_height_filter_gain",
        kwargs.get("sonic_height_filter_gain", None),
    )
    _set_if_not_none(
        cfg.policy.executor, "sonic_height_rate_limit", kwargs.get("sonic_height_rate_limit", None)
    )
    _set_if_not_none(cfg.policy.executor, "base_position_kp", kwargs.get("base_position_kp", None))
    _set_if_not_none(cfg.policy.executor, "base_velocity_kd", kwargs.get("base_velocity_kd", None))
    _set_if_not_none(
        cfg.policy.executor, "base_orientation_kp", kwargs.get("base_orientation_kp", None)
    )
    _set_if_not_none(
        cfg.policy.executor, "base_xy_yaw_limit", kwargs.get("base_xy_yaw_limit", None)
    )
    _set_if_not_none(
        cfg.policy.executor, "base_z_pitch_limit", kwargs.get("base_z_pitch_limit", None)
    )
    _set_if_not_none(
        cfg.policy.executor,
        "base_planar_velocity_limit",
        kwargs.get("base_planar_velocity_limit", None),
    )
    _set_if_not_none(
        cfg.policy.executor, "base_goal_xy_deadband", kwargs.get("base_goal_xy_deadband", None)
    )
    _set_if_not_none(
        cfg.policy.executor,
        "base_goal_resume_distance",
        kwargs.get("base_goal_resume_distance", None),
    )
    _set_if_not_none(
        cfg.policy.executor, "base_goal_yaw_deadband", kwargs.get("base_goal_yaw_deadband", None)
    )
    _set_if_not_none(
        cfg.policy.executor,
        "base_goal_yaw_resume_distance",
        kwargs.get("base_goal_yaw_resume_distance", None),
    )
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
    _set_if_not_none(
        cfg.policy.executor,
        "base_goal_slowdown_enabled",
        kwargs.get("base_goal_slowdown_enabled", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "base_goal_slowdown_distance",
        kwargs.get("base_goal_slowdown_distance", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "base_goal_slowdown_min_speed",
        kwargs.get("base_goal_slowdown_min_speed", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "base_stop_lookahead_time",
        kwargs.get("base_stop_lookahead_time", None),
    )
    _set_if_not_none(
        cfg.policy.executor, "base_stop_distance_bias", kwargs.get("base_stop_distance_bias", None)
    )
    _set_if_not_none(
        cfg.policy.executor, "base_command_rate_limit", kwargs.get("base_command_rate_limit", None)
    )
    _set_if_not_none(
        cfg.policy.executor, "base_command_deadband", kwargs.get("base_command_deadband", None)
    )
    _set_if_not_none(
        cfg.policy.executor,
        "base_disabled_identity_facing",
        kwargs.get("base_disabled_identity_facing", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "base_disabled_zero_waist_upper_body_input",
        kwargs.get("base_disabled_zero_waist_upper_body_input", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "apply_safe_control_targets",
        kwargs.get("apply_safe_control_targets", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "safe_control_only_when_triggered",
        kwargs.get("safe_control_only_when_triggered", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "post_sonic_upper_body_overwrite",
        kwargs.get("post_sonic_upper_body_overwrite", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "override_sonic_upper_body_target",
        kwargs.get("override_sonic_upper_body_target", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "use_fixed_base_arm_kinematics",
        kwargs.get("use_fixed_base_arm_kinematics", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "disable_upper_body_target_rate_limit",
        kwargs.get("disable_upper_body_target_rate_limit", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "override_sonic_upper_body_motor_gains",
        kwargs.get("override_sonic_upper_body_motor_gains", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "use_wbt_motor_gains",
        kwargs.get("use_wbt_motor_gains", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "disable_upper_body_motor_gain_override_when_base_goal_disabled",
        kwargs.get("disable_upper_body_motor_gain_override_when_base_goal_disabled", None),
    )
    _set_if_not_none(
        cfg.policy.executor, "upper_body_motor_kps", kwargs.get("upper_body_motor_kps", None)
    )
    _set_if_not_none(
        cfg.policy.executor, "upper_body_motor_kds", kwargs.get("upper_body_motor_kds", None)
    )
    _set_if_not_none(
        cfg.policy.executor,
        "upper_body_target_filter_gain",
        kwargs.get("upper_body_target_filter_gain", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "upper_body_gravity_compensation",
        kwargs.get("upper_body_gravity_compensation", None),
    )
    _set_if_not_none(
        cfg.policy.executor,
        "upper_body_target_rate_limit",
        kwargs.get("upper_body_target_rate_limit", None),
    )
    return cfg


def config_safety_module(cfg, **kwargs):
    safety_controller_cfg = _active_safety_controller_cfg(cfg)
    for name in (
        "pre_safe_height_command_limit",
        "pre_safe_height_target_lookahead",
        "pre_safe_height_min",
        "pre_safe_height_max",
        "pre_safe_height_recovery_enabled",
        "pre_safe_height_default",
        "pre_safe_height_recovery_kp",
        "pre_safe_height_recovery_deadband",
    ):
        _set_if_not_none(cfg.policy, name, kwargs.get(name, None))

    safe_algo = kwargs.get("safe_algo", None)
    if safe_algo is not None:
        _configure_safe_algo(safety_controller_cfg.safe_algo, safe_algo, **kwargs)

    control_weight = _control_weight(kwargs.get("control_weight", None))
    if control_weight is None:
        control_weight = _default_sonic_control_weight(
            waist=_weight_or_default(kwargs, "waist_control_weight", SONIC_WAIST_CONTROL_WEIGHT),
            upper_body=_weight_or_default(
                kwargs,
                "upper_body_control_weight",
                SONIC_UPPER_BODY_CONTROL_WEIGHT,
            ),
            locomotion=_weight_or_default(
                kwargs,
                "locomotion_control_weight",
                SONIC_LOCOMOTION_CONTROL_WEIGHT,
            ),
            height=_weight_or_default(kwargs, "height_control_weight", SONIC_HEIGHT_CONTROL_WEIGHT),
        )
    if safety_controller_cfg.safe_algo.class_name != "ByPassSafeControl":
        safety_controller_cfg.safe_algo.control_weight = control_weight

    safety_index = kwargs.get("safety_index", None)
    if safety_index is not None:
        _configure_safety_index(safety_controller_cfg.safety_index, safety_index, **kwargs)

    min_distance = _normalize_min_distance(kwargs.get("min_distance", None))
    if min_distance is not None:
        safety_controller_cfg.safety_index.min_distance = min_distance

    enable_self_collision = kwargs.get("enable_self_collision", None)
    if enable_self_collision is not None:
        safety_controller_cfg.safety_index.enable_self_collision = bool(enable_self_collision)

    safe_enabled = safety_controller_cfg.safe_algo.class_name != "ByPassSafeControl"
    if hasattr(cfg.policy, "pre_safe_enabled"):
        cfg.policy.pre_safe_enabled = safe_enabled
        cfg.policy.executor.apply_safe_control_targets = False
    elif safe_enabled and kwargs.get("apply_safe_control_targets", None) is None:
        cfg.policy.executor.apply_safe_control_targets = True

    cfg.metric_selection.trigger_safe_controller = safe_enabled
    return cfg


def config_pipeline(cfg, **kwargs):
    _set_if_not_none(cfg.robot.cfg, "class_name", kwargs.get("robot_cfg", None))
    _set_if_not_none(cfg, "max_num_steps", kwargs.get("max_num_steps", None))
    _set_if_not_none(cfg, "max_num_reset", kwargs.get("max_num_reset", None))
    _set_if_not_none(cfg, "enable_logger", kwargs.get("enable_logger", None))
    _set_if_not_none(cfg, "enable_plotter", kwargs.get("enable_plotter", None))
    cfg.profile_frequency = bool(kwargs.get("profile_frequency", False))
    _set_if_not_none(
        cfg,
        "render_robot_collision_volumes",
        kwargs.get("render_robot_collision_volumes", None),
    )
    return cfg


def _apply_base_goal_pitch(pipeline, **kwargs):
    base_goal_pitch = kwargs.get("base_goal_pitch", 0.0)
    if abs(base_goal_pitch) > 0.0:
        pipeline.env.agent.base_goal_debug_frame[:3, :3] = R.from_euler(
            "xyz",
            [0.0, base_goal_pitch, 0.0],
        ).as_matrix()


def _endpoint_tcp_target(endpoint):
    parsed = urlparse(str(endpoint))
    if parsed.scheme != "tcp" or not parsed.hostname or not parsed.port:
        return None
    host = parsed.hostname
    if host in ("*", "0.0.0.0"):
        host = "127.0.0.1"
    return host, int(parsed.port)


def _wait_for_sonic_endpoint(endpoint, timeout_s, process=None, progress_callback=None):
    target = _endpoint_tcp_target(endpoint)
    if target is None or timeout_s is None or timeout_s <= 0.0:
        return

    host, port = target
    deadline = time.monotonic() + float(timeout_s)
    last_error = None
    while time.monotonic() < deadline:
        if progress_callback is not None:
            progress_callback()
        if process is not None and process.poll() is not None:
            raise RuntimeError(
                f"SONIC backend exited before policy server became ready "
                f"(return code {process.returncode})"
            )
        try:
            with socket.create_connection((host, port), timeout=0.25):
                return
        except OSError as exc:
            last_error = exc
            time.sleep(0.25)
    raise TimeoutError(f"Timed out waiting for SONIC policy server at {endpoint}: {last_error}")


def _resolve_optional_root(value):
    if value is None or str(value).strip() == "":
        configured = os.environ.get("SPARK_SONIC_DEPLOY_ROOT")
        if configured:
            value = configured
        else:
            candidate = (
                Path(__file__).resolve().parents[3]
                / "sonic"
                / "GR00T-WholeBodyControl"
                / "gear_sonic_deploy"
            )
            if candidate.is_dir():
                value = candidate
            else:
                return None
    root = Path(value).expanduser()
    return root if root.is_absolute() else root.resolve()


def _resolve_path(value, root=None):
    path = Path(value).expanduser()
    if not path.is_absolute() and root is not None:
        path = Path(root) / path
    return path


def _resolve_sonic_asset(kwargs, name, relative_default, deploy_root):
    value = kwargs.get(name, None)
    if value is None or str(value).strip() == "":
        if deploy_root is None:
            option = "--" + name.replace("_", "-")
            raise ValueError(
                "SONIC deployment assets were not found. Set "
                "SPARK_SONIC_DEPLOY_ROOT, pass --sonic-deploy-root, or provide "
                f"{option} explicitly."
            )
        return deploy_root / relative_default
    return _resolve_path(value, deploy_root)


def _build_sonic_backend_command(**kwargs):
    kwargs = _with_sonic_runtime_defaults(kwargs)
    deploy_root = _resolve_optional_root(kwargs.get("sonic_deploy_root", None))
    binary = _resolve_sonic_asset(
        kwargs, "sonic_backend_binary", SONIC_BACKEND_BINARY_REL, deploy_root
    )
    policy_file = _resolve_sonic_asset(
        kwargs, "sonic_policy_file", SONIC_POLICY_FILE_REL, deploy_root
    )
    encoder_file = _resolve_sonic_asset(
        kwargs, "sonic_encoder_file", SONIC_ENCODER_FILE_REL, deploy_root
    )
    obs_config = _resolve_sonic_asset(kwargs, "sonic_obs_config", SONIC_OBS_CONFIG_REL, deploy_root)
    planner_file = _resolve_sonic_asset(
        kwargs, "sonic_planner_file", SONIC_PLANNER_FILE_REL, deploy_root
    )
    motion_data = _resolve_sonic_asset(
        kwargs, "sonic_motion_data", SONIC_MOTION_DATA_REL, deploy_root
    )

    required_paths = {
        "sonic_backend_binary": binary,
        "sonic_policy_file": policy_file,
        "sonic_encoder_file": encoder_file,
        "sonic_obs_config": obs_config,
        "sonic_planner_file": planner_file,
        "sonic_motion_data": motion_data,
    }
    missing = [f"{name}={path}" for name, path in required_paths.items() if not path.exists()]
    if missing:
        raise FileNotFoundError("Missing SONIC backend asset(s): " + "; ".join(missing))

    endpoint = kwargs.get("sonic_endpoint", "tcp://127.0.0.1:5560")
    command = [
        str(binary),
        str(kwargs.get("sonic_backend_interface", "lo")),
        str(policy_file),
        str(motion_data),
        "--obs-config",
        str(obs_config),
        "--encoder-file",
        str(encoder_file),
        "--planner-file",
        str(planner_file),
        "--spark-policy-server",
        "--spark-policy-endpoint",
        str(endpoint),
    ]

    planner_precision = str(kwargs["sonic_planner_precision"])
    policy_precision = str(kwargs["sonic_policy_precision"])
    if planner_precision not in ("16", "32"):
        raise ValueError("sonic_planner_precision must be '16' or '32'")
    if policy_precision not in ("16", "32"):
        raise ValueError("sonic_policy_precision must be '16' or '32'")
    command += ["--planner-precision", planner_precision, "--policy-precision", policy_precision]
    batch_size = int(kwargs.get("sonic_server_batch_size", 1) or 1)
    if batch_size > 1:
        command += ["--spark-policy-batch-size", str(batch_size)]

    extra_args = kwargs.get("sonic_server_extra_args", None)
    if extra_args:
        command += shlex.split(extra_args) if isinstance(extra_args, str) else list(extra_args)
    return command, deploy_root


def _launch_sonic_backend(
    command,
    endpoint=None,
    startup_timeout_s=1.0,
    cwd=None,
    progress_callback=None,
):
    if not command:
        return None
    args = shlex.split(command) if isinstance(command, str) else list(command)
    print("[SONIC backend] launching:", " ".join(shlex.quote(str(arg)) for arg in args))
    process_env = os.environ.copy()
    tensorrt_root = Path(
        process_env.get("TensorRT_ROOT")
        or process_env.get("TENSORRT_ROOT")
        or (Path.home() / "TensorRT")
    ).expanduser()
    tensorrt_lib = tensorrt_root / "lib"
    if tensorrt_lib.is_dir():
        existing = process_env.get("LD_LIBRARY_PATH", "")
        entries = [str(tensorrt_lib)]
        if existing:
            entries.append(existing)
        process_env["LD_LIBRARY_PATH"] = os.pathsep.join(entries)
    process = subprocess.Popen(
        args,
        cwd=None if cwd is None else str(cwd),
        env=process_env,
    )
    if endpoint:
        try:
            _wait_for_sonic_endpoint(
                endpoint,
                startup_timeout_s,
                process=process,
                progress_callback=progress_callback,
            )
            print(f"[SONIC backend] policy server ready at {endpoint}")
        except Exception:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
            raise
    elif startup_timeout_s and startup_timeout_s > 0.0:
        time.sleep(float(startup_timeout_s))
    return process


def run(**kwargs):
    """Run SPARK teleop with SONIC as the whole-body backend policy."""
    kwargs = _with_sonic_runtime_defaults(kwargs)
    cfg_cls = _pipeline_config_class(kwargs.get("robot_cfg", None))
    cfg = cfg_cls()
    cfg = config_pipeline(cfg, **kwargs)
    cfg = config_task_module(cfg, **kwargs)
    cfg = config_agent_module(cfg, **kwargs)
    cfg = config_policy_module(cfg, **kwargs)
    cfg = config_safety_module(cfg, **kwargs)

    backend_command = kwargs.get("launch_sonic_server_cmd", None)
    backend_cwd = None
    if not backend_command and kwargs.get("auto_launch_sonic_server", False):
        backend_command, backend_cwd = _build_sonic_backend_command(**kwargs)

    backend = None

    def launch_backend_after_viewer(pipeline):
        nonlocal backend

        def keep_isaac_viewer_responsive():
            if kwargs.get("backend", "mujoco") == "isaac":
                pipeline.env.agent.render()

        backend = _launch_sonic_backend(
            backend_command,
            endpoint=kwargs.get(
                "sonic_endpoint", getattr(cfg.policy.executor, "sonic_endpoint", None)
            ),
            startup_timeout_s=kwargs.get("sonic_server_startup_timeout", 90.0),
            cwd=backend_cwd,
            progress_callback=keep_isaac_viewer_responsive,
        )

    def stop_backend_before_sim_shutdown():
        nonlocal backend
        if backend is None:
            return
        process = backend
        backend = None
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)

    try:
        from spark_pipeline.teleop.unitree_g1_whole_body_config import run_configured_pipeline

        return run_configured_pipeline(
            cfg,
            kwargs,
            pipeline_class=Pipeline,
            error_label="SONIC",
            before_pipeline=launch_backend_after_viewer,
            after_pipeline=stop_backend_before_sim_shutdown,
        )
    finally:
        stop_backend_before_sim_shutdown()


def _parse_cli_args(argv=None):
    parser = argparse.ArgumentParser(description="Run Unitree G1 SPARK teleop with SONIC backend.")
    parser.add_argument("--backend", choices=("mujoco", "isaac"), default="mujoco")
    parser.add_argument("--use-real", type=_parse_bool, nargs="?", const=True, default=USE_REAL)
    parser.add_argument(
        "--with-hand",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=False,
        metavar="BOOL",
        help="Use the Unitree G1 whole-body MuJoCo model with dexterous hands. Default: false.",
    )
    parser.add_argument(
        "--robot-cfg",
        default=None,
        help="Override the robot config class. By default this is selected from --with-hand.",
    )
    parser.add_argument(
        "--agent-cfg",
        default=None,
        help="Agent config class to instantiate.",
    )
    parser.add_argument(
        "--safe-algo",
        choices=("bypass", "ssa", "rssa", "sss", "rsss", "cbf", "rcbf", "pfm", "sma"),
        default="rssa",
        help="SPARK pre-safe layer for the SONIC command path. Default: rssa.",
    )
    parser.add_argument(
        "--safety-index", choices=("si1", "si1a", "si2", "si2a", "si2nn"), default="si1"
    )
    parser.add_argument(
        "--goal-tracking-type",
        choices=("legged", "pid"),
        default=None,
        help="Nominal base-goal tracker used before the SONIC safe layer.",
    )
    parser.add_argument(
        "--locomotion-control-weight",
        type=float,
        default=SONIC_LOCOMOTION_CONTROL_WEIGHT,
        help="Default pre-safe QP weight for virtual base x/y/yaw controls. Default: 10000.",
    )
    parser.add_argument(
        "--waist-control-weight",
        type=float,
        default=SONIC_WAIST_CONTROL_WEIGHT,
        help="Default pre-safe QP weight for waist yaw/roll/pitch controls. Default: 1.",
    )
    parser.add_argument(
        "--upper-body-control-weight",
        type=float,
        default=SONIC_UPPER_BODY_CONTROL_WEIGHT,
        help="Default pre-safe QP weight for arm controls. Default: 1.",
    )
    parser.add_argument(
        "--height-control-weight",
        type=float,
        default=SONIC_HEIGHT_CONTROL_WEIGHT,
        help="Default pre-safe QP weight for virtual base z/height control. Default: 0.1.",
    )
    parser.add_argument(
        "--sonic-locomotion-mode",
        choices=("slow", "walk", "run", "hybrid"),
        default=None,
        help=(
            "SONIC gait style. By default PID uses walk away from the goal "
            "and legged tracking keeps the policy-configured slow mode."
        ),
    )
    parser.add_argument("--sonic-endpoint", default="tcp://127.0.0.1:5560")
    parser.add_argument("--sonic-timeout-ms", type=int, default=None)
    parser.add_argument("--sonic-fallback-mode", choices=("hold", "default", "raise"), default=None)
    parser.add_argument("--sonic-planar-speed-scale", type=float, default=1.5)
    parser.add_argument("--sonic-motion-deadband", type=float, default=0.02)
    parser.add_argument("--base-planar-velocity-limit", type=float, default=0.20)
    parser.add_argument("--base-goal-xy-deadband", type=float, default=0.12)
    parser.add_argument("--base-goal-resume-distance", type=float, default=0.22)
    parser.add_argument("--base-goal-yaw-deadband", type=float, default=0.05)
    parser.add_argument("--base-goal-yaw-resume-distance", type=float, default=0.10)
    parser.add_argument(
        "--base-goal-slowdown-enabled", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument("--base-goal-slowdown-distance", type=float, default=0.35)
    parser.add_argument("--base-goal-slowdown-min-speed", type=float, default=0.20)
    parser.add_argument("--base-stop-lookahead-time", type=float, default=0.35)
    parser.add_argument("--base-stop-distance-bias", type=float, default=0.015)
    parser.add_argument("--base-position-kp", type=float, nargs=3, default=None)
    parser.add_argument("--base-velocity-kd", type=float, nargs=2, default=None)
    parser.add_argument("--base-orientation-kp", type=float, nargs=3, default=None)
    parser.add_argument("--base-xy-yaw-limit", type=float, nargs=3, default=None)
    parser.add_argument("--base-z-pitch-limit", type=float, nargs=2, default=None)
    parser.add_argument("--base-command-rate-limit", type=float, nargs=5, default=None)
    parser.add_argument("--base-command-deadband", type=float, nargs=5, default=None)
    parser.add_argument("--min-distance", type=float, default=None)
    parser.add_argument("--num-obstacle-task", type=int, default=None)
    parser.add_argument("--obstacle-init", type=float, nargs=3, default=None)
    parser.add_argument("--obstacle-velocity", type=float, default=None)
    parser.add_argument("--obstacle-direction", type=float, nargs=3, default=None)
    parser.add_argument("--num-obstacle-debug", type=int, default=None)
    parser.add_argument(
        "--base-disabled-identity-facing", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument(
        "--base-disabled-zero-waist-upper-body-input",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=None,
    )
    parser.add_argument(
        "--apply-safe-control-targets", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument(
        "--safe-control-only-when-triggered", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument(
        "--sonic-height-control-enabled", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument("--sonic-default-height", type=float, default=None)
    parser.add_argument("--sonic-height-min", type=float, default=None)
    parser.add_argument("--sonic-height-max", type=float, default=None)
    parser.add_argument("--sonic-height-deadband", type=float, default=None)
    parser.add_argument("--sonic-height-filter-gain", type=float, default=None)
    parser.add_argument("--sonic-height-rate-limit", type=float, default=None)
    parser.add_argument("--pre-safe-height-command-limit", type=float, default=None)
    parser.add_argument("--pre-safe-height-target-lookahead", type=float, default=None)
    parser.add_argument("--pre-safe-height-min", type=float, default=None)
    parser.add_argument("--pre-safe-height-max", type=float, default=None)
    parser.add_argument(
        "--pre-safe-height-recovery-enabled", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument("--pre-safe-height-default", type=float, default=None)
    parser.add_argument("--pre-safe-height-recovery-kp", type=float, default=None)
    parser.add_argument("--pre-safe-height-recovery-deadband", type=float, default=None)
    parser.add_argument(
        "--post-sonic-upper-body-overwrite",
        dest="post_sonic_upper_body_overwrite",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=None,
        help=(
            "Explicit test flag: after receiving SONIC output, overwrite the "
            "17 waist/arm body joints with the IK upper-body target. Default: false."
        ),
    )
    parser.add_argument(
        "--override-sonic-upper-body-target", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument(
        "--use-fixed-base-arm-kinematics", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument(
        "--disable-upper-body-target-rate-limit",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=None,
    )
    parser.add_argument(
        "--override-sonic-upper-body-motor-gains",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=None,
    )
    parser.add_argument(
        "--disable-upper-body-motor-gain-override-when-base-goal-disabled",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=None,
    )
    parser.add_argument("--upper-body-motor-kps", type=float, nargs=17, default=None)
    parser.add_argument("--upper-body-motor-kds", type=float, nargs=17, default=None)
    parser.add_argument("--upper-body-target-filter-gain", type=float, default=None)
    parser.add_argument(
        "--upper-body-gravity-compensation", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument(
        "--enable-base-goal",
        dest="base_goal_enable",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=True,
        metavar="BOOL",
        help="Enable keyboard-controlled base goal tracking. Pass false to disable. Default: true.",
    )
    parser.add_argument(
        "--control-weight",
        type=float,
        nargs=len(UNITREE_G1_SONIC_PRE_SAFE_CONTROL_NAMES),
        default=None,
        metavar="W",
        help=(
            f"{len(UNITREE_G1_SONIC_PRE_SAFE_CONTROL_NAMES)} pre-safe control weights ordered as "
            "17 upper-body controls plus linear_x, linear_y, yaw, linear_z."
        ),
    )
    parser.add_argument("--manual-movement-step-size", type=float, default=None)
    parser.add_argument(
        "--arm-goal-init-from-current-ee", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument("--enable-ros", type=_parse_bool, nargs="?", const=True, default=None)
    parser.add_argument(
        "--headless",
        dest="enable_viewer",
        action="store_false",
        default=None,
        help="Disable the viewer; the window is enabled by default.",
    )
    add_teleop_viewer_arguments(parser)
    parser.add_argument(
        "--enable-keyboard-control", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument("--enable-logger", type=_parse_bool, nargs="?", const=True, default=None)
    parser.add_argument("--enable-plotter", type=_parse_bool, nargs="?", const=True, default=None)
    parser.add_argument("--ros-version", choices=("ros1", "ros2"), default=None)
    parser.add_argument("--enable-camera", type=_parse_bool, nargs="?", const=True, default=None)
    parser.add_argument("--camera-width", type=int, default=None)
    parser.add_argument("--camera-height", type=int, default=None)
    parser.add_argument("--camera-rate-hz", type=float, default=None)
    parser.add_argument("--camera-display", type=_parse_bool, nargs="?", const=True, default=None)
    parser.add_argument("--send-cmd", type=_parse_bool, nargs="?", const=True, default=False)
    parser.add_argument("--net", default="")
    parser.add_argument("--unitree-model", default="g1")
    parser.add_argument("--level", choices=("low", "high"), default="low")
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
    )
    parser.add_argument("--digital-twin-sync-hz", type=float, default=500.0)
    parser.add_argument(
        "--enable-hand-control", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument(
        "--hand-state-debug-print", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument(
        "--hand-state-zero-invalid", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument(
        "--real-robot-location-enabled", type=_parse_bool, nargs="?", const=True, default=None
    )
    parser.add_argument("--real-robot-location-topic", default="/real_robot_location")
    parser.add_argument("--real-robot-location-timeout", type=float, default=None)
    parser.add_argument(
        "--real-robot-location-orientation-source",
        choices=("topic", "imu", "robot_imu"),
        default=None,
    )
    parser.add_argument(
        "--real-robot-location-align-imu-to-pose",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=None,
    )
    parser.add_argument("--real-robot-location-filter-alpha", type=float, default=None)
    parser.add_argument("--dt", type=float, default=None)
    parser.add_argument("--control-decimation", type=int, default=None)
    parser.add_argument(
        "--isaac-device",
        default=None,
        help="Isaac PhysX tensor device; CPU is recommended for scalar teleop.",
    )
    parser.add_argument("--isaac-usd-path", default=None)
    parser.add_argument("--sim-pd-backend", choices=("explicit", "implicit"), default=None)
    parser.add_argument("--sim-position-error-limit", type=float, default=None)
    parser.add_argument(
        "--hybrid-implicit-upper-body",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=None,
    )
    parser.add_argument("--task-dt", type=float, default=None)
    parser.add_argument("--max-num-steps", type=int, default=None)
    parser.add_argument("--save-path", default=None)
    parser.add_argument("--launch-sonic-server-cmd", default=None)
    parser.add_argument(
        "--auto-launch-sonic-server",
        type=_parse_bool,
        nargs="?",
        const=True,
        default=None,
        help="Launch the current GR00T g1_deploy_onnx_ref SPARK policy server before starting SPARK.",
    )
    parser.add_argument(
        "--sonic-deploy-root",
        default=None,
        help=(
            "Path to GR00T-WholeBodyControl/gear_sonic_deploy. Defaults to "
            "SPARK_SONIC_DEPLOY_ROOT or the standard sibling checkout."
        ),
    )
    parser.add_argument(
        "--sonic-backend-binary",
        default=None,
        help=f"SONIC backend executable. Defaults to --sonic-deploy-root/{SONIC_BACKEND_BINARY_REL}.",
    )
    parser.add_argument(
        "--sonic-backend-interface",
        default="lo",
        help="Positional network-interface argument passed to g1_deploy_onnx_ref. DDS is disabled in policy-server mode.",
    )
    parser.add_argument(
        "--sonic-policy-file",
        default=None,
        help=f"SONIC decoder policy ONNX file. Defaults to --sonic-deploy-root/{SONIC_POLICY_FILE_REL}.",
    )
    parser.add_argument(
        "--sonic-encoder-file",
        default=None,
        help=f"SONIC encoder ONNX file. Defaults to --sonic-deploy-root/{SONIC_ENCODER_FILE_REL}.",
    )
    parser.add_argument(
        "--sonic-obs-config",
        default=None,
        help=f"SONIC observation config. Defaults to --sonic-deploy-root/{SONIC_OBS_CONFIG_REL}.",
    )
    parser.add_argument(
        "--sonic-planner-file",
        default=None,
        help=f"SONIC planner ONNX file. Defaults to --sonic-deploy-root/{SONIC_PLANNER_FILE_REL}.",
    )
    parser.add_argument(
        "--sonic-motion-data",
        default=None,
        help=f"SONIC reference motion data path. Defaults to --sonic-deploy-root/{SONIC_MOTION_DATA_REL}.",
    )
    parser.add_argument("--sonic-policy-precision", choices=("16", "32"), default=None)
    parser.add_argument("--sonic-planner-precision", choices=("16", "32"), default=None)
    parser.add_argument(
        "--sonic-server-startup-timeout",
        type=float,
        default=None,
        help="Seconds to wait for a launched SONIC policy server endpoint before starting the SPARK pipeline.",
    )
    parser.add_argument(
        "--sonic-server-extra-args",
        default="",
        help="Additional raw arguments appended to g1_deploy_onnx_ref when auto-launching.",
    )
    return vars(parser.parse_args(argv))


if __name__ == "__main__":
    cli_kwargs = _parse_cli_args()
    with_hand = cli_kwargs.pop("with_hand")
    robot_cfg = cli_kwargs.pop("robot_cfg")
    agent_cfg = cli_kwargs.pop("agent_cfg")
    if robot_cfg is None:
        robot_cfg = (
            "UnitreeG1WholeBodyWithHandDynamic1Config"
            if with_hand
            else "UnitreeG1WholeBodyDynamic1Config"
        )

    run(
        robot_cfg=robot_cfg,
        agent_cfg=agent_cfg,
        **cli_kwargs,
    )
