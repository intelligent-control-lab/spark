from spark_pipeline.teleop.teleop_pipeline_config import TeleopPipelineConfig


UNITREE_G1_SONIC_CONTROL_NAMES = (
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
)

UNITREE_G1_SONIC_PRE_SAFE_CONTROL_NAMES = (
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
    "linear_x",
    "linear_y",
    "yaw",
)

# The released SONIC MuJoCo configuration (g1_29dof_sonic_model12.yaml)
# uses the crouched Unitree pose below. SONIC feeds offsets from this pose into
# its observation history, so these values must match policy_parameters.hpp.
SONIC_RELEASE_UPPER_BODY_POS = [
    0.0,
    0.0,
    0.0,
    0.2,
    0.2,
    0.0,
    0.6,
    0.0,
    0.0,
    0.0,
    0.2,
    -0.2,
    0.0,
    0.6,
    0.0,
    0.0,
    0.0,
]

SONIC_DEFAULT_ACTUATED_POS = [
    -0.312,
    0.0,
    0.0,
    0.669,
    -0.363,
    0.0,
    -0.312,
    0.0,
    0.0,
    0.669,
    -0.363,
    0.0,
] + SONIC_RELEASE_UPPER_BODY_POS

SONIC_DEFAULT_QPOS = [0.0, 0.0, 0.793, 1.0, 0.0, 0.0, 0.0] + SONIC_DEFAULT_ACTUATED_POS
SONIC_DEFAULT_QPOS_WITH_HAND = (
    SONIC_DEFAULT_QPOS[:29] + [0.0] * 7 + SONIC_DEFAULT_QPOS[29:36] + [0.0] * 7
)

UNITREE_G1_HEAD_CAMERA_CONFIG = dict(
    head=dict(
        type="fixed",
        body_name="torso_link",
        mujoco_camera_name="spark_g1_head_camera",
        frame_id="g1_head_camera_optical_frame",
        # Unitree G1 official d435_joint relative to torso_link:
        # xyz="0.0576235 0.01753 0.42987", rpy="0 0.8307767239493009 0".
        # MuJoCo cameras look along -Z, so xyaxes maps that D435 forward
        # direction to the camera optical axis with image x pointing right.
        pos=[0.12, 0.01753, 0.42987],
        xyaxes=[
            0.0,
            -1.0,
            0.0,
            0.5299192642332049,
            0.0,
            0.848048096156426,
        ],
        fovy=90.0,
        clipping_range=[0.03, 10.0],
        publish_rgb=True,
        publish_depth=True,
        distortion=[],
        distortion_model="plumb_bob",
    )
)

UNITREE_G1_HEAD_CAMERA_TOPICS = dict(
    head=dict(
        rgb="g1/head_camera/color/image_raw",
        depth="g1/head_camera/depth/image_raw",
        rgb_info="g1/head_camera/color/camera_info",
        depth_info="g1/head_camera/depth/camera_info",
    )
)


class UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig(TeleopPipelineConfig):
    """Standalone G1 teleop config using SONIC as the whole-body backend policy."""

    max_num_steps = 50000
    max_num_reset = 1
    enable_logger = False
    enable_plotter = False
    enable_safe_zone_render = False

    class metric_selection(TeleopPipelineConfig.metric_selection):
        dof_pos = False
        dof_vel = False
        dist_self = False
        dist_robot_to_env = False
        dist_goal_arm = False
        dist_goal_base = False
        seed = True
        done = True
        loop_time = True
        trigger_safe_controller = True

    class robot(TeleopPipelineConfig.robot):
        class cfg(TeleopPipelineConfig.robot.cfg):
            class_name = "UnitreeG1WholeBodyDynamic1Config"

    class env(TeleopPipelineConfig.env):
        class task(TeleopPipelineConfig.env.task):
            enable_ros = False
            mode = "Velocity"
            dt = 0.02
            max_episode_length = 50001
            num_obstacle_task = 0
            obstacle_init = [0.6, 0.0, 0.9]
            obstacle_velocity = 0.0
            obstacle_direction = [0.0, 1.0, 0.0]
            use_dual_arm = True
            base_goal_enable = True
            arm_goal_enable = True
            goal_left_init = [0.2, 0.25, 0.1]
            goal_right_init = [0.2, -0.25, 0.1]
            goal_left_velocity = 0.0
            goal_right_velocity = 0.0
            base_goal_init = [0.0, 0.0, 0.793]
            base_goal_velocity = 0.0
            reset_dof_pos = SONIC_DEFAULT_QPOS
            arm_goal_init_from_current_ee = False
            ros_params = dict(
                robot_command_topic="robot_command",
                robot_state_topic="robot_state",
                robot_frame_topic="robot_frames",
                robot_teleop_topic="robot_teleop",
                robot_gripper_topic="robot_gripper",
                obstacle_topic="human_state",
                object_name_topic="object_names",
                object_position_topic="object_positions",
                robot_teleop_joint_topic="robot_teleop_joint",
                debug_frames_topic="debug_frames",
                env_reset_topic="env_reset",
                camera_topics=UNITREE_G1_HEAD_CAMERA_TOPICS,
            )

        class agent(TeleopPipelineConfig.env.agent):
            class_name = "UnitreeG1WholeBodyMujocoAgent"
            use_action_gains = True
            dt = 0.005
            control_decimation = 4
            use_sim_dynamics = True
            enable_viewer = True
            enable_keyboard_control = True
            enable_camera = False
            camera_width = 1280
            camera_height = 720
            camera_rate_hz = 30.0
            camera_display = True
            camera_config = UNITREE_G1_HEAD_CAMERA_CONFIG
            real_time = True
            obstacle_debug = dict(
                num_obstacle=1,
                manual_movement_step_size=0.02,
            )

    class policy(TeleopPipelineConfig.policy):
        class_name = "UnitreeG1SonicSafePolicy"
        goal_tracking_type = "pid"
        # PID treats planar translation and yaw as independent mobile-base
        # channels.  Use SONIC's normal WALK expert while the goal is far
        # away, then let the executor's near-goal profile switch to SLOW_WALK
        # for braking and accurate arrival.  Keeping PID in SLOW_WALK for an
        # entire long approach can leave a valid command active without
        # meaningful locomotion progress.
        pid_sonic_locomotion_mode = "walk"
        pre_safe_enabled = True
        pre_safe_target_dt = 0.02
        pre_safe_hold_steps = 0
        pre_safe_upper_body_mode = "integrated"
        pre_safe_loco_command_limit = [0.08, 0.07, 0.16]
        pre_safe_loco_command_rate_limit = [0.012, 0.012, 0.04]
        pre_safe_height_command_limit = 0.15
        pre_safe_height_target_lookahead = 2.0
        pre_safe_height_min = 0.2
        pre_safe_height_max = 1.0
        pre_safe_height_recovery_enabled = True
        pre_safe_height_default = 0.793
        pre_safe_height_recovery_kp = 1.0
        pre_safe_height_recovery_deadband = 0.015
        pre_safe_upper_body_velocity_limit = [0.10, 0.10, 0.10] + [0.55] * 14
        pre_safe_upper_body_position_error_limit = 0.30

        class safe_robot_cfg:
            # The pre-SONIC safety stage operates on the mobile-base
            # abstraction (upper-body joints plus virtual x/y/yaw DoFs), not
            # on the floating-base whole-body simulation state.
            class_name = "UnitreeG1MobileBaseDynamic1Config"

        class safe_robot_kinematics:
            class_name = "UnitreeG1MobileBaseKinematics"

        class goal_tracking_policy:
            class_name = "LeggedLocomotionTrackingPolicy"
            tracking_dt = 0.02
            max_planar_speed = 0.07
            min_planar_speed = 0.025
            max_yaw_rate = 0.30
            position_gain = 0.65
            velocity_damping = 0.30
            yaw_gain = 0.85
            slowdown_distance = 0.45
            translation_stop_distance = 0.14
            # Resume outside the benchmark's 0.15 m success radius. A larger
            # hysteresis left the robot permanently settled at 0.15--0.22 m.
            translation_resume_distance = 0.16
            position_tolerance = 0.12
            yaw_tolerance = 0.08
            settle_speed = 0.045
            settle_cycles = 5
            align_min_distance = 0.32
            align_enter_angle = 0.55
            align_exit_angle = 0.20
            align_settle_yaw_rate = 0.08
            align_settle_cycles = 4
            command_settle_threshold = 0.01
            max_acceleration = 0.18
            max_yaw_acceleration = 0.75

        class pid_goal_tracking_policy:
            class_name = "TeleopPIDPolicy"
            # A learned gait cannot execute arbitrarily small PID corrections.
            # Latch a reached pose and resume only after a deliberate goal
            # change or material drift.
            base_goal_xy_deadband = 0.12
            base_goal_resume_distance = 0.22
            base_goal_yaw_deadband = 0.05
            base_goal_yaw_resume_distance = 0.10
            base_final_align_speed = 0.025

        class executor:
            class_name = "UnitreeG1SonicPolicy"
            sonic_endpoint = "tcp://127.0.0.1:5560"
            sonic_timeout_ms = 50
            sonic_fallback_mode = "hold"
            sonic_planar_speed_scale = 1.5
            sonic_slow_min_speed = 0.2
            sonic_slow_walk_max_speed = 0.8
            sonic_walk_min_speed = 0.8
            sonic_walk_max_speed = 2.5
            sonic_run_min_speed = 2.5
            sonic_max_speed = 3.0
            sonic_motion_deadband = 0.02
            sonic_safety_speed_quantization = 0.025
            sonic_safety_direction_update_angle = 0.06
            sonic_safety_facing_update_angle = 0.06
            sonic_locomotion_mode = "slow"
            sonic_facing_mode = "movement"
            sonic_height_control_enabled = True
            sonic_default_height = 0.793
            sonic_height_filter_gain = 0.35
            sonic_height_rate_limit = 0.006
            sonic_default_actuated_pos = SONIC_DEFAULT_ACTUATED_POS
            base_position_kp = [0.80, 0.80, 0.60]
            base_velocity_kd = [0.35, 0.35]
            base_xy_yaw_limit = [0.35, 0.35, 0.50]
            # SONIC's planner multiplies this command by its speed scale. A
            # 0.20 m/s envelope requests full walking strides and cannot stop
            # accurately at benchmark pose goals; match the stable WBT
            # locomotion envelope and brake before entering the goal radius.
            base_planar_velocity_limit = 0.12
            base_goal_xy_deadband = 0.12
            base_goal_resume_distance = 0.22
            base_goal_yaw_deadband = 0.05
            base_goal_yaw_resume_distance = 0.10
            base_goal_slowdown_enabled = True
            base_goal_slowdown_distance = 0.50
            base_goal_slowdown_min_speed = 0.20
            base_stop_lookahead_time = 0.80
            base_stop_distance_bias = 0.03
            base_command_rate_limit = [0.08, 0.08, 0.12, 0.004, 0.004]
            base_disabled_identity_facing = True
            base_disabled_zero_waist_upper_body_input = True
            post_sonic_upper_body_overwrite = False
            override_sonic_upper_body_target = False
            use_fixed_base_arm_kinematics = True
            disable_upper_body_target_rate_limit = False
            override_sonic_upper_body_motor_gains = False
            disable_upper_body_motor_gain_override_when_base_goal_disabled = True
            upper_body_target_filter_gain = 0.05
            upper_body_gravity_compensation = True
            apply_safe_control_targets = False
            safe_control_only_when_triggered = True
            safe_target_dt = 0.02

        class safe_controller(TeleopPipelineConfig.policy.safe_controller):
            class safety_index(TeleopPipelineConfig.policy.safe_controller.safety_index):
                class_name = "FirstOrderCollisionSafetyIndex"
                min_distance = {
                    "environment": 0.05,
                    "self": 0.01,
                }
                enable_self_collision = False

            class safe_algo:
                class_name = "ByPassSafeControl"

        class pre_safe_controller:
            class_name = None

            class safety_index(TeleopPipelineConfig.policy.safe_controller.safety_index):
                class_name = "FirstOrderCollisionSafetyIndex"
                min_distance = {
                    "environment": 0.05,
                    "self": 0.01,
                }
                enable_self_collision = False

            class safe_algo:
                class_name = "RelaxedSafeSetAlgorithm"
                eta_ssa = 0.3
                slack_weight = 1e3
                # QP cost order: waist (3), arms (14), planar base (3).
                # Prefer path adjustment, then the constrained arm, and avoid
                # using torso twist as the default collision response.
                control_weight = [25.0] * 3 + [1.0] * 14 + [1.0] * 3


class UnitreeG1SonicCascadeWholeBodyWithHandTeleopPipelineConfig(TeleopPipelineConfig):
    """Standalone SONIC config for the whole-body MuJoCo model with hands."""

    max_num_steps = UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.max_num_steps
    max_num_reset = UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.max_num_reset
    enable_logger = UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.enable_logger
    enable_plotter = UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.enable_plotter
    enable_safe_zone_render = (
        UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.enable_safe_zone_render
    )

    class metric_selection(UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.metric_selection):
        pass

    class robot(UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.robot):
        class cfg(UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.robot.cfg):
            class_name = "UnitreeG1WholeBodyWithHandDynamic1Config"

    class env(UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.env):
        class task(UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.env.task):
            reset_dof_pos = SONIC_DEFAULT_QPOS_WITH_HAND

    class policy(UnitreeG1SonicCascadeWholeBodyTeleopPipelineConfig.policy):
        pass
