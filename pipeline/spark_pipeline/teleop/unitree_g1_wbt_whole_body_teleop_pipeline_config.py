from spark_pipeline.teleop.teleop_pipeline_config import TeleopPipelineConfig


UNITREE_G1_WBT_PRE_SAFE_CONTROL_NAMES = (
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

UNITREE_G1_WBT_SAFE_TELEOP_DEFAULTS = dict(
    use_real=False,
    robot_cfg="UnitreeG1WholeBodyWithHandDynamic1Config",
    safe_algo="rssa",
    safety_index="si1",
    min_distance=0.05,
    num_obstacle_task=0,
    enable_ros=False,
    base_position_kp=[0.50, 0.50, 0.50],
    base_orientation_kp=[0.30, 0.30, 0.50],
    base_xy_yaw_limit=[0.06, 0.04, 0.15],
    base_command_rate_limit=[0.01, 0.01, 0.04, 0.01, 0.003],
    base_command_deadband=[0.008, 0.008, 0.025, 0.004, 0.004],
    loco_start_threshold=0.03,
    safe_loco_start_threshold=0.02,
    stop_threshold=[0.04, 0.04, 0.08],
    base_goal_xy_deadband=0.05,
    base_goal_resume_distance=0.10,
    base_goal_yaw_deadband=0.07,
    base_goal_yaw_resume_distance=0.14,
    pre_safe_loco_command_limit=[0.06, 0.04, 0.15],
    pre_safe_loco_command_rate_limit=[0.01, 0.01, 0.04],
    control_weight=[10000.0] * 3 + [1.0] * 14 + [100.0, 100.0, 100.0],
    pre_safe_height_command_limit=0.0,
)

UNITREE_G1_HEAD_CAMERA_CONFIG = dict(
    head=dict(
        type="fixed",
        body_name="torso_link",
        mujoco_camera_name="spark_g1_head_camera",
        frame_id="g1_head_camera_optical_frame",
        # Teleoperation view: retain the torso-mounted head-camera model but
        # move the optical center forward to avoid head self-occlusion and use
        # a shallower 32-degree downward pitch so both hands stay in frame.
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


class _UnitreeG1WBTDefaults(TeleopPipelineConfig):
    """G1 WBT policy running through the whole-body MuJoCo agent."""

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
            ros_params = dict(
                robot_command_topic="robot_command",
                robot_state_topic="robot_state",
                robot_state_publish_mode="g1_body_29",
                robot_frame_topic="robot_frames",
                robot_location_topic="robot_location",
                robot_hand_state_topic="robot_hand_state",
                robot_teleop_topic="robot_teleop",
                robot_gripper_topic="robot_gripper",
                obstacle_topic="human_state",
                object_name_topic="object_names",
                object_position_topic="object_positions",
                robot_teleop_joint_topic="robot_teleop_joint",
                robot_teleop_mode_topic="robot_teleop_mode",
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
        # Intermediate configuration shared by the executable cascade
        # variants below. It is not itself a complete policy composition.
        class_name = None

        class wbt_arm_robot_cfg:
            class_name = "UnitreeG1DualArmDynamic1Config"

        class wbt_arm_kinematics:
            class_name = "UnitreeG1DualArmKinematics"

        class executor:
            class_name = "UnitreeG1WBTPolicy"
            zero_squat_command = False
            apply_safe_control_targets = True
            safe_control_only_when_triggered = True
            safe_target_dt = 0.02
            # Conservative learned-locomotion envelope. Safety may request an
            # ideal Cartesian correction that is not dynamically feasible for
            # WBT, so filter/rate-limit it before selecting the loco expert.
            base_position_kp = [0.50, 0.50, 0.50]
            base_orientation_kp = [0.30, 0.30, 0.50]
            base_xy_yaw_limit = [0.06, 0.04, 0.15]
            base_z_pitch_limit = [0.10, 0.03]
            base_command_rate_limit = [0.01, 0.01, 0.04, 0.01, 0.003]
            base_command_deadband = [0.008, 0.008, 0.025, 0.004, 0.004]
            loco_command_scale = [3.0, 3.0, 5.0]
            squat_command_scale = [10.0, 10.0]
            loco_start_threshold = 0.03
            safe_loco_start_threshold = 0.02
            stop_threshold = [0.04, 0.04, 0.08]
            base_goal_xy_deadband = 0.05
            base_goal_resume_distance = 0.10
            base_goal_yaw_deadband = 0.07
            base_goal_yaw_resume_distance = 0.14
            motor_kp_scale = 1.0
            motor_kd_scale = 1.0
            upper_body_target_rate_limit = [0.006, 0.006, 0.006] + [0.014] * 14

        class safe_controller(TeleopPipelineConfig.policy.safe_controller):
            class safe_algo:
                class_name = "RelaxedSafeSetAlgorithm"
                eta_ssa = 0.3
                slack_weight = 1e3
                control_weight = [1.0] * 29


class UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig(TeleopPipelineConfig):
    """G1 WBT policy with sport-mode-style pre-WBT safety."""

    pre_safe_control_names = UNITREE_G1_WBT_PRE_SAFE_CONTROL_NAMES
    max_num_steps = _UnitreeG1WBTDefaults.max_num_steps
    max_num_reset = _UnitreeG1WBTDefaults.max_num_reset
    enable_logger = _UnitreeG1WBTDefaults.enable_logger
    enable_plotter = _UnitreeG1WBTDefaults.enable_plotter
    enable_safe_zone_render = _UnitreeG1WBTDefaults.enable_safe_zone_render

    class metric_selection(_UnitreeG1WBTDefaults.metric_selection):
        pass

    class robot(_UnitreeG1WBTDefaults.robot):
        pass

    class env(_UnitreeG1WBTDefaults.env):
        pass

    class policy(_UnitreeG1WBTDefaults.policy):
        class_name = "UnitreeG1WBTSafePolicy"
        goal_tracking_type = "pid"
        pre_safe_enabled = True
        pre_safe_target_dt = 0.02
        pre_safe_hold_steps = 0
        pre_safe_upper_body_mode = "integrated"
        pre_safe_loco_command_limit = [0.06, 0.04, 0.15]
        pre_safe_loco_command_rate_limit = [0.01, 0.01, 0.04]
        pre_safe_height_command_limit = 0.0
        pre_safe_height_target_lookahead = 2.0
        pre_safe_height_min = 0.2
        pre_safe_height_max = 1.0
        pre_safe_height_recovery_enabled = True
        pre_safe_height_default = 0.793
        pre_safe_height_recovery_kp = 1.0
        pre_safe_height_recovery_deadband = 0.015
        pre_safe_height_priority = True
        pre_safe_height_priority_deadband = 0.01
        pre_safe_height_priority_xy_deadband = 0.02
        pre_safe_height_priority_yaw_deadband = 0.05
        pre_safe_upper_body_velocity_limit = [0.0, 0.0, 0.0] + [0.55] * 14
        pre_safe_upper_body_position_error_limit = [0.0, 0.0, 0.0] + [0.30] * 14

        class safe_robot_cfg:
            class_name = "UnitreeG1MobileBaseDynamic1Config"

        class safe_robot_kinematics:
            class_name = "UnitreeG1MobileBaseKinematics"

        class goal_tracking_policy:
            # Use the same acceleration-limited SE(2) tracker as the SONIC
            # composition.  The learned WBT executor should consume a stable
            # locomotion command, rather than implementing a second PID path.
            class_name = "LeggedLocomotionTrackingPolicy"
            tracking_dt = 0.02
            max_planar_speed = 0.06
            min_planar_speed = 0.02
            max_yaw_rate = 0.15
            max_acceleration = 0.15
            max_yaw_acceleration = 0.40

        class pid_goal_tracking_policy:
            class_name = "TeleopPIDPolicy"
            base_goal_xy_deadband = 0.12
            base_goal_resume_distance = 0.22
            base_goal_yaw_deadband = 0.05
            base_goal_yaw_resume_distance = 0.10
            base_final_align_speed = 0.02

        class executor(_UnitreeG1WBTDefaults.policy.executor):
            apply_safe_control_targets = False

        class safe_controller(_UnitreeG1WBTDefaults.policy.safe_controller):
            class safe_algo:
                class_name = "ByPassSafeControl"

        class pre_safe_controller:
            class_name = None

            class safety_index(_UnitreeG1WBTDefaults.policy.safe_controller.safety_index):
                class_name = "FirstOrderCollisionSafetyIndex"
                min_distance = {
                    "environment": 0.05,
                    "self": 0.01,
                }
                enable_self_collision = True

            class safe_algo(_UnitreeG1WBTDefaults.policy.safe_controller.safe_algo):
                class_name = "RelaxedSafeSetAlgorithm"
                eta_ssa = 0.3
                slack_weight = 1e3
                # MobileBase exposes 17 upper-body and three planar/yaw
                # controls. WBT height is an internal policy target rather
                # than an additional robot-config control dimension.
                control_weight = [10000.0, 10000.0, 10000.0] + [1.0] * 14 + [100.0, 100.0, 100.0]


class UnitreeG1WBTCascadeWholeBodyWithHandTeleopPipelineConfig(TeleopPipelineConfig):
    """Cascade WBT pipeline config for the whole-body MuJoCo model with hands."""

    max_num_steps = UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.max_num_steps
    max_num_reset = UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.max_num_reset
    enable_logger = UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.enable_logger
    enable_plotter = UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.enable_plotter
    enable_safe_zone_render = (
        UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.enable_safe_zone_render
    )
    pre_safe_control_names = UNITREE_G1_WBT_PRE_SAFE_CONTROL_NAMES

    class metric_selection(UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.metric_selection):
        pass

    class env(UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.env):
        pass

    class robot(UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.robot):
        class cfg(UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.robot.cfg):
            class_name = "UnitreeG1WholeBodyWithHandDynamic1Config"

    class policy(UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.policy):
        class executor(UnitreeG1WBTCascadeWholeBodyTeleopPipelineConfig.policy.executor):
            disable_upper_body_target_rate_limit = False
            upper_body_target_filter_gain = 0.05
            upper_body_target_rate_limit = [0.0, 0.0, 0.0] + [0.014] * 14
            upper_body_gravity_compensation = True
