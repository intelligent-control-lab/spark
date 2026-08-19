import numpy as np

from spark_task.autonomy.benchmark_goals import (
    DEFAULT_LEFT_ARM_GOAL_RANGE,
    benchmark_scenario_fingerprint,
    benchmark_scenario_seed,
    evaluate_goal_completion,
    sample_benchmark_scenario,
    sample_bounded_position,
)
from spark_policy.control.whole_body.unitree_g1.sonic.batched_policy import (
    UnitreeG1BatchedSonicPolicy,
)
from spark_policy.control.whole_body.unitree_g1.sonic.policy import (
    SONIC_LOCOMOTION_SLOW_WALK,
    SonicBaseTracker,
    UnitreeG1SonicPolicy,
)
from spark_policy.control.pid.teleop import TeleopPIDPolicy
from spark_policy.composed_policy.unitree_g1.sonic_safe import (
    UnitreeG1SonicSafePolicy,
)
from spark_pipeline.teleop.unitree_g1_sonic_whole_body_teleop_pipeline_config import (
    SONIC_DEFAULT_ACTUATED_POS,
    SONIC_DEFAULT_QPOS,
)


def test_sonic_reset_pose_matches_released_mujoco_policy_config():
    expected_lower_body = [
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
    ]
    expected_upper_body = [
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

    np.testing.assert_allclose(SONIC_DEFAULT_ACTUATED_POS[:12], expected_lower_body)
    np.testing.assert_allclose(SONIC_DEFAULT_ACTUATED_POS[12:], expected_upper_body)
    np.testing.assert_allclose(SONIC_DEFAULT_QPOS[7:], SONIC_DEFAULT_ACTUATED_POS)


def test_bounded_goal_sampling_is_deterministic_and_in_range():
    first = sample_bounded_position(np.random.RandomState(20), DEFAULT_LEFT_ARM_GOAL_RANGE)
    second = sample_bounded_position(np.random.RandomState(20), DEFAULT_LEFT_ARM_GOAL_RANGE)
    np.testing.assert_allclose(first, second)
    bounds = np.asarray(DEFAULT_LEFT_ARM_GOAL_RANGE)
    assert np.all(first >= bounds[:, 0])
    assert np.all(first <= bounds[:, 1])


def test_benchmark_scenario_is_backend_and_device_neutral():
    kwargs = dict(
        seed=benchmark_scenario_seed(0),
        root_position=(0.0, 0.0, 0.793),
        right_arm_goal_range=((-0.05, 0.30), (-0.36, -0.18), (0.0, 0.28)),
        left_arm_goal_range=((-0.05, 0.30), (0.18, 0.36), (0.0, 0.28)),
        base_goal_range=((1.5, 1.8), (-0.4, 0.4), (0.793, 0.793)),
        base_goal_rot_range=(-0.35, 0.35),
        base_goal_relative_to_current=True,
        base_goal_workspace_range=((-2.0, 2.0), (-2.0, 2.0)),
        base_goal_minimum_distance=1.5,
        num_obstacles=10,
        obstacle_range=((-1.0, 1.0), (-1.0, 1.0), (0.5, 1.3)),
        obstacle_radius=0.05,
        obstacle_keepout=0.05,
        obstacle_goal_keepaway=0.15,
    )

    mujoco_sample = sample_benchmark_scenario(**kwargs)
    isaac_cpu_sample = sample_benchmark_scenario(**kwargs)
    isaac_cuda_sample = sample_benchmark_scenario(**kwargs)

    assert benchmark_scenario_fingerprint(mujoco_sample) == benchmark_scenario_fingerprint(
        isaac_cpu_sample
    )
    assert benchmark_scenario_fingerprint(mujoco_sample) == benchmark_scenario_fingerprint(
        isaac_cuda_sample
    )
    np.testing.assert_array_equal(
        mujoco_sample.obstacle_positions, isaac_cuda_sample.obstacle_positions
    )


def test_benchmark_scenario_stream_separates_environments_and_episodes():
    assert benchmark_scenario_seed(0, environment_index=0, episode_index=0) == 1
    assert benchmark_scenario_seed(0, environment_index=0, episode_index=1) == 2
    assert benchmark_scenario_seed(0, environment_index=1, episode_index=0) == 100_004


def test_benchmark_scenario_enforces_surface_keepouts():
    scenario = sample_benchmark_scenario(
        seed=7,
        arm_goal_enabled=False,
        base_goal_enabled=False,
        num_obstacles=8,
        obstacle_range=((-1.0, 1.0), (-1.0, 1.0), (0.5, 1.0)),
        obstacle_radius=0.05,
        obstacle_keepout=0.10,
        obstacle_robot_keepaway=0.15,
        robot_collision_centers=((0.0, 0.0, 0.7),),
        robot_collision_radii=(0.1,),
    )

    centers = scenario.obstacle_positions
    pair_distance = np.linalg.norm(centers[:, None] - centers[None], axis=-1)
    pair_distance += np.eye(len(centers)) * 10.0
    assert pair_distance.min() >= 0.20
    assert np.linalg.norm(centers - np.array([0.0, 0.0, 1.493]), axis=1).min() >= 0.30


def test_goal_completion_supports_independent_environment_rows():
    result = evaluate_goal_completion(
        left_position=[[0.00, 0.00, 0.00], [0.10, 0.00, 0.00]],
        right_position=[[0.00, 0.00, 0.00], [0.00, 0.10, 0.00]],
        left_goal=np.zeros((2, 3)),
        right_goal=np.zeros((2, 3)),
        base_position=[[0.02, 0.00, 0.8], [0.30, 0.00, 0.8]],
        base_goal=np.zeros((2, 3)),
        arm_tolerance=0.05,
        base_tolerance=0.15,
    )
    np.testing.assert_array_equal(result["reached"], [True, False])
    np.testing.assert_array_equal(result["arm_reached"], [True, False])
    np.testing.assert_array_equal(result["base_reached"], [True, False])


def test_goal_completion_supports_single_arm_embodiments():
    result = evaluate_goal_completion(
        left_position=None,
        right_position=[[0.01, 0.00, 0.00], [0.10, 0.00, 0.00]],
        left_goal=None,
        right_goal=np.zeros((2, 3)),
        arm_tolerance=0.05,
        base_enabled=False,
        dual_arm=False,
    )

    np.testing.assert_array_equal(result["reached"], [True, False])
    np.testing.assert_array_equal(result["arm_reached"], [True, False])
    np.testing.assert_array_equal(result["left_error"], [0.0, 0.0])


def test_goal_completion_can_require_end_effector_orientation():
    identity = np.eye(3)
    upside_down = np.diag([1.0, -1.0, -1.0])
    result = evaluate_goal_completion(
        left_position=None,
        right_position=[0.0, 0.0, 0.0],
        left_goal=None,
        right_goal=[0.0, 0.0, 0.0],
        right_orientation=identity,
        right_goal_orientation=upside_down,
        arm_tolerance=0.05,
        arm_orientation_tolerance=0.1,
        arm_position_only=False,
        base_enabled=False,
        dual_arm=False,
    )

    assert not bool(result["reached"])
    assert result["right_error"] == 0.0
    assert result["right_orientation_error"] == np.pi


def test_sonic_base_goal_does_not_implicitly_command_height():
    policy = UnitreeG1SonicPolicy.__new__(UnitreeG1SonicPolicy)
    policy.sonic_height_control_enabled = True
    policy.sonic_height_min = 0.2
    policy.sonic_height_max = 1.0
    policy.sonic_default_height = 0.793
    policy.sonic_height_deadband = 0.01
    base_goal = np.eye(4)
    base_goal[2, 3] = 0.4

    assert policy._height_command({"goal_teleop": {"base": base_goal}}) == -1.0
    assert policy._height_command({"sonic_height": 0.4}) == 0.4


def test_batched_sonic_height_is_an_explicit_fourth_channel():
    quat = np.array([0.0, 0.0, 0.0, 1.0])
    nominal = UnitreeG1BatchedSonicPolicy._planner([0.1, 0.0, 0.0], quat)
    safety = UnitreeG1BatchedSonicPolicy._planner([0.1, 0.0, 0.0, 0.65], quat)

    assert nominal["height"] == -1.0
    assert safety["height"] == 0.65


def test_batched_sonic_yaw_command_changes_planner_facing():
    quat = np.array([0.0, 0.0, 0.0, 1.0])
    planner = UnitreeG1BatchedSonicPolicy._planner([0.0, 0.0, 0.4], quat)

    np.testing.assert_allclose(planner["facing"][:2], [np.cos(0.5), np.sin(0.5)], atol=1.0e-7)


def test_batched_sonic_pid_uses_walk_far_and_slow_walk_near_goal():
    policy = UnitreeG1BatchedSonicPolicy.__new__(UnitreeG1BatchedSonicPolicy)
    policy.locomotion_mode = "walk"
    policy.slowdown_distance = 0.50
    policy._last_planner_movement = [None]
    policy._last_planner_facing = [None]
    command = np.array([0.12, 0.0, 0.0])
    quat = np.array([0.0, 0.0, 0.0, 1.0])

    far = policy._planner(0, command, quat, 1.5)
    near = policy._planner(0, command, quat, 0.3)

    assert far["mode"] == 2
    assert far["speed"] >= 0.8
    assert near["mode"] == 1
    assert 0.2 <= near["speed"] <= 0.8


def test_batched_sonic_facing_updates_below_old_heading_hysteresis():
    policy = UnitreeG1BatchedSonicPolicy.__new__(UnitreeG1BatchedSonicPolicy)
    policy.locomotion_mode = "walk"
    policy.slowdown_distance = 0.50
    policy._last_planner_movement = [None]
    policy._last_planner_facing = [None]
    command = np.array([0.0, 0.0, 0.08])

    policy._planner(0, command, np.array([0.0, 0.0, 0.0, 1.0]), 0.1)
    yaw = 0.03
    planner = policy._planner(
        0,
        command,
        np.array([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)]),
        0.1,
    )

    expected_facing_yaw = yaw + 0.1
    np.testing.assert_allclose(
        planner["facing"][:2],
        [np.cos(expected_facing_yaw), np.sin(expected_facing_yaw)],
        atol=1.0e-7,
    )


def test_batched_sonic_can_restart_one_stalled_base_tracker():
    class FakeTracker:
        def __init__(self):
            self.reset_count = 0

        def reset(self):
            self.reset_count += 1

    policy = UnitreeG1BatchedSonicPolicy.__new__(UnitreeG1BatchedSonicPolicy)
    policy._num_envs = 2
    policy._base_trackers = [FakeTracker(), FakeTracker()]
    policy._last_planner_movement = [np.ones(2), np.ones(2)]
    policy._last_planner_facing = [np.ones(2), np.ones(2)]

    policy.reset_base_tracking([1])

    assert policy._base_trackers[0].reset_count == 0
    assert policy._base_trackers[1].reset_count == 1
    assert policy._last_planner_movement[0] is not None
    assert policy._last_planner_movement[1] is None
    assert policy._last_planner_facing[1] is None


def test_batched_sonic_matches_scalar_upper_body_and_motor_contract():
    import torch
    from spark_policy.control.whole_body.unitree_g1.wbt.config import (
        WBT_MOTOR_KDS,
        WBT_MOTOR_KPS,
    )

    class FakeClient:
        endpoint = "fake"

        def __init__(self):
            self.request_value = None

        def request(self, value):
            self.request_value = value
            return {
                "ok": True,
                "target_actuated_pos": np.arange(29, dtype=np.float32),
            }

        def close(self):
            pass

    policy = UnitreeG1BatchedSonicPolicy(["tcp://127.0.0.1:65530"], batch_size=1, device="cpu")
    fake = FakeClient()
    policy.clients = [fake]
    try:
        target, info = policy.infer_tensor(
            body_joint_pos=torch.zeros(1, 29),
            body_joint_vel=torch.zeros(1, 29),
            root_pose_w=torch.tensor([[0.0, 0.0, 0.793, 0.0, 0.0, 0.0, 1.0]]),
            root_angular_velocity=torch.zeros(1, 3),
            command=torch.tensor([[0.0, 0.0, 0.0, 0.793]]),
            upper_body_target=torch.ones(1, 17),
            base_goal_distance=torch.zeros(1),
        )
    finally:
        policy.close()

    # Sonic's response remains authoritative; SPARK does not overwrite its
    # upper-body output with the conditioning target after inference.
    torch.testing.assert_close(target[0], torch.arange(29, dtype=torch.float32))
    np.testing.assert_allclose(info["motor_kps"][0].numpy(), WBT_MOTOR_KPS)
    np.testing.assert_allclose(info["motor_kds"][0].numpy(), WBT_MOTOR_KDS)
    # Scalar filtering uses gain 0.05 followed by the per-joint rate limit.
    sent_upper = np.asarray(fake.request_value["upper_body_position"])
    assert np.isclose(sent_upper[0], 0.006)
    assert np.isclose(sent_upper[3], 0.014)
    # Absolute height is filtered around SONIC's nominal posture instead of
    # being jerk-limited from zero at the start of every episode.
    assert fake.request_value["planner"]["height"] == -1.0


def test_sonic_stops_translation_while_finishing_goal_yaw():
    policy = UnitreeG1SonicPolicy.__new__(UnitreeG1SonicPolicy)
    policy.base_goal_xy_deadband = 0.15
    policy.base_goal_resume_distance = 0.22
    policy.base_goal_yaw_deadband = 0.05
    policy.base_goal_yaw_resume_distance = 0.10
    policy.base_stop_lookahead_time = 0.35
    policy.base_stop_distance_bias = 0.015
    policy.base_position_kp = np.array([0.8, 0.8, 0.6])
    policy.base_velocity_kd = np.array([0.35, 0.35])
    policy.base_orientation_kp = np.array([0.5, 0.5, 0.5])
    policy._base_goal_arrived = False
    policy._base_translation_stopped = False
    policy._base_translation_hold_xy = None
    policy._estimate_base_xy_velocity = lambda _xy: np.zeros(2)
    policy._filter_base_command = lambda command, **_kwargs: np.asarray(command)

    base = np.eye(4)
    goal = np.eye(4)
    goal[:2, 3] = [0.10, 0.0]
    angle = 0.30
    goal[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    command = policy._base_command_from_goal(
        {"robot_base_frame": base},
        {"base_goal_enable": True, "goal_teleop": {"base": goal}},
    )

    np.testing.assert_allclose(command[:2], 0.0)
    assert command[2] > 0.0

    # Small gait drift must not restart translation on the next cycle.
    drifted_base = base.copy()
    drifted_base[0, 3] = -0.03
    command = policy._base_command_from_goal(
        {"robot_base_frame": drifted_base},
        {"base_goal_enable": True, "goal_teleop": {"base": goal}},
    )
    np.testing.assert_allclose(command[:2], 0.0)


def test_sonic_uses_slow_walk_for_near_goal_corrections():
    policy = UnitreeG1SonicPolicy.__new__(UnitreeG1SonicPolicy)
    policy.base_goal_slowdown_enabled = True
    policy.base_goal_slowdown_distance = 0.35
    policy.base_goal_xy_deadband = 0.12
    policy.base_goal_slowdown_min_speed = 0.20
    policy.sonic_slow_walk_max_speed = 0.8

    mode, speed = policy._apply_near_goal_speed_profile(
        speed=0.6, locomotion_mode="walk", xy_distance=0.20
    )

    assert mode == SONIC_LOCOMOTION_SLOW_WALK
    assert 0.20 <= speed < 0.6


def test_sonic_base_tracker_is_jerk_limited_and_safety_responsive():
    nominal = SonicBaseTracker(0.02, [0.2] * 5, [1.0] * 5, [0.6] * 5, [4.0] * 5)
    safety = SonicBaseTracker(0.02, [0.2] * 5, [1.0] * 5, [0.6] * 5, [4.0] * 5)
    target = np.array([0.12, 0.0, 0.0, 0.0, 0.0])

    nominal_command = nominal.update(target)
    safety_command = safety.update(target, safety_active=True)

    assert 0.0 < nominal_command[0] < safety_command[0] < target[0]
    assert nominal.phase == "tracking"
    assert safety.phase == "safety"


def test_teleop_pid_keeps_pose_yaw_active_during_translation():
    policy = TeleopPIDPolicy.__new__(TeleopPIDPolicy)
    policy.base_goal_xy_deadband = 0.12
    policy.base_goal_resume_distance = 0.22
    policy.base_goal_yaw_deadband = 0.05
    policy.base_goal_yaw_resume_distance = 0.10
    policy.base_final_align_speed = 0.025
    policy._base_goal_hold_frame = None

    goal = np.eye(4)
    goal[:2, 3] = [0.30, 0.20]
    target_yaw = 1.20
    goal[:2, :2] = [
        [np.cos(target_yaw), -np.sin(target_yaw)],
        [np.sin(target_yaw), np.cos(target_yaw)],
    ]
    info = {}

    policy._apply_base_goal_hysteresis(
        np.zeros(20),
        {"robot_base_frame": np.eye(4)},
        {"base_goal_enable": True, "goal_teleop": {"base": goal}},
        info,
    )

    assert info["locomotion_phase"] == "approach"
    assert np.isclose(info["locomotion_facing_yaw"], target_yaw)


def test_sonic_safe_upper_body_correction_only_updates_active_branch():
    policy = UnitreeG1SonicSafePolicy.__new__(UnitreeG1SonicSafePolicy)
    policy.pre_safe_target_dt = 0.02
    policy.pre_safe_upper_body_mode = "delta"
    policy.pre_safe_upper_body_velocity_limit = np.ones(17)
    policy._pre_safe_upper_body_cmd = None

    override, mask, delta = policy._upper_body_delta_from_sport_control(
        agent_feedback={},
        sport_u_ref=np.zeros(20),
        sport_u_safe=np.ones(20),
        safe_active=True,
        active_sides={"left"},
    )

    assert override is None
    assert mask is None
    np.testing.assert_allclose(delta[:3], 0.0)
    np.testing.assert_allclose(delta[3:10], 0.02)
    np.testing.assert_allclose(delta[10:], 0.0)


def test_wbt_safe_upper_body_correction_only_updates_active_branch():
    from spark_policy.composed_policy.unitree_g1.wbt_safe import UnitreeG1WBTSafePolicy

    policy = UnitreeG1WBTSafePolicy.__new__(UnitreeG1WBTSafePolicy)
    policy.pre_safe_target_dt = 0.02
    policy.pre_safe_upper_body_mode = "delta"
    policy.pre_safe_upper_body_velocity_limit = np.ones(17)
    policy._pre_safe_upper_body_cmd = None

    override, mask, delta = policy._upper_body_delta_from_sport_control(
        agent_feedback={},
        sport_u_ref=np.zeros(20),
        sport_u_safe=np.ones(20),
        safe_active=True,
        active_sides={"left"},
    )

    assert override is None
    assert mask is None
    np.testing.assert_allclose(delta[:3], 0.0)
    np.testing.assert_allclose(delta[3:10], 0.02)
    np.testing.assert_allclose(delta[10:], 0.0)


def test_wbt_safe_torso_correction_does_not_overwrite_arm_targets():
    from spark_policy.composed_policy.unitree_g1.wbt_safe import UnitreeG1WBTSafePolicy

    policy = UnitreeG1WBTSafePolicy.__new__(UnitreeG1WBTSafePolicy)
    policy.pre_safe_target_dt = 0.02
    policy.pre_safe_upper_body_mode = "delta"
    policy.pre_safe_upper_body_velocity_limit = np.ones(17)
    policy._pre_safe_upper_body_cmd = None

    _, _, delta = policy._upper_body_delta_from_sport_control(
        agent_feedback={},
        sport_u_ref=np.zeros(20),
        sport_u_safe=np.ones(20),
        safe_active=True,
        active_sides={"waist"},
    )

    np.testing.assert_allclose(delta[:3], 0.02)
    np.testing.assert_allclose(delta[3:], 0.0)


def test_sonic_grouped_safety_weights_match_twenty_control_layout():
    from example.unitree_g1.sonic_support import _default_sonic_control_weight

    weight = _default_sonic_control_weight(
        waist=25.0,
        upper_body=1.0,
        locomotion=1.0,
        height=0.1,
    )
    assert len(weight) == 20
    np.testing.assert_allclose(weight[:3], 25.0)
    np.testing.assert_allclose(weight[3:17], 1.0)
    np.testing.assert_allclose(weight[17:], 1.0)


def test_sonic_pid_safe_translation_does_not_replace_pose_facing():
    policy = UnitreeG1SonicSafePolicy.__new__(UnitreeG1SonicSafePolicy)
    policy.goal_tracking_type = "pid"

    facing = policy._facing_yaw_after_pre_safe(
        facing_yaw=1.20,
        nominal_sonic_command=np.array([0.10, 0.00, 0.20, 0.0, 0.0]),
        # Large lateral safety correction, small yaw correction.
        safe_sonic_command=np.array([0.00, 0.08, 0.15, 0.0, 0.0]),
        sport_u_safe=np.zeros(20),
        agent_feedback={},
    )

    assert np.isclose(facing, 1.15)
