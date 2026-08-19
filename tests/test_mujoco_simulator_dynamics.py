"""Cross-robot MuJoCo simulator-dynamics conformance tests."""

from __future__ import annotations

import importlib
import inspect

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

import spark_agent
import spark_robot
from spark_robot import RobotConfig


def _mujoco_configs():
    seen = set()
    for name in sorted(dir(spark_robot)):
        config_type = getattr(spark_robot, name)
        if (
            not inspect.isclass(config_type)
            or config_type is RobotConfig
            or not issubclass(config_type, RobotConfig)
            or inspect.isabstract(config_type)
            or config_type in seen
        ):
            continue
        seen.add(config_type)
        try:
            config = config_type()
        except TypeError:
            continue
        if "mujoco" in config.agent_class_names:
            yield name, config


def _hold_action(name, config):
    if not name.startswith("UnitreeG1WholeBody"):
        return {}
    target = np.array(
        [config.DefaultDoFVal[getattr(config.DoFs, motor.name)] for motor in config.MujocoMotors],
        dtype=float,
    )
    return {"action_info": {"target_actuated_pos": target}}


@pytest.mark.parametrize(
    "config_name",
    (
        "AgiBotG1FixedBaseDynamic1Config",
        "AgiBotG1MobileBaseDynamic1Config",
        "GalaxeaR1LiteFixedBaseDynamic1CollisionConfig",
        "GalaxeaR1LiteRightArmDynamic1CollisionConfig",
        "GalaxeaR1LiteDualArmDynamic1CollisionConfig",
        "GalaxeaR1LiteMobileBaseDynamic1CollisionConfig",
        "FanucLRMate200iDSingleArmDynamic1CollisionConfig",
        "FanucLRMate200iDDualArmDynamic1Config",
        "KinovaGen3SingleArmDynamic1CollisionConfig",
        "KinovaGen3DualArmDynamic1Config",
        "KukaIIWA14SingleArmDynamic1CollisionConfig",
        "KukaIIWA14DualArmDynamic1Config",
        "UnitreeG1FixedBaseDynamic1Config",
        "UnitreeG1RightArmDynamic1Config",
        "UnitreeG1MobileBaseDynamic1Config",
    ),
)
def test_joint_control_agents_anchor_simulator_dynamics_to_feedback(config_name):
    config = getattr(spark_robot, config_name)()
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    source = inspect.getsource(agent_type._send_control_sim_dynamics)
    assert "_compose_simulator_dynamics_state()" in source


@pytest.mark.parametrize(("name", "config"), tuple(_mujoco_configs()))
def test_declared_mujoco_simulator_dynamics_hold_finite_default_state(name, config):
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    agent = agent_type(
        config,
        dynamics_backend="simulator",
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
    )
    try:
        assert agent.dynamics_backend == "simulator"
        assert not agent.stabilize_sim_dynamics_joint_positions
        assert agent.dt == pytest.approx(config.simulator_dynamics.physics_dt)
        assert agent.control_decimation == config.simulator_dynamics.control_decimation

        agent.reset({})
        before = np.asarray(agent.get_feedback()["dof_pos_fbk"], dtype=float)
        cycles = max(
            1,
            round(
                config.simulator_dynamics.hold_duration / config.simulator_dynamics.control_period
            ),
        )
        action = _hold_action(name, config)
        for _ in range(cycles):
            agent.send_control(np.zeros(len(config.Control), dtype=float), **action)

        feedback = agent.get_feedback()
        position = np.asarray(feedback["dof_pos_fbk"], dtype=float)
        velocity = np.asarray(feedback["dof_vel_fbk"], dtype=float)
        assert np.isfinite(position).all(), name
        assert np.isfinite(velocity).all(), name
        assert np.max(np.abs(position - before)) <= config.simulator_dynamics.max_position_drift, (
            name
        )
        assert np.max(np.abs(velocity)) <= config.simulator_dynamics.max_abs_velocity, name
    finally:
        agent.close_viewer()


@pytest.mark.parametrize(
    "config_name",
    (
        "KinovaGen3SingleArmDynamic1CollisionConfig",
        "KinovaGen3DualArmDynamic1CollisionConfig",
        "KukaIIWA14SingleArmDynamic1CollisionConfig",
        "KukaIIWA14DualArmDynamic1CollisionConfig",
        "UnitreeG1RightArmDynamic1Config",
        "UnitreeG1DualArmDynamic1Config",
        "UnitreeG1FixedBaseDynamic1Config",
        "UnitreeG1MobileBaseDynamic1Config",
    ),
)
def test_manipulator_position_servos_hold_against_gravity(config_name):
    config = getattr(spark_robot, config_name)()
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    agent = agent_type(
        config,
        dynamics_backend="simulator",
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
    )
    try:
        agent.reset({})
        initial = np.asarray(agent.get_feedback()["dof_pos_fbk"], dtype=float)
        positions = []
        velocities = []
        for _ in range(250):
            agent.send_control(np.zeros(len(config.Control), dtype=float))
            feedback = agent.get_feedback()
            positions.append(feedback["dof_pos_fbk"])
            velocities.append(feedback["dof_vel_fbk"])

        assert np.max(np.abs(np.asarray(positions) - initial)) < 1.0e-3
        assert np.max(np.abs(np.asarray(velocities))) < 1.0e-3
    finally:
        agent.close_viewer()


@pytest.mark.parametrize(
    "config_name",
    (
        "KinovaGen3SingleArmDynamic1CollisionConfig",
        "KinovaGen3DualArmDynamic1Config",
        "KukaIIWA14SingleArmDynamic1CollisionConfig",
        "KukaIIWA14DualArmDynamic1Config",
    ),
)
def test_manipulator_velocity_commands_survive_the_damped_position_servo(config_name):
    config = getattr(spark_robot, config_name)()
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    agent = agent_type(
        config,
        dynamics_backend="simulator",
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
    )
    try:
        agent.reset({})
        initial = np.asarray(agent.get_feedback()["dof_pos_fbk"], dtype=float)
        wrist_motor_indices = np.arange(6, len(agent.arm_dof_indices), 7)
        directions = np.where(np.arange(len(wrist_motor_indices)) % 2 == 0, 1.0, -1.0)
        command = np.zeros(len(config.Control), dtype=float)
        for motor_index, direction in zip(wrist_motor_indices, directions):
            command[agent.arm_control_indices[motor_index]] = 0.5 * direction

        for _ in range(50):
            agent.send_control(command)
            agent.get_feedback()
        moving = np.asarray(agent.get_feedback()["dof_pos_fbk"], dtype=float)

        wrist_dofs = agent.arm_dof_indices[wrist_motor_indices]
        signed_motion = (moving[wrist_dofs] - initial[wrist_dofs]) * directions
        assert np.all(signed_motion > 0.4)
        assert np.all(signed_motion < 0.7)

        for _ in range(100):
            agent.send_control(np.zeros_like(command))
            agent.get_feedback()
        stopped_feedback = agent.get_feedback()
        stopped = np.asarray(stopped_feedback["dof_pos_fbk"], dtype=float)
        stopped_velocity = np.asarray(stopped_feedback["dof_vel_fbk"], dtype=float)
        assert np.max(np.abs(stopped[wrist_dofs] - moving[wrist_dofs])) < 0.01
        assert np.max(np.abs(stopped_velocity)) < 1.0e-3
    finally:
        agent.close_viewer()


@pytest.mark.parametrize(
    "config_name",
    (
        "KinovaGen3SingleArmDynamic1CollisionConfig",
        "KinovaGen3DualArmDynamic1Config",
        "KukaIIWA14SingleArmDynamic1CollisionConfig",
        "KukaIIWA14DualArmDynamic1Config",
    ),
)
def test_manipulator_affine_servo_request_stays_inside_engine_force_clamp(config_name):
    config = getattr(spark_robot, config_name)()
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    agent = agent_type(
        config,
        dynamics_backend="simulator",
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
    )
    try:
        agent.reset({})
        position = np.asarray(agent.get_feedback()["dof_pos_fbk"], dtype=float)
        ctrl = np.zeros(agent.model.nu, dtype=float)
        agent._apply_arm_ctrl(ctrl, position + 1.0, np.ones(agent.num_dof))

        for actuator_id in agent.arm_actuator_indices:
            actuator_id = int(actuator_id)
            if not bool(agent.model.actuator_forcelimited[actuator_id]):
                continue
            raw_force = float(
                agent.model.actuator_gainprm[actuator_id, 0] * ctrl[actuator_id]
                + agent.model.actuator_biasprm[actuator_id, 0]
                + agent.model.actuator_biasprm[actuator_id, 1]
                * agent.data.actuator_length[actuator_id]
                + agent.model.actuator_biasprm[actuator_id, 2]
                * agent.data.actuator_velocity[actuator_id]
            )
            low, high = agent.sim_force_limit_margin * agent.model.actuator_forcerange[actuator_id]
            assert float(low) - 1.0e-9 <= raw_force <= float(high) + 1.0e-9
    finally:
        agent.close_viewer()


@pytest.mark.parametrize(
    ("runner_module", "config_name"),
    (
        (
            "example.kinova_gen3.run_kinova_gen3_teleop",
            "KinovaGen3SingleArmDynamic1CollisionConfig",
        ),
        (
            "example.kinova_gen3.run_kinova_gen3_teleop",
            "KinovaGen3DualArmDynamic1CollisionConfig",
        ),
        (
            "example.kuka_iiwa14.run_kuka_iiwa14_teleop",
            "KukaIIWA14SingleArmDynamic1CollisionConfig",
        ),
        (
            "example.kuka_iiwa14.run_kuka_iiwa14_teleop",
            "KukaIIWA14DualArmDynamic1CollisionConfig",
        ),
    ),
)
def test_manipulator_simulator_teleop_converges_to_cartesian_home(runner_module, config_name):
    from spark_robot import get_agent_class_name

    runner = importlib.import_module(runner_module)

    kwargs = dict(
        robot_cfg=config_name,
        agent_cfg=get_agent_class_name(config_name, backend="mujoco"),
        dynamics_backend="simulator",
        safe_algo="bypass",
        safety_index="si1",
        minimum_distance=0.05,
        num_obstacle_task=0,
        num_obstacle_debug=0,
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
        max_num_steps=600,
        render_robot_collision_volumes=False,
        dt=None,
        control_decimation=None,
        device="cpu",
    )
    cfg = runner.PipelineConfig()
    for configure in (
        runner.config_pipeline,
        runner.config_task_module,
        runner.config_agent_module,
        runner.config_policy_module,
        runner.config_safety_module,
    ):
        cfg = configure(cfg, **kwargs)

    pipeline = runner.Pipeline(cfg)
    try:
        feedback, task_info = pipeline.env.reset()
        pipeline.policy.reset()
        action, action_info = pipeline.policy.act(feedback, task_info)
        for _ in range(600):
            feedback, task_info = pipeline.env.step(action, action_info)
            action, action_info = pipeline.policy.act(feedback, task_info)

        frames = pipeline.robot_kinematics.forward_kinematics(feedback["dof_pos_fbk"])
        goals_and_frames = [("right", pipeline.robot_cfg.Frames.R_ee)]
        if hasattr(pipeline.robot_cfg.Frames, "L_ee"):
            goals_and_frames.append(("left", pipeline.robot_cfg.Frames.L_ee))
        for side, frame_id in goals_and_frames:
            goal = np.asarray(task_info["goal_teleop"][side][0], dtype=float)
            actual = np.asarray(frames[frame_id], dtype=float)
            position_error = np.linalg.norm(goal[:3, 3] - actual[:3, 3])
            relative_rotation = goal[:3, :3] @ actual[:3, :3].T
            orientation_error = np.arccos(
                np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)
            )
            assert position_error < 0.02
            assert orientation_error < 0.05
    finally:
        pipeline.env.agent.close_viewer()


def test_fanuc_modeled_dynamics_applies_safe_control_instead_of_nominal_ik_target():
    """The modeled backend must not bypass the safety-filtered velocity command."""

    from spark_robot import get_agent_class_name

    runner = importlib.import_module("example.fanuc_lrmate200id.run_fanuc_lrmate200id_teleop")
    config_name = "FanucLRMate200iDSingleArmDynamic1CollisionConfig"
    kwargs = dict(
        robot_cfg=config_name,
        agent_cfg=get_agent_class_name(config_name, backend="mujoco"),
        dynamics_backend="model",
        safe_algo="rssa",
        safety_index=None,
        minimum_distance=0.05,
        num_obstacle_task=0,
        num_obstacle_debug=1,
        enable_viewer=False,
        enable_keyboard_control=False,
        enable_camera=False,
        real_time=False,
        max_num_steps=1,
        render_robot_collision_volumes=False,
        dt=None,
        control_decimation=None,
        device="cpu",
    )
    cfg = runner.PipelineConfig()
    for configure in (
        runner.config_pipeline,
        runner.config_task_module,
        runner.config_agent_module,
        runner.config_policy_module,
        runner.config_safety_module,
    ):
        cfg = configure(cfg, **kwargs)

    pipeline = runner.Pipeline(cfg)
    try:
        feedback, task_info = pipeline.env.reset()
        pipeline.policy.reset()
        initial_position = np.asarray(feedback["dof_pos_cmd"], dtype=float).copy()

        monitor = pipeline.policy.safety_monitor
        robot_frames = pipeline.robot_kinematics.forward_kinematics(initial_position)
        end_volume_frame = monitor.collision_vol_frame_ids[-1]
        end_volume_center = (feedback["robot_base_frame"] @ robot_frames[end_volume_frame])[:3, 3]
        obstacle_position = end_volume_center + np.array([0.08, 0.0, 0.0])
        pipeline.env.agent.obstacle_debug_frame[0, :3, 3] = obstacle_position
        pipeline.env.agent.obstacle_debug_frame_last[0, :3, 3] = obstacle_position

        feedback = pipeline.env.agent.get_feedback()
        pipeline.env.task.step(feedback)
        task_info = pipeline.env.task.get_info()
        action, action_info = pipeline.policy.act(feedback, task_info)

        assert action_info["trigger_safe"] is True
        assert np.linalg.norm(action_info["u_safe"] - action_info["u_ref"]) > 0.1

        expected_position = (
            initial_position + pipeline.robot_cfg.simulator_dynamics.control_period * action
        )
        nominal_target = np.asarray(action_info["dof_pos_target"], dtype=float)
        next_feedback, _ = pipeline.env.step(action, action_info)
        applied_position = np.asarray(next_feedback["dof_pos_cmd"], dtype=float)

        assert applied_position == pytest.approx(expected_position)
        assert not np.allclose(applied_position, nominal_target)
    finally:
        pipeline.env.agent.close_viewer()


@pytest.mark.parametrize(
    ("config_name", "expected_joint_count"),
    (
        ("KinovaGen3SingleArmDynamic1CollisionConfig", 8),
        ("KinovaGen3DualArmDynamic1Config", 16),
        ("KukaIIWA14SingleArmDynamic1CollisionConfig", 8),
        ("KukaIIWA14DualArmDynamic1Config", 16),
    ),
)
def test_robotiq_gripper_passive_linkage_is_damped(config_name, expected_joint_count):
    """Keep the equality-constrained Robotiq linkage from visibly chattering."""

    config = getattr(spark_robot, config_name)()
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    agent = agent_type(
        config,
        dynamics_backend="simulator",
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
    )
    try:
        passive_link_names = (
            "driver_joint",
            "follower_joint",
            "spring_link_joint",
            "coupler_joint",
        )
        gripper_dofs = []
        for joint_id in range(agent.model.njnt):
            name = mujoco.mj_id2name(agent.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or ""
            if not name.endswith(passive_link_names):
                continue
            gripper_dofs.append(int(agent.model.jnt_dofadr[joint_id]))

        assert len(gripper_dofs) == expected_joint_count
        assert np.min(agent.model.dof_damping[gripper_dofs]) >= 0.05
        assert np.min(agent.model.dof_armature[gripper_dofs]) >= 0.005
    finally:
        agent.close_viewer()


@pytest.mark.parametrize(
    "config_name",
    (
        "KukaIIWA14SingleArmDynamic1CollisionConfig",
        "KukaIIWA14DualArmDynamic1CollisionConfig",
    ),
)
def test_kuka_mujoco_arm_servos_do_not_retain_the_legacy_joint_one_gain(config_name):
    """A 500k position gain visibly destabilizes benchmark goal transitions."""

    config = getattr(spark_robot, config_name)()
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    agent = agent_type(
        config,
        dynamics_backend="simulator",
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
    )
    try:
        gains = agent.model.actuator_gainprm[agent.arm_actuator_indices, 0]
        damping = -agent.model.actuator_biasprm[agent.arm_actuator_indices, 2]
        assert np.max(gains) <= 2000.0
        assert np.max(damping) <= 200.0
    finally:
        agent.close_viewer()


@pytest.mark.parametrize(
    "config_name",
    (
        "AgiBotG1RightArmDynamic1Config",
        "GalaxeaR1LiteRightArmDynamic1CollisionConfig",
        "UnitreeG1RightArmWithHandDynamic1Config",
        "FanucLRMate200iDSingleArmDynamic1CollisionConfig",
        "KinovaGen3SingleArmDynamic1CollisionConfig",
        "KukaIIWA14SingleArmDynamic1CollisionConfig",
    ),
)
def test_mujoco_gripper_keyboard_override_is_shared_across_robot_families(config_name):
    config = getattr(spark_robot, config_name)()
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    agent = agent_type(
        config,
        dynamics_backend="simulator",
        enable_viewer=False,
        enable_keyboard_control=False,
        enable_hand_control=True,
        real_time=False,
    )
    try:
        agent.reset({})
        agent.get_feedback()
        command = np.zeros(len(config.Control), dtype=float)

        agent._set_gripper_debug_state("right", True)
        agent.send_control(command, action_info={"right_gripper_goal": False})
        assert agent.right_gripper_goal is True

        # A matching pipeline value acknowledges and releases the keyboard
        # override; subsequent pipeline commands become authoritative again.
        agent.send_control(command, action_info={"right_gripper_goal": True})
        agent.send_control(command, action_info={"right_gripper_goal": False})
        assert agent.right_gripper_goal is False
    finally:
        agent.close_viewer()


@pytest.mark.parametrize(
    ("config_name", "agent_name", "sides"),
    (
        (
            "FanucLRMate200iDSingleArmDynamic1CollisionConfig",
            "FanucLRMate200iDSingleArmAgent",
            ("right",),
        ),
        (
            "FanucLRMate200iDDualArmDynamic1CollisionConfig",
            "FanucLRMate200iDDualArmAgent",
            ("right", "left"),
        ),
    ),
)
def test_fanuc_mujoco_gripper_keys_physically_close_and_open_all_three_fingers(
    config_name, agent_name, sides
):
    config = getattr(spark_robot, config_name)()
    agent = getattr(spark_agent, agent_name)(
        config,
        dynamics_backend="simulator",
        enable_viewer=False,
        enable_keyboard_control=False,
        enable_hand_control=True,
        real_time=False,
    )
    try:
        agent.reset({})
        agent.get_feedback()
        command = np.zeros(len(config.Control), dtype=float)
        joint_positions = {}
        for side in sides:
            prefix = f"{side}/" if len(sides) > 1 else ""
            names = [
                f"{prefix}finger_{finger}_joint_{joint}"
                for finger in ("A", "B", "C")
                for joint in (1, 2, 3)
            ]
            joint_ids = [
                mujoco.mj_name2id(agent.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in names
            ]
            assert all(joint_id >= 0 for joint_id in joint_ids)
            joint_positions[side] = [agent.model.jnt_qposadr[joint_id] for joint_id in joint_ids]

        close_action = {f"{side}_gripper_goal": True for side in sides}
        for _ in range(200):
            agent.send_control(command, action_info=close_action)
            agent.get_feedback()
        for side, qpos_ids in joint_positions.items():
            closed = agent.data.qpos[qpos_ids].reshape(3, 3)
            assert np.all(closed[:, 0] > 0.9), side
            assert np.all(closed[:, 1] > 1.4), side
            assert np.all(closed[:, 2] < -0.75), side

        open_action = {f"{side}_gripper_goal": False for side in sides}
        for _ in range(200):
            agent.send_control(command, action_info=open_action)
            agent.get_feedback()
        for side, qpos_ids in joint_positions.items():
            assert np.max(np.abs(agent.data.qpos[qpos_ids])) < 0.08, side
        assert np.isfinite(agent.data.qpos).all()
        assert np.isfinite(agent.data.qvel).all()
    finally:
        agent.close_viewer()


@pytest.mark.parametrize(
    ("config_name", "agent_name"),
    (
        ("KinovaGen3SingleArmDynamic1CollisionConfig", "KinovaGen3SingleArmAgent"),
        ("KukaIIWA14SingleArmDynamic1CollisionConfig", "KukaIIWA14SingleArmAgent"),
    ),
)
def test_mujoco_bracket_keys_physically_close_and_open_2f85_gripper(config_name, agent_name):
    import glfw

    config = getattr(spark_robot, config_name)()
    agent = getattr(spark_agent, agent_name)(
        config,
        dynamics_backend="simulator",
        enable_viewer=False,
        enable_keyboard_control=True,
        enable_hand_control=True,
        real_time=False,
    )
    try:
        agent.reset({})
        agent.get_feedback()
        command = np.zeros(len(config.Control), dtype=float)
        joint_id = mujoco.mj_name2id(
            agent.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "right/right_driver_joint",
        )
        qpos_index = int(agent.model.jnt_qposadr[joint_id])

        agent._key_callback(glfw.KEY_RIGHT_BRACKET)
        for _ in range(80):
            agent.send_control(command, action_info={"right_gripper_goal": False})
            agent.get_feedback()
        closed_position = float(agent.data.qpos[qpos_index])

        agent._key_callback(glfw.KEY_LEFT_BRACKET)
        for _ in range(80):
            agent.send_control(command, action_info={"right_gripper_goal": True})
            agent.get_feedback()
        reopened_position = float(agent.data.qpos[qpos_index])

        assert closed_position > 0.5
        assert abs(reopened_position) < 0.05
    finally:
        agent.close_viewer()


@pytest.mark.parametrize(
    "config_name",
    (
        "KinovaGen3SingleArmDynamic1CollisionConfig",
        "KukaIIWA14SingleArmDynamic1CollisionConfig",
    ),
)
def test_modeled_dynamics_directly_updates_passive_2f85_linkage(config_name):
    config = getattr(spark_robot, config_name)()
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    agent = agent_type(
        config,
        dynamics_backend="model",
        enable_viewer=False,
        enable_keyboard_control=True,
        enable_hand_control=True,
        real_time=False,
    )
    try:
        agent.reset({})
        agent.get_feedback()
        command = np.zeros(len(config.Control), dtype=float)
        joint_id = mujoco.mj_name2id(
            agent.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "right/right_driver_joint",
        )
        qpos_index = int(agent.model.jnt_qposadr[joint_id])

        agent.send_control(command, action_info={"right_gripper_goal": True})
        assert agent.data.qpos[qpos_index] == pytest.approx(0.79)

        agent.send_control(command, action_info={"right_gripper_goal": False})
        assert agent.data.qpos[qpos_index] == pytest.approx(0.0)
    finally:
        agent.close_viewer()


def test_agibot_omnipicker_uses_coupled_open_and_closed_poses():
    config = spark_robot.AgiBotG1MobileBaseDynamic1Config()
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    agent = agent_type(
        config,
        dynamics_backend="model",
        enable_viewer=False,
        enable_keyboard_control=True,
        enable_hand_control=True,
        real_time=False,
    )
    try:
        agent.reset({})
        agent.get_feedback()
        command = np.zeros(len(config.Control), dtype=float)
        joint_names = (
            "narrow1_joint",
            "narrow3_joint",
            "narrow4_joint",
            "narrow2_joint",
            "wide1_joint",
            "wide3_joint",
            "wide4_joint",
            "wide2_joint",
        )

        def qpos():
            values = []
            for suffix in joint_names:
                joint_id = mujoco.mj_name2id(
                    agent.model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    f"right_{suffix}",
                )
                values.append(agent.data.qpos[agent.model.jnt_qposadr[joint_id]])
            return np.asarray(values)

        agent.send_control(command, action_info={"right_gripper_goal": True})
        np.testing.assert_allclose(qpos(), 0.0)
        agent.send_control(command, action_info={"right_gripper_goal": False})
        np.testing.assert_allclose(
            qpos(),
            [-0.60, -0.06, -0.15, -0.39, 0.60, -0.06, 0.15, 0.27],
        )
    finally:
        agent.close_viewer()


def test_agibot_simulator_gripper_slews_without_overshoot_or_contact_lock():
    config = spark_robot.AgiBotG1MobileBaseDynamic1Config()
    agent_type = getattr(spark_agent, config.agent_class_name("mujoco"))
    agent = agent_type(
        config,
        dynamics_backend="simulator",
        enable_viewer=False,
        enable_keyboard_control=True,
        enable_hand_control=True,
        real_time=False,
    )
    try:
        agent.reset({})
        agent.get_feedback()
        command = np.zeros(len(config.Control), dtype=float)
        joint_id = mujoco.mj_name2id(
            agent.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "right_narrow1_joint",
        )
        qpos_index = int(agent.model.jnt_qposadr[joint_id])

        for _ in range(100):
            agent.send_control(command, action_info={"right_gripper_goal": True})
            agent._post_control_processing()
            agent.get_feedback()
        assert abs(agent.data.qpos[qpos_index]) < 0.01
        assert agent.data.ncon == 0

        for _ in range(100):
            agent.send_control(command, action_info={"right_gripper_goal": False})
            agent._post_control_processing()
            agent.get_feedback()
        assert agent.data.qpos[qpos_index] == pytest.approx(-0.60, abs=0.01)
        assert agent.data.ncon == 0
    finally:
        agent.close_viewer()
