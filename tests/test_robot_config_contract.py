from __future__ import annotations

import inspect
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import spark_agent
import spark_robot
from spark_robot import RobotConfig


def _default_constructible_configs():
    for name in sorted(dir(spark_robot)):
        config_type = getattr(spark_robot, name)
        if (
            not inspect.isclass(config_type)
            or config_type is RobotConfig
            or not issubclass(config_type, RobotConfig)
            or inspect.isabstract(config_type)
        ):
            continue
        if any(
            parameter.default is inspect.Parameter.empty
            and parameter.kind not in (parameter.VAR_POSITIONAL, parameter.VAR_KEYWORD)
            for parameter in inspect.signature(config_type).parameters.values()
        ):
            continue
        yield name, config_type()


def test_robot_configs_declare_valid_backends_and_assets():
    resource_root = Path(spark_robot.SPARK_ROBOT_RESOURCE_DIR)
    for name, config in _default_constructible_configs():
        with_backend = dict(config.agent_class_names)
        assert with_backend, f"{name} has no backend mapping"
        for agent_name in with_backend.values():
            assert agent_name in dir(spark_agent), f"{name}: unknown agent {agent_name}"

        if config.model_only:
            assert set(with_backend) == {"dynamics"}
            continue

        assert "mujoco" in with_backend
        assert "isaac" in with_backend
        assert (resource_root / config.mujoco_model_path).is_file()
        assert hasattr(spark_robot, config.kinematics_class_name)


def test_robot_configs_expose_structured_backend_capabilities():
    for name, config in _default_constructible_configs():
        capabilities = config.backend_capabilities()
        assert set(capabilities) == set(config.agent_class_names), name
        for backend, capability in capabilities.items():
            assert capability.agent_class_name == config.agent_class_names[backend], name

        if not config.model_only:
            isaac_capability = capabilities["isaac"]
            assert isaac_capability.single_environment, name
            assert isaac_capability.batched, name
            assert isaac_capability.tensor_io, name
            assert isaac_capability.rendering, name

        simulator = config.simulator_dynamics
        assert simulator.physics_dt > 0.0, name
        assert simulator.control_decimation > 0, name
        assert simulator.control_period > 0.0, name
        assert simulator.state_owner == "simulator", name
        assert not simulator.allows_state_projection, name

        isaac_spec = config.isaac_articulation
        if isaac_spec is not None:
            assert config.agent_class_names.get("isaac", "").endswith("IsaacAgent"), name
            isaac_capability = capabilities["isaac"]
            assert isaac_capability.single_environment, name
            assert isaac_capability.batched, name
            assert isaac_capability.tensor_io, name
            assert isaac_capability.rendering, name
            assert isaac_capability.validation == "validated", name
            urdf_path = Path(spark_robot.SPARK_ROBOT_RESOURCE_DIR) / isaac_spec.urdf_path
            assert urdf_path.is_file(), (name, urdf_path)
            urdf_joints = {
                joint.attrib["name"]
                for joint in ET.parse(urdf_path).findall(".//joint")
                if joint.attrib.get("type") != "fixed"
            }
            declared_joints = (
                {
                    f"{instance.prefix}_{joint_name}"
                    for instance in isaac_spec.instances
                    for joint_name in urdf_joints
                }
                if isaac_spec.instances
                else urdf_joints
            )
            if isaac_spec.planar_base is not None:
                declared_joints = declared_joints | set(isaac_spec.planar_base.joint_names)
            assert set(isaac_spec.joint_names) <= declared_joints, name
            for gripper in isaac_spec.grippers:
                assert set(gripper.joint_names) <= declared_joints, (name, gripper.side)
            assert len(isaac_spec.joint_names) == len(config.DoFs), name


def test_agibot_physical_configs_share_the_qualified_simulator_schedule():
    configs = [
        config
        for name, config in _default_constructible_configs()
        if name.startswith("AgiBotG1") and not config.model_only
    ]
    assert configs
    for config in configs:
        assert config.simulator_dynamics.physics_dt == 0.005
        assert config.simulator_dynamics.control_decimation == 4
        assert config.simulator_dynamics.control_period == 0.02


def test_unitree_configs_share_the_real_rate_simulator_schedule():
    configs = [
        config for name, config in _default_constructible_configs() if name.startswith("UnitreeG1")
    ]
    assert configs
    for config in configs:
        assert config.simulator_dynamics.physics_dt == 0.005
        assert config.simulator_dynamics.control_decimation == 4
        assert config.simulator_dynamics.control_period == 0.02


def test_r1lite_benchmark_configs_share_the_qualified_simulator_schedule():
    configs = [
        config
        for name, config in _default_constructible_configs()
        if name.startswith(
            (
                "GalaxeaR1LiteRightArm",
                "GalaxeaR1LiteDualArm",
                "GalaxeaR1LiteMobileBase",
            )
        )
    ]
    assert configs
    for config in configs:
        assert config.simulator_dynamics.physics_dt == 0.005
        assert config.simulator_dynamics.control_decimation == 4
        assert config.simulator_dynamics.control_period == 0.02


def test_r1lite_fixed_base_uses_shared_qualified_schedule():
    configs = [
        config
        for name, config in _default_constructible_configs()
        if name.startswith("GalaxeaR1LiteFixedBase")
    ]
    assert configs
    for config in configs:
        assert config.simulator_dynamics.physics_dt == 0.005
        assert config.simulator_dynamics.control_decimation == 4
        assert config.simulator_dynamics.control_period == 0.02


def test_r1lite_fixed_and_mobile_isaac_joints_share_drive_gains():
    fixed = spark_robot.GalaxeaR1LiteFixedBaseDynamic1Config().isaac_articulation
    mobile = spark_robot.GalaxeaR1LiteMobileBaseDynamic1Config().isaac_articulation
    assert fixed.stiffness == mobile.stiffness
    assert fixed.damping == mobile.damping


def test_r1lite_fixed_collision_frames_have_shared_link_local_grounding():
    for config_type in (
        spark_robot.GalaxeaR1LiteFixedBaseDynamic1CollisionConfig,
        spark_robot.GalaxeaR1LiteFixedBaseDynamic2CollisionConfig,
    ):
        config = config_type()
        assert set(config.CollisionVolBodyNames) == set(config.CollisionVol)
        assert set(config.CollisionVolLocalOffsets) == set(config.CollisionVol)


def test_robot_config_control_and_mapping_contracts():
    for name, config in _default_constructible_configs():
        dofs = set(config.DoFs)
        controls = set(config.Control)
        normal = set(config.NormalControl)
        weak = set(config.WeakControl)
        delicate = set(config.DelicateControl)

        assert set(config.DefaultDoFVal) == dofs, name
        assert set(config.ControlLimit) == controls, name
        assert normal | weak | delicate == controls, name
        assert not normal & weak, name
        assert not normal & delicate, name
        assert not weak & delicate, name

        assert set(config.MujocoDoF_to_DoF) == set(config.MujocoDoFs), name
        assert set(config.MujocoDoF_to_DoF.values()) <= dofs, name
        assert set(config.DoF_to_MujocoDoF) <= dofs, name
        assert set(config.MujocoMotor_to_Control) == set(config.MujocoMotors), name
        assert set(config.MujocoMotor_to_Control.values()) <= controls, name
        assert set(config.RealMotor_to_Control) <= set(config.RealMotors), name
        assert set(config.RealMotor_to_Control.values()) <= controls, name

        if "real" in config.agent_class_names:
            assert set(config.RealMotorPosLimit) == set(config.RealMotors), name


def test_declared_linear_dynamics_supply_matrices():
    for name, config in _default_constructible_configs():
        model = config.create_dynamics_model()
        state = np.zeros(model.state_dim)
        control = np.zeros(model.control_dim)
        next_state = model.step(
            state,
            control,
            0.01,
            getattr(model, "default_integrator", "Euler"),
        )
        assert next_state.shape == state.shape, name
        assert np.isfinite(next_state).all(), name
        if model.is_linear:
            state_matrix, control_matrix = model.continuous_matrices()
            assert state_matrix.shape == (model.state_dim, model.state_dim), name
            assert control_matrix.shape == (model.state_dim, model.control_dim), name
