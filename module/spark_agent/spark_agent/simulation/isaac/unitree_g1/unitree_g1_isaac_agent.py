"""Isaac Sim agent for Unitree's 29-DoF G1 revision 1.0 model."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from spark_agent.dynamics import DynamicsExecutor
from spark_agent.simulation.isaac.isaac_agent import IsaacAgent, import_urdf_to_usd
from spark_robot import (
    SPARK_ROBOT_RESOURCE_DIR,
    RobotConfig,
    UnitreeG1WholeBodyDynamic1Config,
)
from spark_robot.unitree_g1.actuation import UNITREE_G1_GRIPPER_SPECS


G1_29DOF_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

G1_VIRTUAL_JOINT_NAMES = {
    "LinearX": "pelvis_x_joint",
    "LinearY": "pelvis_y_joint",
    "RotYaw": "pelvis_yaw_joint",
}


class UnitreeG1IsaacAgent(IsaacAgent):
    """Run the official G1 URDF through SPARK's single-agent contract.

    The class supports both the 36-value floating-base whole-body config and
    reduced fixed-base configs. Articulation joints are always resolved by
    name; no Isaac/USD joint-index ordering is assumed.
    """

    def __init__(
        self,
        robot_cfg: RobotConfig,
        *,
        urdf_path: str | None = None,
        usd_path: str | None = None,
        fixed_base: bool = False,
        spawn_position=(0.0, 0.0, 0.793),
        spawn_orientation=(1.0, 0.0, 0.0, 0.0),
        control_decimation: int | None = None,
        use_action_gains: bool = True,
        feedforward_torque_scale: float = 1.0,
        effort_limit: float | None = None,
        sim_stiffness_scale: float | None = None,
        sim_damping_scale: float | None = None,
        scale_action_gains: bool = False,
        sim_pd_backend: str = "explicit",
        hybrid_implicit_upper_body: bool = False,
        hybrid_arm_stiffness_scale: float = 1.0,
        hybrid_arm_damping_scale: float = 1.0,
        sim_position_error_limit: float | None = 0.35,
        gravity_compensation_scale: float = 0.0,
        coriolis_compensation_scale: float = 0.0,
        enable_actuator_diagnostics: bool = False,
        uncontrolled_joint_hold_kp: float = 300.0,
        uncontrolled_joint_hold_kd: float = 12.0,
        allow_self_collision: bool = False,
        asset_cache_dir: str | None = None,
        dynamics_backend: str = "model",
        use_sim_dynamics: bool | None = None,
        model_integrator: str | None = None,
        model_substeps: int = 1,
        enable_hand_control: bool = False,
        hand_kp: float | None = None,
        hand_kd: float | None = None,
        joint_armature_lower_body: float = 0.01,
        joint_armature_upper_body: float = 0.001,
        joint_stiffness: dict[str, float] | None = None,
        joint_damping: dict[str, float] | None = None,
        joint_armature: dict[str, float] | None = None,
        default_joint_positions: dict[str, float] | None = None,
        joint_friction: float | None = None,
        **kwargs,
    ) -> None:
        motors = tuple(getattr(robot_cfg, "MujocoMotors", ()))
        if not motors:
            raise ValueError(f"{robot_cfg.__class__.__name__} does not define any G1 MujocoMotors")
        available_by_motor_name = {
            "".join(part.capitalize() for part in name.removesuffix("_joint").split("_")): name
            for name in G1_29DOF_JOINT_NAMES
        }
        available_by_motor_name.update(G1_VIRTUAL_JOINT_NAMES)
        try:
            selected_joint_names = tuple(available_by_motor_name[motor.name] for motor in motors)
        except KeyError as exc:
            raise ValueError(
                f"G1 config motor {exc.args[0]!r} is absent from the official revision-1.0 model"
            ) from exc

        configured_urdf_path = getattr(robot_cfg, "isaac_model_path", None)
        if configured_urdf_path is None:
            configured_mjcf_path = Path(robot_cfg.mujoco_model_path)
            configured_urdf_path = Path("unitree_g1") / "urdf" / f"{configured_mjcf_path.stem}.urdf"
        source = Path(urdf_path or configured_urdf_path).expanduser()
        if not source.is_absolute():
            source = Path(SPARK_ROBOT_RESOURCE_DIR) / source
        resolved_usd = usd_path or import_urdf_to_usd(
            source,
            fix_base=fixed_base,
            allow_self_collision=allow_self_collision,
            cache_dir=asset_cache_dir,
        )

        self.fixed_base = bool(fixed_base)
        self._has_floating_base = bool(
            not self.fixed_base
            and all(
                hasattr(robot_cfg.DoFs, name)
                for name in (
                    "LinearX",
                    "LinearY",
                    "LinearZ",
                    "QuaternionW",
                    "QuaternionX",
                    "QuaternionY",
                    "QuaternionZ",
                )
            )
        )
        self.spawn_position = np.asarray(spawn_position, dtype=float).reshape(3)
        self.spawn_orientation = np.asarray(spawn_orientation, dtype=float).reshape(4)
        timing = robot_cfg.simulator_dynamics
        self.control_decimation = max(
            1,
            int(timing.control_decimation if control_decimation is None else control_decimation),
        )
        physics_dt = float(timing.physics_dt if kwargs.get("dt") is None else kwargs["dt"])
        kwargs["dt"] = physics_dt
        self.use_action_gains = bool(use_action_gains)
        self.feedforward_torque_scale = float(feedforward_torque_scale)
        self.effort_limit = None if effort_limit is None else abs(float(effort_limit))
        self.sim_stiffness_scale = float(
            getattr(robot_cfg, "IsaacSimStiffnessScale", 1.0)
            if sim_stiffness_scale is None
            else sim_stiffness_scale
        )
        self.sim_damping_scale = float(
            getattr(robot_cfg, "IsaacSimDampingScale", 2.0)
            if sim_damping_scale is None
            else sim_damping_scale
        )
        self.scale_action_gains = bool(scale_action_gains)
        self.sim_pd_backend = str(sim_pd_backend).lower()
        if self.sim_pd_backend not in {"explicit", "implicit"}:
            raise ValueError("sim_pd_backend must be 'explicit' or 'implicit'")
        self.hybrid_implicit_upper_body = bool(hybrid_implicit_upper_body)
        self.hybrid_arm_stiffness_scale = float(hybrid_arm_stiffness_scale)
        self.hybrid_arm_damping_scale = float(hybrid_arm_damping_scale)
        if self.hybrid_arm_stiffness_scale <= 0.0:
            raise ValueError("hybrid_arm_stiffness_scale must be positive")
        if self.hybrid_arm_damping_scale <= 0.0:
            raise ValueError("hybrid_arm_damping_scale must be positive")
        self.sim_position_error_limit = (
            None if sim_position_error_limit is None else abs(float(sim_position_error_limit))
        )
        self.gravity_compensation_scale = float(gravity_compensation_scale)
        self.coriolis_compensation_scale = float(coriolis_compensation_scale)
        self.enable_actuator_diagnostics = bool(enable_actuator_diagnostics)
        self.uncontrolled_joint_hold_kp = float(uncontrolled_joint_hold_kp)
        self.uncontrolled_joint_hold_kd = float(uncontrolled_joint_hold_kd)
        self.enable_hand_control = bool(enable_hand_control)
        self.body_robot_cfg = UnitreeG1WholeBodyDynamic1Config()
        self.hand_kp = float(UNITREE_G1_GRIPPER_SPECS["left"].kp if hand_kp is None else hand_kp)
        self.hand_kd = float(UNITREE_G1_GRIPPER_SPECS["left"].kd if hand_kd is None else hand_kd)
        self._motors = motors
        self.num_actuated = len(motors)
        self.g1_joint_names = selected_joint_names
        self.default_joint_positions = dict(default_joint_positions or {})
        self._has_complete_body = set(selected_joint_names) == set(G1_29DOF_JOINT_NAMES)
        self._upper_body_actuated_mask = np.asarray(
            [
                motor.name.startswith(
                    (
                        "Waist",
                        "LeftShoulder",
                        "LeftElbow",
                        "LeftWrist",
                        "RightShoulder",
                        "RightElbow",
                        "RightWrist",
                    )
                )
                for motor in motors
            ],
            dtype=bool,
        )
        self._arm_actuated_mask = np.asarray(
            [
                motor.name.startswith(
                    (
                        "LeftShoulder",
                        "LeftElbow",
                        "LeftWrist",
                        "RightShoulder",
                        "RightElbow",
                        "RightWrist",
                    )
                )
                for motor in motors
            ],
            dtype=bool,
        )
        self._joint_dof_indices = np.asarray(
            [
                int(robot_cfg.MujocoDoF_to_DoF[getattr(robot_cfg.MujocoDoFs, m.name)])
                for m in motors
            ],
            dtype=int,
        )
        self.kps = np.asarray([robot_cfg.MujocoMotorKps[m] for m in motors], dtype=float)
        self.kds = np.asarray([robot_cfg.MujocoMotorKds[m] for m in motors], dtype=float)
        urdf_joint_limits = {
            joint.get("name"): joint.find("limit") for joint in ET.parse(source).findall(".//joint")
        }
        self._sim_effort_limits = np.asarray(
            [
                abs(float(urdf_joint_limits[name].get("effort", "inf")))
                if urdf_joint_limits.get(name) is not None
                else np.inf
                for name in self.g1_joint_names
            ],
            dtype=float,
        )
        self._sim_joint_lower = np.asarray(
            [
                float(urdf_joint_limits[name].get("lower", "-inf"))
                if urdf_joint_limits.get(name) is not None
                else -np.inf
                for name in self.g1_joint_names
            ],
            dtype=float,
        )
        self._sim_joint_upper = np.asarray(
            [
                float(urdf_joint_limits[name].get("upper", "inf"))
                if urdf_joint_limits.get(name) is not None
                else np.inf
                for name in self.g1_joint_names
            ],
            dtype=float,
        )
        if self.effort_limit is not None:
            self._sim_effort_limits = np.minimum(self._sim_effort_limits, self.effort_limit)
        if use_sim_dynamics is not None:
            dynamics_backend = "simulator" if use_sim_dynamics else "model"
        if dynamics_backend not in {"model", "simulator"}:
            raise ValueError("dynamics_backend must be 'model' or 'simulator'")
        self.dynamics_backend = dynamics_backend
        self.dynamics_model = robot_cfg.create_dynamics_model()
        self.dynamics_executor = DynamicsExecutor(
            self.dynamics_model,
            dt=physics_dt * self.control_decimation,
            integrator=model_integrator,
            substeps=int(model_substeps),
        )

        kwargs.setdefault("ground_static_friction", 1.0)
        kwargs.setdefault("ground_dynamic_friction", 1.0)
        kwargs.setdefault("ground_restitution", 0.0)
        # SPARK imports the simulator-independent URDF rather than Unitree's
        # pre-authored USD.  Its point-foot contacts need the more robust
        # scalar PhysX solver budget used by the validated IsaacLab path.
        kwargs.setdefault("solver_position_iteration_count", 8)
        kwargs.setdefault("solver_velocity_iteration_count", 4)
        kwargs.setdefault("enabled_self_collisions", False)
        super().__init__(
            robot_cfg,
            usd_path=resolved_usd,
            joint_names=self.g1_joint_names,
            control_mode="effort",
            **kwargs,
        )
        if joint_stiffness:
            self.kps = np.asarray(
                [
                    joint_stiffness.get(name, value)
                    for name, value in zip(self.g1_joint_names, self.kps)
                ],
                dtype=float,
            )
        if joint_damping:
            self.kds = np.asarray(
                [
                    joint_damping.get(name, value)
                    for name, value in zip(self.g1_joint_names, self.kds)
                ],
                dtype=float,
            )
        self.use_sim_dynamics = self.dynamics_backend == "simulator"
        articulation_joint_names = tuple(self.articulation.dof_names)
        available_names = set(articulation_joint_names)
        missing_selected_joints = [
            name for name in self.g1_joint_names if name not in available_names
        ]
        if missing_selected_joints:
            raise ValueError(
                "Imported G1 articulation is missing configured joints "
                f"{missing_selected_joints}; asset: {source}"
            )
        self._all_articulation_joint_names = articulation_joint_names
        self._all_articulation_joint_indices = self._torch.as_tensor(
            list(range(len(articulation_joint_names))),
            device=self.device,
            dtype=self._torch.long,
        )
        lower_body_keywords = ("hip", "waist", "knee", "ankle", "pelvis", "torso")
        armatures = self._torch.as_tensor(
            [
                joint_armature.get(
                    name,
                    joint_armature_lower_body
                    if any(keyword in name for keyword in lower_body_keywords)
                    else joint_armature_upper_body,
                )
                if joint_armature
                else (
                    joint_armature_lower_body
                    if any(keyword in name for keyword in lower_body_keywords)
                    else joint_armature_upper_body
                )
                for name in articulation_joint_names
            ],
            device=self.device,
            dtype=self._torch.float32,
        )
        self.articulation._articulation_view.set_armatures(armatures)
        if joint_friction is not None:
            friction = self._torch.full(
                (1, len(articulation_joint_names)),
                float(joint_friction),
                device=self.device,
                dtype=self._torch.float32,
            )
            self.articulation._articulation_view.set_friction_coefficients(
                friction,
                joint_indices=self._all_articulation_joint_indices,
            )
        if self.sim_pd_backend == "explicit":
            # Pre-authored Isaac assets can ship with active joint drives,
            # whereas SPARK's imported URDF deliberately creates zero-gain
            # drives.  Explicit effort-PD must see the same plant in both
            # cases; otherwise the asset drive silently fights SPARK's torque.
            zero_gains = self._torch.zeros(
                len(articulation_joint_names),
                device=self.device,
                dtype=self._torch.float32,
            )
            self.articulation._articulation_view.set_gains(
                kps=zero_gains,
                kds=zero_gains,
                joint_indices=self._all_articulation_joint_indices,
            )
        selected_names = set(self.g1_joint_names)
        self._gripper_specs = {
            side: spec
            for side, spec in UNITREE_G1_GRIPPER_SPECS.items()
            if all(name in available_names for name in spec.joint_names)
        }
        auxiliary_names = (
            {name for spec in self._gripper_specs.values() for name in spec.joint_names}
            if self.enable_hand_control
            else set()
        )
        self._uncontrolled_joint_names = tuple(
            name
            for name in articulation_joint_names
            if name not in selected_names and name not in auxiliary_names
        )
        self._uncontrolled_joint_indices = self._torch.as_tensor(
            [articulation_joint_names.index(name) for name in self._uncontrolled_joint_names],
            device=self.device,
            dtype=self._torch.long,
        )
        self._all_default_joint_positions = np.zeros(len(articulation_joint_names), dtype=float)
        selected_defaults = self._default_joint_positions()
        selected_default_by_name = dict(zip(self.g1_joint_names, selected_defaults))
        gripper_default_by_name = {
            name: value
            for spec in self._gripper_specs.values()
            for name, value in zip(spec.joint_names, spec.open_position)
        }
        for index, name in enumerate(articulation_joint_names):
            self._all_default_joint_positions[index] = selected_default_by_name.get(
                name, gripper_default_by_name.get(name, 0.0)
            )
        self._uncontrolled_joint_targets = np.asarray(
            [
                self._all_default_joint_positions[articulation_joint_names.index(name)]
                for name in self._uncontrolled_joint_names
            ],
            dtype=float,
        )
        self._sim_target_pos = selected_defaults.copy()
        self._sim_target_vel = np.zeros(self.num_actuated, dtype=float)
        self._sim_feedforward = np.zeros(self.num_actuated, dtype=float)
        self._sim_kps = self.sim_stiffness_scale * self.kps
        self._sim_kds = self.sim_damping_scale * self.kds
        self._sim_direct_effort = None
        self._upper_body_gravity_compensation = False
        self._init_gripper_control(urdf_joint_limits)

    def _init_gripper_control(self, urdf_joint_limits) -> None:
        self._gripper_joint_indices = {}
        self._gripper_targets = {}
        self._gripper_effort_limits = {}
        for side, spec in self._gripper_specs.items():
            indices = [self._all_articulation_joint_names.index(name) for name in spec.joint_names]
            self._gripper_joint_indices[side] = self._torch.as_tensor(
                indices, device=self.device, dtype=self._torch.long
            )
            self._gripper_targets[side] = spec.target(False)
            self._gripper_effort_limits[side] = np.asarray(
                [
                    abs(float(urdf_joint_limits[name].get("effort", "inf")))
                    if urdf_joint_limits.get(name) is not None
                    else np.inf
                    for name in spec.joint_names
                ],
                dtype=float,
            )

    def _update_gripper_targets(self, action_info: dict) -> None:
        if not self.enable_hand_control:
            return
        for side, spec in self._gripper_specs.items():
            control_key = f"{side}_gripper_control"
            goal_key = f"{side}_gripper_goal"
            if control_key in action_info:
                target = spec.clamp(action_info[control_key])
                closed = bool(np.linalg.norm(target - spec.target(False)) > 1e-6)
            else:
                fallback = action_info.get("gripper_goal") if side == "right" else None
                goal = action_info.get(goal_key, fallback)
                if side in self._keyboard_gripper_goal_override:
                    keyboard_goal = bool(self._keyboard_gripper_goal_override[side])
                    if goal is not None and bool(goal) == keyboard_goal:
                        self._keyboard_gripper_goal_override.pop(side, None)
                    else:
                        goal = keyboard_goal
                if goal is None:
                    goal = getattr(self, goal_key)
                closed = bool(goal)
                target = spec.target(closed)
            setattr(self, goal_key, closed)
            setattr(self, f"{side}_gripper_debug_state", closed)
            self._gripper_targets[side] = spec.clamp(target)

    def _apply_gripper_efforts(self) -> None:
        if not self.enable_hand_control:
            return
        for side, indices in self._gripper_joint_indices.items():
            current_pos = self.articulation.get_joint_positions(joint_indices=indices)
            current_vel = self.articulation.get_joint_velocities(joint_indices=indices)
            target = self._torch.as_tensor(
                self._gripper_targets[side], device=self.device, dtype=self._torch.float32
            )
            effort = self.hand_kp * (target - current_pos) - self.hand_kd * current_vel
            limit = self._torch.as_tensor(
                self._gripper_effort_limits[side],
                device=self.device,
                dtype=self._torch.float32,
            )
            effort = self._torch.maximum(self._torch.minimum(effort, limit), -limit)
            self.articulation.set_joint_efforts(effort, joint_indices=indices)

    def _write_gripper_configuration(self, max_step: float | None = None) -> None:
        if not self.enable_hand_control:
            return
        for side, indices in self._gripper_joint_indices.items():
            target = self._torch.as_tensor(
                self._gripper_targets[side], device=self.device, dtype=self._torch.float32
            )
            if max_step is not None:
                current = self.articulation.get_joint_positions(joint_indices=indices)
                target = current + self._torch.clamp(
                    target - current,
                    -abs(float(max_step)),
                    abs(float(max_step)),
                )
            self.articulation.set_joint_positions(target, joint_indices=indices)
            self.articulation.set_joint_velocities(
                self._torch.zeros_like(target), joint_indices=indices
            )

    def _root_pose(self):
        # ``SingleArticulation.get_world_pose`` queries its wrapper Xform and
        # can remain at the authored spawn pose while PhysX advances the
        # floating articulation.  WBT must observe the authoritative tensor
        # state or its gravity projection and base controller operate on a
        # permanently upright, stationary base.
        position, orientation = self.articulation._articulation_view.get_world_poses()
        position = position[0]
        orientation = orientation[0]
        if hasattr(position, "detach"):
            position = position.detach().cpu().numpy()
        if hasattr(orientation, "detach"):
            orientation = orientation.detach().cpu().numpy()
        return np.asarray(position, dtype=float), np.asarray(orientation, dtype=float)

    def _root_velocities(self):
        view = self.articulation._articulation_view
        linear = view.get_linear_velocities()[0]
        angular = view.get_angular_velocities()[0]
        if hasattr(linear, "detach"):
            linear = linear.detach().cpu().numpy()
        if hasattr(angular, "detach"):
            angular = angular.detach().cpu().numpy()
        return np.asarray(linear, dtype=float), np.asarray(angular, dtype=float)

    def _set_root_state(self, position, orientation) -> None:
        """Write the floating root through PhysX's authoritative tensor view.

        ``SingleArticulation.set_world_pose`` targets the wrapper Xform.  That
        happens to coincide with the articulation root for URDFs imported by
        SPARK, but it does not for nested USD assets such as Unitree's original
        Isaac model.  Reading and writing the same tensor view also prevents a
        reset from leaving the rendered prim and simulated root out of sync.
        """
        view = self.articulation._articulation_view
        position = self._torch.as_tensor(
            position, device=self.device, dtype=self._torch.float32
        ).reshape(1, 3)
        orientation = self._torch.as_tensor(
            orientation, device=self.device, dtype=self._torch.float32
        ).reshape(1, 4)
        view.set_world_poses(positions=position, orientations=orientation)
        # The GPU PhysX pipeline does not support the split angular-velocity
        # writer.  Write the complete [linear, angular] root twist atomically;
        # this is supported by both CPU and GPU articulation views.
        zeros = self._torch.zeros((1, 6), device=self.device, dtype=self._torch.float32)
        view.set_velocities(zeros)

    def _default_joint_positions(self) -> np.ndarray:
        defaults = np.asarray(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs], dtype=float
        )
        selected = defaults[self._joint_dof_indices]
        return np.asarray(
            [
                self.default_joint_positions.get(name, value)
                for name, value in zip(self.g1_joint_names, selected)
            ],
            dtype=float,
        )

    def reset(self, agent_reset_info=None, **kwargs) -> None:
        reset_info = dict(agent_reset_info or {})
        joint_pos = self._default_joint_positions()
        requested = reset_info.get("reset_dof_pos", reset_info.get("state"))
        if requested is not None:
            requested = np.asarray(requested, dtype=float).reshape(-1)
            if requested.size == self.num_dof:
                joint_pos = requested[self._joint_dof_indices]
            elif requested.size == self.num_actuated:
                joint_pos = requested
            else:
                raise ValueError(
                    f"G1 reset state must contain {self.num_dof} config values or "
                    f"{self.num_actuated} joint values"
                )

        self._set_root_state(self.spawn_position, self.spawn_orientation)
        all_joint_pos = self._all_default_joint_positions.copy()
        selected_by_name = dict(zip(self.g1_joint_names, joint_pos))
        for index, name in enumerate(self._all_articulation_joint_names):
            if name in selected_by_name:
                all_joint_pos[index] = selected_by_name[name]
        joint_pos_tensor = self._torch.as_tensor(
            all_joint_pos, device=self.device, dtype=self._torch.float32
        )
        joint_vel_tensor = self._torch.zeros(
            len(self._all_articulation_joint_names),
            device=self.device,
            dtype=self._torch.float32,
        )
        self.articulation.set_joint_positions(
            joint_pos_tensor, joint_indices=self._all_articulation_joint_indices
        )
        self.articulation.set_joint_velocities(
            joint_vel_tensor, joint_indices=self._all_articulation_joint_indices
        )
        self._set_root_state(self.spawn_position, self.spawn_orientation)
        self.articulation.set_joint_efforts(
            joint_vel_tensor, joint_indices=self._all_articulation_joint_indices
        )
        self.dof_pos_cmd = None
        self.dof_vel_cmd = None
        self.dof_acc_cmd = None
        self.last_control = np.zeros(self.num_actuated, dtype=float)
        self.step_index = 0
        self.time = float(reset_info.get("time", 0.0))
        self.sim.step(render=False)
        # The reset step initializes PhysX/Fabric but also applies gravity once.
        # Restore the exact requested state so both simulators start from the
        # same configuration and zero velocity.
        self.articulation.set_joint_positions(
            joint_pos_tensor, joint_indices=self._all_articulation_joint_indices
        )
        self.articulation.set_joint_velocities(
            joint_vel_tensor, joint_indices=self._all_articulation_joint_indices
        )
        self._refresh_feedback()
        initial_full_state = self.robot_cfg.compose_state_from_dof(
            self.dof_pos_fbk, self.dof_vel_fbk
        )
        self.dynamics_executor.reset(
            self.dynamics_model.project_state(initial_full_state),
            time=self.time,
            seed=reset_info.get("seed"),
        )
        self._sim_target_pos = joint_pos.copy()
        self._sim_target_vel = np.zeros(self.num_actuated, dtype=float)
        self._sim_feedforward = np.zeros(self.num_actuated, dtype=float)
        self._sim_kps = self.sim_stiffness_scale * self.kps
        self._sim_kds = self.sim_damping_scale * self.kds
        self._sim_direct_effort = None
        self.left_gripper_goal = False
        self.right_gripper_goal = False
        self.left_gripper_debug_state = False
        self.right_gripper_debug_state = False
        self._keyboard_gripper_goal_override.clear()
        for side, spec in self._gripper_specs.items():
            self._gripper_targets[side] = spec.target(False)

    def _target_from_action_info(self, action_info: dict):
        current_pos = self.dof_pos_fbk[self._joint_dof_indices]
        current_vel = self.dof_vel_fbk[self._joint_dof_indices]
        if "target_actuated_pos" in action_info:
            target_pos = np.asarray(action_info["target_actuated_pos"], dtype=float).reshape(
                self.num_actuated
            )
        elif "target_dof_pos" in action_info:
            value = np.asarray(action_info["target_dof_pos"], dtype=float).reshape(-1)
            target_pos = (
                value[self._joint_dof_indices]
                if value.size == self.num_dof
                else value.reshape(self.num_actuated)
            )
        elif "sol_acc" in action_info:
            acceleration = np.asarray(action_info["sol_acc"], dtype=float).reshape(-1)
            acceleration = acceleration[-self.num_actuated :]
            target_pos = current_pos + 0.5 * acceleration * self.dt**2
        else:
            return None

        if "target_actuated_vel" in action_info:
            target_vel = np.asarray(action_info["target_actuated_vel"], dtype=float).reshape(
                self.num_actuated
            )
        elif "target_dof_vel" in action_info:
            value = np.asarray(action_info["target_dof_vel"], dtype=float).reshape(-1)
            target_vel = (
                value[self._joint_dof_indices]
                if value.size == self.num_dof
                else value.reshape(self.num_actuated)
            )
        elif "sol_acc" in action_info:
            target_vel = (
                current_vel
                + np.asarray(action_info["sol_acc"], dtype=float).reshape(-1)[-self.num_actuated :]
                * self.dt
            )
        else:
            target_vel = np.zeros(self.num_actuated, dtype=float)
        return target_pos, target_vel

    def _integrated_target_from_command(self, command, action_info):
        """Mirror MuJoCo's velocity-model-to-PD-target control semantics."""
        command_state = self.robot_cfg.compose_state_from_dof(self.dof_pos_cmd, self.dof_vel_cmd)
        model_state = self.dynamics_model.project_state(command_state)
        model_control = self.dynamics_model.project_control(command)
        next_model_state = self.dynamics_executor.step(
            model_control,
            state=model_state,
            parameters=action_info.get("dynamics_parameters"),
            exogenous=action_info.get("dynamics_exogenous"),
        )
        next_full_state = self.dynamics_model.merge_state(next_model_state, into=command_state)
        self.dof_pos_cmd = np.asarray(
            self.robot_cfg.decompose_state_to_dof_pos(next_full_state), dtype=float
        )
        self.dof_vel_cmd = np.asarray(
            self.robot_cfg.decompose_state_to_dof_vel(next_full_state), dtype=float
        )
        target_pos = self._bounded_sim_target(self.dof_pos_cmd[self._joint_dof_indices])
        self.dof_pos_cmd[self._joint_dof_indices] = target_pos
        return (
            target_pos,
            self.dof_vel_cmd[self._joint_dof_indices],
        )

    def _bounded_sim_target(self, target_pos: np.ndarray) -> np.ndarray:
        """Apply the same hard joint limits and bounded PD error to every target path."""
        current_pos = self.dof_pos_fbk[self._joint_dof_indices]
        bounded = np.clip(
            np.asarray(target_pos, dtype=float).reshape(self.num_actuated),
            self._sim_joint_lower,
            self._sim_joint_upper,
        )
        if self.sim_position_error_limit is not None:
            bounded = current_pos + np.clip(
                bounded - current_pos,
                -self.sim_position_error_limit,
                self.sim_position_error_limit,
            )
        return bounded

    def _inverse_dynamics_feedforward(self):
        feedforward = self._torch.zeros(
            self.num_actuated, device=self.device, dtype=self._torch.float32
        )
        view = self.articulation._articulation_view
        if self.gravity_compensation_scale or self._upper_body_gravity_compensation:
            gravity = view.get_generalized_gravity_forces()
            if gravity is not None:
                # PhysX prepends six floating-base generalized coordinates to
                # the articulation joint forces.  Articulation joint indices
                # therefore cannot index this tensor directly.
                # The deprecated Isaac Sim 6 articulation wrapper reports a
                # tensor width that does not expose this prefix in ``shape``;
                # nevertheless floating-base generalized-force indices are
                # shifted by six, as documented by PhysX.
                # Isaac 6 may return either generalized forces with the six
                # floating-base entries prepended or joint-only forces. Infer
                # the layout from the actual tensor width instead of assuming
                # one representation for every imported asset.
                base_offset = (
                    6
                    if self._has_floating_base
                    and gravity.shape[-1] >= len(self._all_articulation_joint_names) + 6
                    else 0
                )
                gravity = gravity[0, self.joint_indices + base_offset]
                if self.gravity_compensation_scale:
                    feedforward = feedforward + self.gravity_compensation_scale * gravity
                if self._upper_body_gravity_compensation:
                    mask = self._torch.as_tensor(
                        self._upper_body_actuated_mask,
                        device=self.device,
                        dtype=self._torch.bool,
                    )
                    feedforward = feedforward + self._torch.where(
                        mask, gravity, self._torch.zeros_like(gravity)
                    )
        if self.coriolis_compensation_scale:
            coriolis = view.get_coriolis_and_centrifugal_forces()
            if coriolis is not None:
                base_offset = (
                    6
                    if self._has_floating_base
                    and coriolis.shape[-1] >= len(self._all_articulation_joint_names) + 6
                    else 0
                )
                coriolis = coriolis[0, self.joint_indices + base_offset]
                feedforward = feedforward + self.coriolis_compensation_scale * coriolis
        return feedforward

    def _apply_simulator_efforts(self) -> None:
        if self._sim_direct_effort is None and self.sim_pd_backend == "implicit":
            from isaacsim.core.utils.types import ArticulationAction

            target_pos = self._torch.as_tensor(
                self._sim_target_pos, device=self.device, dtype=self._torch.float32
            )
            target_vel = self._torch.as_tensor(
                self._sim_target_vel, device=self.device, dtype=self._torch.float32
            )
            kps = self._torch.as_tensor(
                self._sim_kps, device=self.device, dtype=self._torch.float32
            )
            kds = self._torch.as_tensor(
                self._sim_kds, device=self.device, dtype=self._torch.float32
            )
            feedforward = self._torch.as_tensor(
                self._sim_feedforward, device=self.device, dtype=self._torch.float32
            )
            feedforward = (
                self.feedforward_torque_scale * feedforward + self._inverse_dynamics_feedforward()
            )
            self.articulation._articulation_view.set_gains(
                kps=kps, kds=kds, joint_indices=self.joint_indices
            )
            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=target_pos,
                    joint_velocities=target_vel,
                    joint_efforts=feedforward,
                    joint_indices=self.joint_indices,
                )
            )
            if self.enable_actuator_diagnostics:
                current_pos = self.articulation.get_joint_positions(
                    joint_indices=self.joint_indices
                )
                current_vel = self.articulation.get_joint_velocities(
                    joint_indices=self.joint_indices
                )
                requested = (
                    feedforward
                    + kps * (target_pos - current_pos)
                    + kds * (target_vel - current_vel)
                )
                limits = self._torch.as_tensor(
                    self._sim_effort_limits,
                    device=self.device,
                    dtype=self._torch.float32,
                )
                self._last_requested_effort_tensor = requested.detach().clone()
                self._last_applied_effort_tensor = (
                    self._torch.clamp(requested, -limits, limits).detach().clone()
                )
            if self._uncontrolled_joint_names:
                hold_pos = self.articulation.get_joint_positions(
                    joint_indices=self._uncontrolled_joint_indices
                )
                hold_vel = self.articulation.get_joint_velocities(
                    joint_indices=self._uncontrolled_joint_indices
                )
                hold_target = self._torch.as_tensor(
                    self._uncontrolled_joint_targets,
                    device=self.device,
                    dtype=self._torch.float32,
                )
                self.articulation.set_joint_efforts(
                    self.uncontrolled_joint_hold_kp * (hold_target - hold_pos)
                    - self.uncontrolled_joint_hold_kd * hold_vel,
                    joint_indices=self._uncontrolled_joint_indices,
                )
            self._apply_gripper_efforts()
            return

        if self._sim_direct_effort is None:
            current_pos = self.articulation.get_joint_positions(joint_indices=self.joint_indices)
            current_vel = self.articulation.get_joint_velocities(joint_indices=self.joint_indices)
            target_pos = self._torch.as_tensor(
                self._sim_target_pos, device=self.device, dtype=self._torch.float32
            )
            target_vel = self._torch.as_tensor(
                self._sim_target_vel, device=self.device, dtype=self._torch.float32
            )
            feedforward = self._torch.as_tensor(
                self._sim_feedforward, device=self.device, dtype=self._torch.float32
            )
            kps = self._torch.as_tensor(
                self._sim_kps, device=self.device, dtype=self._torch.float32
            )
            kds = self._torch.as_tensor(
                self._sim_kds, device=self.device, dtype=self._torch.float32
            )
            effort = (
                self.feedforward_torque_scale * feedforward
                + kps * (target_pos - current_pos)
                + kds * (target_vel - current_vel)
                + self._inverse_dynamics_feedforward()
            )
        else:
            effort = self._torch.as_tensor(
                self._sim_direct_effort,
                device=self.device,
                dtype=self._torch.float32,
            )
        requested_effort = effort
        effort_limits = self._torch.as_tensor(
            self._sim_effort_limits,
            device=self.device,
            dtype=self._torch.float32,
        )
        effort = self._torch.maximum(self._torch.minimum(effort, effort_limits), -effort_limits)
        if self.hybrid_implicit_upper_body and self._sim_direct_effort is None:
            upper_mask = self._torch.as_tensor(
                self._upper_body_actuated_mask,
                device=self.device,
                dtype=self._torch.bool,
            )
            effort = self._torch.where(upper_mask, self._torch.zeros_like(effort), effort)
        if self.enable_actuator_diagnostics:
            self._last_requested_effort_tensor = requested_effort.detach().clone()
            self._last_applied_effort_tensor = effort.detach().clone()
        self.articulation.set_joint_efforts(
            effort,
            joint_indices=self.joint_indices,
        )
        if self.hybrid_implicit_upper_body and self._sim_direct_effort is None:
            # The WBT gait depends on explicit effort-PD, but the official
            # asset's low-inertia wrists are numerically underdamped with the
            # same sampled controller.  Let PhysX integrate the arm/waist PD
            # drive implicitly while retaining explicit lower-body efforts.
            from isaacsim.core.utils.types import ArticulationAction

            upper_mask = self._torch.as_tensor(
                self._upper_body_actuated_mask,
                device=self.device,
                dtype=self._torch.bool,
            )
            upper_indices = self.joint_indices[upper_mask]
            implicit_kps = kps[upper_mask].clone()
            implicit_kds = kds[upper_mask].clone()
            arm_within_upper = self._torch.as_tensor(
                self._arm_actuated_mask[self._upper_body_actuated_mask],
                device=self.device,
                dtype=self._torch.bool,
            )
            implicit_kps = self._torch.where(
                arm_within_upper,
                implicit_kps * self.hybrid_arm_stiffness_scale,
                implicit_kps,
            )
            implicit_kds = self._torch.where(
                arm_within_upper,
                implicit_kds * self.hybrid_arm_damping_scale,
                implicit_kds,
            )
            self.articulation._articulation_view.set_gains(
                kps=implicit_kps,
                kds=implicit_kds,
                joint_indices=upper_indices,
            )
            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=target_pos[upper_mask],
                    joint_velocities=target_vel[upper_mask],
                    joint_indices=upper_indices,
                )
            )

        if self._uncontrolled_joint_names:
            hold_pos = self.articulation.get_joint_positions(
                joint_indices=self._uncontrolled_joint_indices
            )
            hold_vel = self.articulation.get_joint_velocities(
                joint_indices=self._uncontrolled_joint_indices
            )
            hold_target = self._torch.as_tensor(
                self._uncontrolled_joint_targets,
                device=self.device,
                dtype=self._torch.float32,
            )
            hold_effort = (
                self.uncontrolled_joint_hold_kp * (hold_target - hold_pos)
                - self.uncontrolled_joint_hold_kd * hold_vel
            )
            self.articulation.set_joint_efforts(
                hold_effort,
                joint_indices=self._uncontrolled_joint_indices,
            )
        self._apply_gripper_efforts()

    def _enforce_uncontrolled_joint_locks(self) -> None:
        """Make reduced SPARK configs match their reduced MuJoCo mechanism.

        The imported URDF always contains all 29 G1 joints, while dual-arm,
        right-arm, and fixed-base SPARK configurations intentionally expose a
        subset and their MJCF files omit the remaining joint freedoms. PhysX
        must therefore treat those remaining URDF joints as locked rather than
        merely uncontrolled.
        """
        if not self._uncontrolled_joint_names:
            return
        targets = self._torch.as_tensor(
            self._uncontrolled_joint_targets,
            device=self.device,
            dtype=self._torch.float32,
        )
        zero_velocity = self._torch.zeros_like(targets)
        self.articulation.set_joint_positions(
            targets, joint_indices=self._uncontrolled_joint_indices
        )
        self.articulation.set_joint_velocities(
            zero_velocity, joint_indices=self._uncontrolled_joint_indices
        )

    def _send_control_sim_dynamics(self, command, **kwargs) -> None:
        action_info = dict(kwargs.get("action_info") or {})
        self._update_gripper_targets(action_info)
        self._upper_body_gravity_compensation = bool(
            action_info.get("upper_body_gravity_compensation", False)
        )
        targets = self._target_from_action_info(action_info)
        direct_effort = (
            bool(action_info.get("direct_effort", False))
            or str(action_info.get("control_mode", "")).lower() == "effort"
        )
        if direct_effort:
            self._sim_direct_effort = np.asarray(command, dtype=float).reshape(self.num_actuated)
            self._sim_target_pos = self.dof_pos_fbk[self._joint_dof_indices].copy()
            self._sim_target_vel = self.dof_vel_fbk[self._joint_dof_indices].copy()
            self._sim_feedforward = np.zeros(self.num_actuated, dtype=float)
        elif targets is None:
            self._sim_direct_effort = None
            target_pos, target_vel = self._integrated_target_from_command(command, action_info)
            self._sim_target_pos = target_pos.copy()
            self._sim_target_vel = target_vel.copy()
            self._sim_feedforward = np.zeros(self.num_actuated, dtype=float)
            self._sim_kps = self.sim_stiffness_scale * self.kps
            self._sim_kds = self.sim_damping_scale * self.kds
        else:
            self._sim_direct_effort = None
            target_pos, target_vel = targets
            target_pos = self._bounded_sim_target(target_pos)
            action_kps = action_info.get("target_actuated_kps", action_info.get("motor_kps"))
            action_kds = action_info.get("target_actuated_kds", action_info.get("motor_kds"))
            use_action_kps = self.use_action_gains and action_kps is not None
            use_action_kds = self.use_action_gains and action_kds is not None
            kps = np.asarray(self.kps if action_kps is None else action_kps, dtype=float).reshape(
                -1
            )
            kds = np.asarray(self.kds if action_kds is None else action_kds, dtype=float).reshape(
                -1
            )
            if kps.size != self.num_actuated:
                kps = self.kps
                use_action_kps = False
            if kds.size != self.num_actuated:
                kds = self.kds
                use_action_kds = False
            feedforward = np.asarray(
                action_info.get(
                    "feedforward_torque",
                    action_info.get("sol_torque", np.zeros(self.num_actuated)),
                ),
                dtype=float,
            ).reshape(-1)
            if feedforward.size != self.num_actuated:
                feedforward = np.zeros(self.num_actuated, dtype=float)
            self.dof_pos_cmd = self.dof_pos_fbk.copy()
            self.dof_vel_cmd = self.dof_vel_fbk.copy()
            self.dof_pos_cmd[self._joint_dof_indices] = target_pos
            self.dof_vel_cmd[self._joint_dof_indices] = target_vel
            self._sim_target_pos = target_pos.copy()
            self._sim_target_vel = target_vel.copy()
            self._sim_feedforward = feedforward.copy()
            self._sim_kps = (
                self.sim_stiffness_scale * kps
                if self.scale_action_gains or not use_action_kps
                else kps.copy()
            )
            self._sim_kds = (
                self.sim_damping_scale * kds
                if self.scale_action_gains or not use_action_kds
                else kds.copy()
            )
        self._apply_simulator_efforts()
        self.last_control = np.asarray(command, dtype=float).reshape(-1).copy()

    def _send_control_modeled_dynamics(self, command, **kwargs) -> None:
        """Advance configured kinematics/dynamics and write qpos into Isaac.

        PhysX is not stepped in this mode. Isaac acts as a viewer for the state
        produced by the same robot-config dynamics abstraction used by the
        MuJoCo agent's modeled-dynamics path.
        """
        action_info = dict(kwargs.get("action_info") or {})
        self._update_gripper_targets(action_info)
        control = self.dynamics_model.project_control(command)
        next_model_state = self.dynamics_executor.step(
            control,
            parameters=action_info.get("dynamics_parameters"),
            exogenous=action_info.get("dynamics_exogenous"),
        )
        current_full_state = self.robot_cfg.compose_state_from_dof(
            self.dof_pos_fbk, self.dof_vel_fbk
        )
        next_full_state = self.dynamics_model.merge_state(next_model_state, into=current_full_state)
        self.dof_pos_cmd = np.asarray(
            self.robot_cfg.decompose_state_to_dof_pos(next_full_state), dtype=float
        )
        self.dof_vel_cmd = np.asarray(
            self.robot_cfg.decompose_state_to_dof_vel(next_full_state), dtype=float
        )
        self.dof_acc_cmd = np.zeros(self.num_dof, dtype=float)
        self.last_control = np.asarray(command, dtype=float).reshape(-1).copy()
        self._write_modeled_configuration()

    def _write_modeled_configuration(self) -> None:
        joint_pos = self.dof_pos_cmd[self._joint_dof_indices]
        # Floating-base qpos has seven coordinates while qvel has six. Body
        # joint velocity indices are consequently shifted by one.
        velocity_offset = self.num_dof - self.dof_vel_cmd.size
        joint_vel_indices = self._joint_dof_indices - velocity_offset
        joint_vel = self.dof_vel_cmd[joint_vel_indices]
        self.articulation.set_joint_positions(
            self._torch.as_tensor(joint_pos, device=self.device, dtype=self._torch.float32),
            joint_indices=self.joint_indices,
        )
        self.articulation.set_joint_velocities(
            self._torch.as_tensor(joint_vel, device=self.device, dtype=self._torch.float32),
            joint_indices=self.joint_indices,
        )
        if self._has_floating_base:
            self._set_root_state(self.dof_pos_cmd[:3], self.dof_pos_cmd[3:7])
        self._write_gripper_configuration()

    def _post_control_processing(self, **kwargs) -> None:
        if self.use_sim_dynamics:
            for substep in range(self.control_decimation):
                # Recompute PD and lock-joint efforts from the latest PhysX
                # state, matching MuJoCo's per-substep controller update.
                self._apply_simulator_efforts()
                # Physics must run at every decimation substep, but presenting
                # all of those substeps makes Isaac spend up to N times more
                # time in Kit than MuJoCo and makes commands look artificially
                # slow. Present only the state observed by the next policy
                # call, after the final physics substep.
                render = (
                    self.render_enabled
                    and self.render_on_step
                    and substep == self.control_decimation - 1
                )
                self.sim.step(render=render)
                self._enforce_uncontrolled_joint_locks()
        else:
            # The model state is authoritative in viewer-only mode. No PhysX
            # integration is performed, but rendered links still need an
            # explicit articulation FK update after direct joint-state writes.
            if self.render_enabled and self.render_on_step:
                self._synchronize_kinematic_articulation()
                self.sim.render()
        self.step_index += 1
        self.time = (
            self.time + self.dt * self.control_decimation
            if self.use_sim_dynamics
            else self.dynamics_executor.time
        )
        self._refresh_feedback()
        if self.enable_camera and self.render_enabled and self.render_on_step:
            self._capture_camera_feedback()

    def _refresh_feedback(self) -> None:
        if not getattr(self, "use_sim_dynamics", True) and self.dof_pos_cmd is not None:
            self.dof_pos_fbk = self.dof_pos_cmd.copy()
            self.dof_vel_fbk = self.dof_vel_cmd.copy()
            self.dof_acc_cmd = np.zeros(self.num_dof, dtype=float)
            return
        joint_pos = self.articulation.get_joint_positions(joint_indices=self.joint_indices)
        joint_vel = self.articulation.get_joint_velocities(joint_indices=self.joint_indices)
        joint_pos = joint_pos.detach().cpu().numpy().astype(float, copy=True)
        joint_vel = joint_vel.detach().cpu().numpy().astype(float, copy=True)
        position, orientation = self._root_pose()
        linear, angular = self._root_velocities()
        self.dof_pos_fbk = np.asarray(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs], dtype=float
        )
        self.dof_vel_fbk = np.zeros(self.num_dof, dtype=float)
        self.dof_pos_fbk[self._joint_dof_indices] = joint_pos
        self.dof_vel_fbk[self._joint_dof_indices] = joint_vel
        if self._has_floating_base:
            self.dof_pos_fbk[:3] = position
            self.dof_pos_fbk[3:7] = orientation
            self.dof_vel_fbk[:3] = linear
            # The existing whole-body config stores 36 velocity slots although
            # a floating base has only six physical velocity coordinates.
            self.dof_vel_fbk[3:6] = angular
        self.dof_pos_cmd = self.dof_pos_fbk.copy() if self.dof_pos_cmd is None else self.dof_pos_cmd
        self.dof_vel_cmd = self.dof_vel_fbk.copy() if self.dof_vel_cmd is None else self.dof_vel_cmd
        self.dof_acc_cmd = np.zeros(self.num_dof, dtype=float)

    def get_feedback(self):
        feedback = super().get_feedback()
        feedback["qpos_fbk"] = self.dof_pos_fbk.copy()
        feedback["qvel_fbk"] = self.dof_vel_fbk.copy()
        if self._has_complete_body:
            body_qpos = np.asarray(
                [self.body_robot_cfg.DefaultDoFVal[dof] for dof in self.body_robot_cfg.DoFs],
                dtype=float,
            )
            body_qvel = np.zeros(body_qpos.size - 1, dtype=float)
            body_qpos[:7] = self.dof_pos_fbk[:7]
            body_qvel[:6] = self.dof_vel_fbk[:6]
            # PhysX reports root angular velocity in world coordinates, while
            # WBT was trained on the body-frame IMU gyroscope (and MuJoCo's
            # floating-base convention supplies the equivalent local value).
            # Keep translational qvel in world coordinates, but rotate omega
            # into the pelvis frame before exposing canonical WBT feedback.
            w, x, y, z = body_qpos[3:7]
            rotation_world_from_body = np.array(
                [
                    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
                ],
                dtype=float,
            )
            body_qvel[3:6] = rotation_world_from_body.T @ body_qvel[3:6]
            for motor in self._motors:
                model_dof = self.robot_cfg.MujocoDoF_to_DoF[
                    getattr(self.robot_cfg.MujocoDoFs, motor.name)
                ]
                body_dof = self.body_robot_cfg.MujocoDoF_to_DoF[
                    getattr(self.body_robot_cfg.MujocoDoFs, motor.name)
                ]
                model_index = int(model_dof)
                body_index = int(body_dof)
                body_qpos[body_index] = self.dof_pos_fbk[model_index]
                body_qvel[body_index - 1] = self.dof_vel_fbk[model_index]
            feedback["body_qpos_fbk"] = body_qpos
            feedback["body_qvel_fbk"] = body_qvel
        else:
            feedback["body_qpos_fbk"] = self.dof_pos_fbk.copy()
            feedback["body_qvel_fbk"] = self.dof_vel_fbk.copy()
        for side, indices in self._gripper_joint_indices.items():
            position = self.articulation.get_joint_positions(joint_indices=indices)
            velocity = self.articulation.get_joint_velocities(joint_indices=indices)
            feedback[f"{side}_gripper_pos_fbk"] = (
                position.detach().cpu().numpy().astype(float, copy=True)
            )
            feedback[f"{side}_gripper_vel_fbk"] = (
                velocity.detach().cpu().numpy().astype(float, copy=True)
            )
        return feedback
