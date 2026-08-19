import numpy as np
from scipy.spatial.transform import Rotation

from spark_policy.control.whole_body.unitree_g1.sport import UnitreeG1SportPolicy
from spark_policy.control.whole_body.unitree_g1.wbt.config import (
    WBT_MOTOR_KDS,
    WBT_MOTOR_KPS,
)
from spark_policy.control.whole_body.unitree_g1.wbt import UnitreeG1WBTPolicy
from spark_policy.composed_policy.unitree_g1.wbt_safe import UnitreeG1WBTSafePolicy


class UnitreeG1SportExecutorAdapter:
    """Present the Sport network through the learned-locomotion executor port."""

    def __init__(self, robot_cfg, robot_kinematics=None, **kwargs):
        self.use_wbt_motor_gains = bool(kwargs.get("use_wbt_motor_gains", False))
        self.control_dt = float(kwargs.get("control_dt", 0.02))
        self.policy = UnitreeG1SportPolicy(robot_cfg)
        if not np.isclose(self.policy.config.control_dt, self.control_dt):
            raise ValueError(
                "Sport policy control_dt must match its executor control_dt: "
                f"{self.policy.config.control_dt} != {self.control_dt}"
            )
        # Reuse WBT's backend-independent Cartesian IK/target filtering only;
        # Sport remains the sole lower-body recurrent policy.
        self.upper_body_policy = UnitreeG1WBTPolicy(robot_cfg, robot_kinematics, **kwargs)
        default_dof = np.asarray(
            [robot_cfg.DefaultDoFVal[dof] for dof in robot_cfg.DoFs],
            dtype=float,
        )
        self.default_upper_body_target = default_dof[-17:].copy()

    def reset(self, context=None):
        self.policy.reset(context)
        self.upper_body_policy.reset(context)

    @staticmethod
    def _zero_base_command():
        return np.zeros(5, dtype=float)

    def act(self, agent_feedback, task_info):
        feedback = dict(agent_feedback)
        if "root_pose_w" not in feedback:
            base = np.asarray(feedback["robot_base_frame"], dtype=float)
            feedback["root_pose_w"] = np.concatenate(
                [base[:3, 3], Rotation.from_matrix(base[:3, :3]).as_quat()]
            )
        if "body_joint_pos" not in feedback:
            qpos = np.asarray(
                feedback.get("body_qpos_fbk", feedback["dof_pos_fbk"]),
                dtype=float,
            ).reshape(-1)
            feedback["body_joint_pos"] = qpos[-29:]
        if "body_joint_vel" not in feedback:
            qvel = np.asarray(
                feedback.get("body_qvel_fbk", feedback["dof_vel_fbk"]),
                dtype=float,
            ).reshape(-1)
            feedback["body_joint_vel"] = qvel[-29:]
        if "root_velocity_w" not in feedback:
            qvel = np.asarray(
                feedback.get("body_qvel_fbk", feedback["dof_vel_fbk"]),
                dtype=float,
            ).reshape(-1)
            feedback["root_velocity_w"] = np.pad(qvel[:6], (0, max(0, 6 - qvel[:6].size)))[:6]
        feedback.setdefault(
            "root_angular_velocity_b",
            np.asarray(feedback["root_velocity_w"], dtype=float)[3:6],
        )

        command = np.asarray(task_info.get("wbt_command", np.zeros(5)), dtype=float).reshape(-1)
        sport_task = {**task_info, "sport_command": command[:3].copy()}

        # Sport controls only the lower 12 joints. Use the same nominal arm
        # goal solver and filtering as WBT, including safety overrides.
        upper_target, ik_success = self.upper_body_policy.upper_body_target(feedback, task_info)
        sport_task["upper_body_target"] = upper_target

        control, info = self.policy.act(feedback, sport_task)
        info["ik_success"] = bool(ik_success)
        info["target_upper_body_pos"] = np.asarray(upper_target, dtype=float).copy()
        info["policy_control_dt"] = self.control_dt
        # Sport and WBT intentionally share one Isaac actuator experiment.
        # Supplying gains explicitly also prevents the Isaac agent from taking
        # its fallback path, which applies an additional damping scale.
        if self.use_wbt_motor_gains:
            info["motor_kps"] = WBT_MOTOR_KPS.copy()
            info["motor_kds"] = WBT_MOTOR_KDS.copy()
        return control, info


class UnitreeG1SportSafePolicy(UnitreeG1WBTSafePolicy):
    """Legged tracker -> safety filter -> Unitree Sport executor."""
