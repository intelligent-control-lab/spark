from spark_policy.core.policy import BasePolicy
from spark_robot import RobotKinematics, RobotConfig
import numpy as np
from scipy.spatial.transform import Rotation as R


def wrap_to_pi(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


UPPER_BODY_DOF_NAMES = (
    "WaistYaw",
    "WaistRoll",
    "WaistPitch",
    "LeftShoulderPitch",
    "LeftShoulderRoll",
    "LeftShoulderYaw",
    "LeftElbow",
    "LeftWristRoll",
    "LeftWristPitch",
    "LeftWristYaw",
    "RightShoulderPitch",
    "RightShoulderRoll",
    "RightShoulderYaw",
    "RightElbow",
    "RightWristRoll",
    "RightWristPitch",
    "RightWristYaw",
)


class BasePIDPolicy(BasePolicy):
    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics, **kwargs) -> None:
        super().__init__(robot_cfg, robot_kinematics)
        self.position_kp = np.asarray(kwargs.get("position_kp", 1.0), dtype=float)
        self.velocity_kd = np.asarray(kwargs.get("velocity_kd", 0.1), dtype=float)
        self._base_goal_neutral_z = None
        self._base_goal_neutral_pitch = None
        self._lift_body_neutral = None
        self._body_pitch_neutral = None

    def _set_named_dof_gain(self, gains, name: str, value: float):
        dof = self.robot_cfg.DoFs.__members__.get(name)
        if dof is not None and int(dof) < len(gains):
            gains[int(dof)] = value

    def _clip_named_dof_target(self, dof_pos_target, name: str):
        dof = self.robot_cfg.DoFs.__members__.get(name)
        real_motor = getattr(self.robot_cfg, "RealMotors", None)
        real_limits = getattr(self.robot_cfg, "RealMotorPosLimit", {})
        if dof is None or real_motor is None or name not in real_motor.__members__:
            return

        motor = real_motor.__members__[name]
        if motor in real_limits:
            low, high = real_limits[motor]
            dof_pos_target[dof] = np.clip(dof_pos_target[dof], low, high)

    def _teleop_upper_body_mode(self, task_info: dict) -> str:
        mode = str(task_info.get("teleop_upper_body_mode", "cartesian")).strip().lower()
        aliases = {
            "cart": "cartesian",
            "ee": "cartesian",
            "ik": "cartesian",
            "pos": "joint",
            "position": "joint",
            "joint_pos": "joint",
            "joint_position": "joint",
        }
        mode = aliases.get(mode, mode)
        return mode if mode in ("cartesian", "joint", "auto") else "cartesian"

    def _apply_teleop_joint_upper_body_target(self, dof_pos_target, task_info: dict) -> bool:
        target = task_info.get("teleop_joint_upper_body_target", None)
        if target is None:
            return False

        target = np.asarray(target, dtype=float).reshape(-1)
        if target.shape[0] != len(UPPER_BODY_DOF_NAMES):
            raise ValueError(
                f"teleop_joint_upper_body_target must have 17 entries, got {target.shape[0]}"
            )

        mask = task_info.get("teleop_joint_upper_body_target_mask", None)
        if mask is None:
            mask = np.ones(len(UPPER_BODY_DOF_NAMES), dtype=bool)
        else:
            mask = np.asarray(mask, dtype=bool).reshape(-1)
            if mask.shape[0] != len(UPPER_BODY_DOF_NAMES):
                raise ValueError(
                    f"teleop_joint_upper_body_target_mask must have 17 entries, got {mask.shape[0]}"
                )

        applied = False
        dofs = self.robot_cfg.DoFs.__members__
        for idx, name in enumerate(UPPER_BODY_DOF_NAMES):
            if not mask[idx] or name not in dofs:
                continue
            dof = dofs[name]
            if int(dof) < len(dof_pos_target):
                dof_pos_target[dof] = target[idx]
                self._clip_named_dof_target(dof_pos_target, name)
                applied = True
        return applied

    def tracking_pos_with_vel(
        self, dof_pos_target, dof_vel_target, dof_pos_current, dof_vel_current
    ):
        K_p = np.broadcast_to(self.position_kp, (len(self.robot_cfg.Control),)).copy()
        K_d = np.broadcast_to(self.velocity_kd, (len(self.robot_cfg.Control),)).copy()
        self._set_named_dof_gain(K_p, "LiftBody", 5.0)
        self._set_named_dof_gain(K_d, "LiftBody", 0.2)
        self._set_named_dof_gain(K_p, "BodyPitch", 4.0)
        self._set_named_dof_gain(K_d, "BodyPitch", 0.2)
        dof_vel_nominal = K_p * (dof_pos_target - dof_pos_current) + K_d * (
            dof_vel_target - dof_vel_current
        )
        nominal_control = (
            np.linalg.pinv(self.robot_cfg.dynamics_g(dof_pos_current)) @ dof_vel_nominal
        )
        return nominal_control

    def tracking_pos_with_acc(
        self, dof_pos_target, dof_vel_target, dof_pos_current, dof_vel_current
    ):
        K_p_vel = 10.0 * np.ones(len(self.robot_cfg.DoFs))
        K_d_vel = 0.4 * np.ones(len(self.robot_cfg.DoFs))
        K_p_acc = 10.0 * np.ones(len(self.robot_cfg.DoFs))
        K_d_acc = 5.0 * np.ones(len(self.robot_cfg.DoFs))
        nominal_dof_vel = K_p_vel * (dof_pos_target - dof_pos_current) - K_d_vel * dof_vel_current
        nominal_dof_acc = K_p_acc * (dof_pos_target - dof_pos_current) - K_d_acc * dof_vel_current
        nominal_control = np.linalg.pinv(
            self.robot_cfg.dynamics_g(np.concatenate((dof_pos_current, dof_vel_current)))
        ) @ np.concatenate((nominal_dof_vel, nominal_dof_acc))
        return nominal_control

    def act(self, agent_feedback: dict, task_info: dict):
        info = {}

        dof_pos_cmd = agent_feedback["dof_pos_cmd"]
        dof_vel_cmd = agent_feedback["dof_vel_cmd"]
        dof_pos_fbk = agent_feedback["dof_pos_fbk"]
        dof_vel_fbk = agent_feedback["dof_vel_fbk"]
        goal_teleop = task_info["goal_teleop"]
        robot_base_frame = agent_feedback["robot_base_frame"]

        dof_pos_target = dof_pos_fbk.copy()

        try:
            teleop_mode = self._teleop_upper_body_mode(task_info)
            joint_target_applied = False
            arm_ik_owns_shared_torso = False
            if task_info["arm_goal_enable"]:
                if teleop_mode in ("joint", "auto"):
                    joint_target_applied = self._apply_teleop_joint_upper_body_target(
                        dof_pos_target, task_info
                    )
                if not joint_target_applied and teleop_mode != "joint":
                    ik_kwargs = {}
                    if task_info.get("arm_goal_position_only", False):
                        count = 2 if "left" in goal_teleop else 1
                        ik_kwargs["target_options"] = [
                            {"orientation_mask": (False, False, False)} for _ in range(count)
                        ]
                    if "left" not in goal_teleop:
                        dof_pos_target, _ = self.robot_kinematics.inverse_kinematics(
                            [np.linalg.inv(robot_base_frame) @ goal_teleop["right"][0]],
                            dof_pos_fbk,
                            **ik_kwargs,
                        )
                    else:
                        arm_goals_base = [
                            np.linalg.inv(robot_base_frame) @ goal_teleop["right"][0],
                            np.linalg.inv(robot_base_frame) @ goal_teleop["left"][0],
                        ]
                        active_arm_goal = task_info.get("active_arm_goal")
                        active_arm_ik = getattr(
                            self.robot_kinematics,
                            "inverse_kinematics_active_arm",
                            None,
                        )
                        if active_arm_goal in ("right", "left") and callable(active_arm_ik):
                            dof_pos_target, _ = active_arm_ik(
                                arm_goals_base,
                                dof_pos_fbk,
                                active_side=active_arm_goal,
                                **ik_kwargs,
                            )
                            arm_ik_owns_shared_torso = True
                        else:
                            dof_pos_target, _ = self.robot_kinematics.inverse_kinematics(
                                arm_goals_base,
                                dof_pos_fbk,
                                **ik_kwargs,
                            )
            if task_info["base_goal_enable"]:
                rot = R.from_matrix(goal_teleop["base"][0][:3, :3])
                euler = rot.as_euler("xyz")
                dofs = self.robot_cfg.DoFs.__members__
                # position targets
                if "LinearX" in dofs:
                    dof_pos_target[self.robot_cfg.DoFs.LinearX] = goal_teleop["base"][0][0, 3]
                if "LinearY" in dofs:
                    dof_pos_target[self.robot_cfg.DoFs.LinearY] = goal_teleop["base"][0][1, 3]
                if "RotYaw" in dofs:
                    # current yaw (DO NOT wrap; keep unwrapped if fbk is unwrapped)
                    yaw_cur = float(dof_pos_fbk[self.robot_cfg.DoFs.RotYaw])
                    yaw_cur = wrap_to_pi(yaw_cur)
                    dof_pos_fbk[self.robot_cfg.DoFs.RotYaw] = yaw_cur

                    # wrapped target from rotation matrix
                    yaw_tgt_wrapped = wrap_to_pi(float(euler[2]))

                    # yaw target near current yaw (may become > pi or < -pi)
                    dyaw_short = wrap_to_pi(yaw_tgt_wrapped - wrap_to_pi(yaw_cur))

                    dof_pos_target[self.robot_cfg.DoFs.RotYaw] = (
                        dof_pos_fbk[self.robot_cfg.DoFs.RotYaw] + dyaw_short
                    )

                if "LinearZ" in self.robot_cfg.DoFs.__members__:
                    dof_pos_target[self.robot_cfg.DoFs.LinearZ] = goal_teleop["base"][0][2, 3]
                elif "LiftBody" in self.robot_cfg.DoFs.__members__:
                    if self._base_goal_neutral_z is None:
                        self._base_goal_neutral_z = float(goal_teleop["base"][0][2, 3])
                        self._lift_body_neutral = float(dof_pos_fbk[self.robot_cfg.DoFs.LiftBody])
                    if not arm_ik_owns_shared_torso:
                        dof_pos_target[self.robot_cfg.DoFs.LiftBody] = (
                            self._lift_body_neutral
                            + float(goal_teleop["base"][0][2, 3])
                            - self._base_goal_neutral_z
                        )
                        self._clip_named_dof_target(dof_pos_target, "LiftBody")
                if "RotPitch" in self.robot_cfg.DoFs.__members__:
                    dof_pos_target[self.robot_cfg.DoFs.RotPitch] = float(euler[1])
                elif "BodyPitch" in self.robot_cfg.DoFs.__members__:
                    if self._base_goal_neutral_pitch is None:
                        self._base_goal_neutral_pitch = float(euler[1])
                        self._body_pitch_neutral = float(dof_pos_fbk[self.robot_cfg.DoFs.BodyPitch])
                    if not arm_ik_owns_shared_torso:
                        dof_pos_target[self.robot_cfg.DoFs.BodyPitch] = (
                            self._body_pitch_neutral
                            + float(euler[1])
                            - self._base_goal_neutral_pitch
                        )
                        self._clip_named_dof_target(dof_pos_target, "BodyPitch")

            if self.robot_cfg.dynamics_order == 1:
                dof_vel_target = np.zeros_like(dof_vel_fbk)
                dof_pos_current = dof_pos_fbk
                dof_vel_current = dof_vel_fbk
                control = self.tracking_pos_with_vel(
                    dof_pos_target, dof_vel_target, dof_pos_current, dof_vel_current
                )
            if self.robot_cfg.dynamics_order == 2:
                dof_vel_target = np.zeros_like(dof_vel_cmd)
                # NOTE: Using feedback control may case instability. Be cautious about using fbk pos and vel for real robot control; consider using cmd pos and vel instead for more stable open-loop control.
                dof_pos_current = dof_pos_cmd
                dof_vel_current = dof_vel_cmd
                control = self.tracking_pos_with_acc(
                    dof_pos_target, dof_vel_target, dof_pos_current, dof_vel_current
                )

            info["ik_success"] = bool(joint_target_applied or teleop_mode != "joint")
            info["teleop_upper_body_mode"] = teleop_mode
            info["teleop_joint_target_applied"] = bool(joint_target_applied)
        except Exception:
            control = np.zeros_like(dof_pos_fbk)
            info["ik_success"] = False
            dof_pos_target = dof_pos_fbk.copy()
            # raise e

        info["dof_pos_target"] = dof_pos_target.copy()
        if "left_gripper_goal" in task_info["goal_teleop"]:
            info["left_gripper_goal"] = task_info["goal_teleop"]["left_gripper_goal"]
        if "right_gripper_goal" in task_info["goal_teleop"]:
            info["right_gripper_goal"] = task_info["goal_teleop"]["right_gripper_goal"]

        for control_id in self.robot_cfg.Control:
            control[control_id] = np.clip(
                control[control_id],
                -self.robot_cfg.ControlLimit[control_id],
                self.robot_cfg.ControlLimit[control_id],
            )

        return control, info
