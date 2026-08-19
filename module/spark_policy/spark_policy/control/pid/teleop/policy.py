from spark_policy.control.pid.base import BasePIDPolicy
from spark_robot import RobotKinematics, RobotConfig
import numpy as np


def _wrap_to_pi(angle: float) -> float:
    return (float(angle) + np.pi) % (2.0 * np.pi) - np.pi


class TeleopPIDPolicy(BasePIDPolicy):
    def __init__(
        self,
        robot_cfg: RobotConfig,
        robot_kinematics: RobotKinematics,
        **kwargs,
    ) -> None:
        kwargs.setdefault("velocity_kd", 0.0)
        super().__init__(robot_cfg, robot_kinematics, **kwargs)
        # Continuous PID is appropriate for holonomic plants, but a learned
        # legged executor cannot realize millimetre-scale corrections.  These
        # optional thresholds let a policy composition latch a reached SE(2)
        # goal instead of repeatedly starting an opposite gait after natural
        # standing drift.  Zero defaults preserve the legacy PID behavior.
        self.base_goal_xy_deadband = max(0.0, float(kwargs.get("base_goal_xy_deadband", 0.0)))
        self.base_goal_yaw_deadband = max(0.0, float(kwargs.get("base_goal_yaw_deadband", 0.0)))
        self.base_goal_resume_distance = max(
            self.base_goal_xy_deadband,
            float(kwargs.get("base_goal_resume_distance", self.base_goal_xy_deadband)),
        )
        self.base_goal_yaw_resume_distance = max(
            self.base_goal_yaw_deadband,
            float(kwargs.get("base_goal_yaw_resume_distance", self.base_goal_yaw_deadband)),
        )
        self.base_final_align_speed = max(0.0, float(kwargs.get("base_final_align_speed", 0.0)))
        self._base_goal_hold_frame = None

    def reset(self, context=None) -> None:
        del context
        self._base_goal_hold_frame = None
        self._base_goal_neutral_z = None
        self._base_goal_neutral_pitch = None
        self._lift_body_neutral = None
        self._body_pitch_neutral = None

    def _control_index(self, name: str):
        control = getattr(self.robot_cfg, "Control", None)
        if control is None or not hasattr(control, name):
            return None
        return int(getattr(control, name))

    def _zero_base_control(self, control: np.ndarray, *, translation=True, yaw=True):
        names = []
        if translation:
            names.extend(("vLinearX", "vLinearY"))
        if yaw:
            names.append("vRotYaw")
        for name in names:
            index = self._control_index(name)
            if index is not None and index < control.size:
                control[index] = 0.0

    def _apply_minimum_planar_speed(self, control: np.ndarray) -> None:
        if self.base_final_align_speed <= 0.0:
            self._zero_base_control(control, translation=True, yaw=False)
            return
        indices = [self._control_index(name) for name in ("vLinearX", "vLinearY")]
        if any(index is None or index >= control.size for index in indices):
            return
        planar = np.asarray([control[index] for index in indices], dtype=float)
        speed = float(np.linalg.norm(planar))
        if speed <= 1.0e-9:
            planar = np.array([self.base_final_align_speed, 0.0], dtype=float)
        elif speed < self.base_final_align_speed:
            planar *= self.base_final_align_speed / speed
        for index, value in zip(indices, planar):
            control[index] = value

    def _apply_base_goal_hysteresis(
        self,
        control: np.ndarray,
        agent_feedback: dict,
        task_info: dict,
        info: dict,
    ) -> None:
        if not task_info.get("base_goal_enable", True):
            self._base_goal_hold_frame = None
            self._zero_base_control(control)
            info["locomotion_phase"] = "disabled"
            return

        goal = task_info.get("goal_teleop", {}).get("base", None)
        if goal is None:
            self._base_goal_hold_frame = None
            return

        goal = np.asarray(goal, dtype=float).reshape(-1, 4, 4)[0]
        base = np.asarray(agent_feedback["robot_base_frame"], dtype=float).reshape(4, 4)
        distance = float(np.linalg.norm(goal[:2, 3] - base[:2, 3]))
        yaw = float(np.arctan2(base[1, 0], base[0, 0]))
        target_yaw = float(np.arctan2(goal[1, 0], goal[0, 0]))
        yaw_error = _wrap_to_pi(target_yaw - yaw)
        info["base_distance"] = distance
        info["base_yaw_error"] = yaw_error
        # A mobile-base PID regulates x, y, and yaw concurrently.  Learned
        # legged executors express those channels as independent planner
        # ``movement`` and ``facing`` vectors, so keep the PID yaw objective
        # active throughout translation.  Without this hint SONIC falls back
        # to facing the movement vector and recreates the legged tracker's
        # turn-then-walk-then-final-turn strategy.
        info["locomotion_facing_yaw"] = target_yaw

        if self.base_goal_xy_deadband <= 0.0 and self.base_goal_yaw_deadband <= 0.0:
            return

        if self._base_goal_hold_frame is not None:
            held = self._base_goal_hold_frame
            held_yaw = float(np.arctan2(held[1, 0], held[0, 0]))
            translation_goal_moved = (
                float(np.linalg.norm(goal[:2, 3] - held[:2, 3])) > self.base_goal_xy_deadband
            )
            yaw_goal_moved = abs(_wrap_to_pi(target_yaw - held_yaw)) > self.base_goal_yaw_deadband
            robot_departed = (
                distance >= self.base_goal_resume_distance
                or abs(yaw_error) >= self.base_goal_yaw_resume_distance
            )
            if not translation_goal_moved and not yaw_goal_moved and not robot_departed:
                self._zero_base_control(control)
                info["locomotion_phase"] = "arrived"
                return
            self._base_goal_hold_frame = None

        translation_reached = distance <= self.base_goal_xy_deadband
        yaw_reached = abs(yaw_error) <= self.base_goal_yaw_deadband

        if translation_reached and yaw_reached:
            self._base_goal_hold_frame = goal.copy()
            self._zero_base_control(control)
            info["locomotion_phase"] = "arrived"
        elif translation_reached:
            # Sonic and WBT cannot reliably complete a meaningful facing
            # change from a fully idle reference. Keep the smallest supported
            # gait active until yaw enters its deadband, then latch standing.
            self._apply_minimum_planar_speed(control)
            info["locomotion_phase"] = "final_align"
        else:
            if yaw_reached:
                self._zero_base_control(control, translation=False, yaw=True)
            info["locomotion_phase"] = "approach"

    def act(self, agent_feedback: dict, task_info: dict):
        control, info = super().act(agent_feedback, task_info)
        control = np.asarray(control, dtype=float).reshape(-1).copy()
        self._apply_base_goal_hysteresis(control, agent_feedback, task_info, info)
        return control, info

    def tracking_pos_with_vel(
        self, dof_pos_target, dof_vel_target, dof_pos_current, dof_vel_current
    ):
        # NOTE: Parameters for real robot open loop control with cmd pos and vel
        # K_p = 120.0 * np.ones(len(self.robot_cfg.Control))
        # K_d = 0.0 * np.ones(len(self.robot_cfg.Control))
        # NOTE: Parameters for real robot closed loop control with fbk pos and vel
        # K_p = 50.0 * np.ones(len(self.robot_cfg.Control))
        # K_d = 5.0 * np.ones(len(self.robot_cfg.Control))
        # NOTE: Parameters for viz robot open loop control with fbk pos and vel
        # K_p = 10.0 * np.ones(len(self.robot_cfg.Control))
        # K_d = 0.0 * np.ones(len(self.robot_cfg.Control))
        # NOTE: Parameters for sim robot open loop control with fbk pos and vel
        K_p = np.broadcast_to(self.position_kp, (len(self.robot_cfg.Control),)).copy()
        K_d = np.broadcast_to(self.velocity_kd, (len(self.robot_cfg.Control),)).copy()
        dof_vel_nominal = K_p * (dof_pos_target - dof_pos_current) + K_d * (
            dof_vel_target - dof_vel_current
        )
        nominal_control = (
            np.linalg.pinv(self.robot_cfg.dynamics_g(dof_pos_current)) @ dof_vel_nominal
        )
        return nominal_control

    # def tracking_pos_with_acc(
    #     self,
    #     dof_pos_target,
    #     dof_vel_target,
    #     dof_pos_current,
    #     dof_vel_current,
    # ):
    #     """
    #     Second-order (PD) position regulation that produces an acceleration target,
    #     then maps it to control using ONLY the acceleration block of dynamics_g.

    #     Notes:
    #     - dynamics_g(state) returns g_x with shape (2*num_dof, num_control)
    #         where only the bottom half (accelerations) is actuated.
    #     - If you want "track position with 0 velocity", pass dof_vel_target=None
    #         or a zero vector; we will default to zeros if None.
    #     """
    #     # ---- defaults / shapes -----
    #     n = dof_pos_current.shape[0]

    #     # ---- gains (stable at ~500 Hz; tune wn) ----
    #     # Use a critically damped 2nd-order system: qdd = wn^2 * e + 2*wn * edot
    #     wn = 5.0   # rad/s;1
    #     zeta = 1.0

    #     K_p = (wn ** 2) * np.ones(n)
    #     K_d = (2.0 * zeta * wn) * np.ones(n)

    #     # ---- PD in joint/DoF space ----
    #     dof_pos_error = dof_pos_target - dof_pos_current
    #     dof_vel_error = dof_vel_target - dof_vel_current
    #     dof_acc_target = K_p * dof_pos_error + K_d * dof_vel_error

    #     # ---- map desired accelerations to control ----
    #     state = np.concatenate((dof_pos_current, dof_vel_current))
    #     g_x = self.robot_cfg.dynamics_g(state)

    #     # Only bottom half is actuated: qdd = B(q) u
    #     B = g_x[self.robot_cfg.num_dof:, :]  # (num_dof, num_control)

    #     # Damped least-squares inverse for robustness vs singularities
    #     lam = 1e-2
    #     Binv = B.T @ np.linalg.inv(B @ B.T + (lam ** 2) * np.eye(B.shape[0]))

    #     nominal_control = Binv @ dof_acc_target
    #     return nominal_control
