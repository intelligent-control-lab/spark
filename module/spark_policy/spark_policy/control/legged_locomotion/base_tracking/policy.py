from __future__ import annotations

import numpy as np

from spark_policy.control.pid.base import BasePIDPolicy, wrap_to_pi
from spark_robot import RobotConfig, RobotKinematics


class LeggedLocomotionTrackingPolicy(BasePIDPolicy):
    """Goal tracker for legged command executors.

    Upper-body references retain the established PID/IK behavior. The virtual
    mobile-base controls are replaced by a gait-aware SE(2) tracker whose
    output is expressed in the world frame for collision-safety filtering.
    """

    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics, **kwargs) -> None:
        super().__init__(robot_cfg, robot_kinematics, **kwargs)
        self.dt = float(kwargs.get("tracking_dt", 0.02))
        self.max_speed = float(kwargs.get("max_planar_speed", 0.07))
        self.min_speed = float(kwargs.get("min_planar_speed", 0.025))
        self.max_yaw_rate = float(kwargs.get("max_yaw_rate", 0.30))
        self.position_gain = float(kwargs.get("position_gain", 0.65))
        self.velocity_damping = float(kwargs.get("velocity_damping", 0.30))
        self.yaw_gain = float(kwargs.get("yaw_gain", 0.85))
        self.slowdown_distance = float(kwargs.get("slowdown_distance", 0.45))
        self.translation_stop_distance = float(kwargs.get("translation_stop_distance", 0.14))
        self.translation_resume_distance = float(kwargs.get("translation_resume_distance", 0.16))
        self.position_tolerance = float(kwargs.get("position_tolerance", 0.12))
        self.yaw_tolerance = float(kwargs.get("yaw_tolerance", 0.08))
        self.settle_speed = float(kwargs.get("settle_speed", 0.045))
        self.settle_cycles = max(1, int(kwargs.get("settle_cycles", 5)))
        self.align_min_distance = float(kwargs.get("align_min_distance", 0.32))
        self.align_enter_angle = float(kwargs.get("align_enter_angle", 0.55))
        self.align_exit_angle = float(kwargs.get("align_exit_angle", 0.20))
        self.align_settle_yaw_rate = float(kwargs.get("align_settle_yaw_rate", 0.08))
        self.align_settle_cycles = max(1, int(kwargs.get("align_settle_cycles", 4)))
        self.command_settle_threshold = float(kwargs.get("command_settle_threshold", 0.01))
        self.max_acceleration = float(kwargs.get("max_acceleration", 0.18))
        self.max_yaw_acceleration = float(kwargs.get("max_yaw_acceleration", 0.75))
        self._phase = "idle"
        self._command_world = np.zeros(3, dtype=float)
        self._settle_count = 0
        self._align_settle_count = 0
        self._goal_xy = None
        self._arrived = False

    def reset(self, context=None) -> None:
        del context
        self._phase = "idle"
        self._command_world.fill(0.0)
        self._settle_count = 0
        self._align_settle_count = 0
        self._goal_xy = None
        self._arrived = False
        self._base_goal_neutral_z = None
        self._base_goal_neutral_pitch = None
        self._lift_body_neutral = None
        self._body_pitch_neutral = None

    def _control_index(self, name: str):
        control = getattr(self.robot_cfg, "Control", None)
        return (
            None if control is None or not hasattr(control, name) else int(getattr(control, name))
        )

    def _dof_value(self, values, name: str, default: float = 0.0) -> float:
        dofs = getattr(self.robot_cfg, "DoFs", None)
        if dofs is None or not hasattr(dofs, name):
            return float(default)
        index = int(getattr(dofs, name))
        values = np.asarray(values, dtype=float).reshape(-1)
        return float(values[index]) if index < values.size else float(default)

    def _rate_limit(self, target: np.ndarray) -> np.ndarray:
        target = np.asarray(target, dtype=float).reshape(3)
        limit = np.array(
            [self.max_acceleration * self.dt] * 2 + [self.max_yaw_acceleration * self.dt],
            dtype=float,
        )
        self._command_world += np.clip(target - self._command_world, -limit, limit)
        return self._command_world.copy()

    def _base_command(self, agent_feedback: dict, task_info: dict) -> tuple[np.ndarray, dict]:
        goal = task_info.get("goal_teleop", {}).get("base", None)
        if not task_info.get("base_goal_enable", True) or goal is None:
            self._phase = "idle"
            self._arrived = False
            self._goal_xy = None
            return self._rate_limit(np.zeros(3)), {"locomotion_phase": self._phase}

        goal = np.asarray(goal, dtype=float).reshape(-1, 4, 4)[0]
        base = np.asarray(agent_feedback["robot_base_frame"], dtype=float)
        position = base[:2, 3]
        yaw = float(np.arctan2(base[1, 0], base[0, 0]))
        target_yaw = float(np.arctan2(goal[1, 0], goal[0, 0]))
        error = goal[:2, 3] - position
        distance = float(np.linalg.norm(error))
        direction = error / max(distance, 1.0e-9)
        travel_yaw = float(np.arctan2(direction[1], direction[0]))
        travel_error = wrap_to_pi(travel_yaw - yaw)
        goal_yaw_error = wrap_to_pi(target_yaw - yaw)
        velocity = np.array(
            [
                self._dof_value(agent_feedback["dof_vel_fbk"], "LinearX"),
                self._dof_value(agent_feedback["dof_vel_fbk"], "LinearY"),
            ]
        )
        measured_speed = float(np.linalg.norm(velocity))
        measured_yaw_rate = self._dof_value(agent_feedback["dof_vel_fbk"], "RotYaw")

        new_goal = self._goal_xy is None or np.linalg.norm(goal[:2, 3] - self._goal_xy) > 1.0e-6
        if new_goal:
            self._goal_xy = goal[:2, 3].copy()
            self._arrived = False
            self._settle_count = 0
            self._align_settle_count = 0
            self._phase = (
                "align"
                if distance >= self.align_min_distance
                and abs(travel_error) > self.align_enter_angle
                else "approach"
            )

        if self._arrived:
            if distance <= self.translation_resume_distance:
                return self._rate_limit(np.zeros(3)), {
                    "locomotion_phase": "arrived",
                    "locomotion_facing_yaw": target_yaw,
                    "base_distance": distance,
                }
            self._arrived = False
            self._phase = "approach"

        if self._phase == "align":
            if distance < self.align_min_distance or abs(travel_error) <= self.align_exit_angle:
                self._phase = "align_settle"
                self._align_settle_count = 0
            else:
                target = np.array(
                    [
                        0.0,
                        0.0,
                        np.clip(
                            self.yaw_gain * travel_error, -self.max_yaw_rate, self.max_yaw_rate
                        ),
                    ]
                )
                return self._rate_limit(target), {
                    "locomotion_phase": self._phase,
                    "locomotion_facing_yaw": travel_yaw,
                    "base_distance": distance,
                    "travel_heading_error": travel_error,
                }

        if self._phase == "align_settle":
            # The learned gait needs the turn animation and physical yaw
            # momentum to finish before a walk animation is requested.
            settled = abs(measured_yaw_rate) <= self.align_settle_yaw_rate
            self._align_settle_count = self._align_settle_count + 1 if settled else 0
            target = np.zeros(3, dtype=float)
            command = self._rate_limit(target)
            if self._align_settle_count >= self.align_settle_cycles and np.linalg.norm(
                command
            ) <= max(self.command_settle_threshold, 1.0e-6):
                self._phase = "approach"
                self._align_settle_count = 0
            return command, {
                "locomotion_phase": self._phase,
                "locomotion_facing_yaw": travel_yaw,
                "base_distance": distance,
                "travel_heading_error": travel_error,
                "measured_yaw_rate": measured_yaw_rate,
            }

        if distance <= self.translation_stop_distance:
            self._phase = "settle" if abs(goal_yaw_error) <= self.yaw_tolerance else "final_align"

        if self._phase in ("settle", "final_align"):
            yaw_rate = np.clip(
                self.yaw_gain * goal_yaw_error, -self.max_yaw_rate, self.max_yaw_rate
            )
            if self._phase == "final_align" and abs(goal_yaw_error) > self.yaw_tolerance:
                # SONIC's locomotion planner does not reliably realize a large
                # facing change from a fully idle reference. Keep a minimal
                # goal-directed gait active while it turns, then settle once
                # the requested yaw has been reached.
                target = np.array(
                    [
                        direction[0] * self.min_speed,
                        direction[1] * self.min_speed,
                        yaw_rate,
                    ]
                )
            else:
                self._phase = "settle"
                target = np.array([0.0, 0.0, yaw_rate])
            settled = (
                measured_speed <= self.settle_speed and abs(goal_yaw_error) <= self.yaw_tolerance
            )
            self._settle_count = self._settle_count + 1 if settled else 0
            if distance > self.translation_resume_distance and measured_speed <= self.settle_speed:
                self._phase = "align" if abs(travel_error) > self.align_enter_angle else "approach"
                self._settle_count = 0
            elif distance <= self.position_tolerance and self._settle_count >= self.settle_cycles:
                self._arrived = True
                self._phase = "arrived"
                target[:] = 0.0
            return self._rate_limit(target), {
                "locomotion_phase": self._phase,
                "locomotion_facing_yaw": target_yaw,
                "base_distance": distance,
                "base_yaw_error": goal_yaw_error,
            }

        speed = self.position_gain * distance - self.velocity_damping * float(
            np.dot(velocity, direction)
        )
        if distance < self.slowdown_distance:
            speed_limit = self.min_speed + (self.max_speed - self.min_speed) * (
                (distance - self.translation_stop_distance)
                / max(self.slowdown_distance - self.translation_stop_distance, 1.0e-6)
            )
            speed_limit = float(np.clip(speed_limit, self.min_speed, self.max_speed))
        else:
            speed_limit = self.max_speed
        speed = float(np.clip(speed, self.min_speed, speed_limit))
        yaw_rate = float(
            np.clip(self.yaw_gain * travel_error, -self.max_yaw_rate, self.max_yaw_rate)
        )
        target = np.array([direction[0] * speed, direction[1] * speed, yaw_rate])
        return self._rate_limit(target), {
            "locomotion_phase": self._phase,
            "locomotion_facing_yaw": travel_yaw,
            "base_distance": distance,
            "travel_heading_error": travel_error,
        }

    def act(self, agent_feedback: dict, task_info: dict):
        control, info = super().act(agent_feedback, task_info)
        control = np.asarray(control, dtype=float).reshape(-1).copy()
        base_command, tracking_info = self._base_command(agent_feedback, task_info)
        for value, name in zip(base_command, ("vLinearX", "vLinearY", "vRotYaw")):
            index = self._control_index(name)
            if index is not None and index < control.size:
                control[index] = value
        info.update(tracking_info)
        info["locomotion_command_world"] = base_command.copy()
        return control, info
