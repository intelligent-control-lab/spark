#!/usr/bin/env python3
"""Generate deterministic cross-backend SPARK documentation GIFs.

The public robot launchers remain interactive.  This tool builds the same
robot-local configurations, then drives their existing keyboard goal/debug
state with a smooth deterministic trajectory for reproducible media capture.
Each case runs in a fresh process so Isaac/PhysX state cannot leak between
robot assets.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import importlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = REPO_ROOT / "build" / "media" / "robot_matrix"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


@dataclass(frozen=True)
class DemoCase:
    family: str
    variant: str
    runner_module: str
    robot_config: str
    test_case: str
    camera_lookat: tuple[float, float, float] | None
    camera_distance: float | None
    camera_azimuth: float | None
    camera_elevation: float | None
    obstacle_far: tuple[float, float, float]
    obstacle_near: tuple[float, float, float]
    right_goal_offset: tuple[float, float, float]
    isaac_obstacle_near: tuple[float, float, float] | None = None
    left_goal_offset: tuple[float, float, float] = (0.04, 0.08, -0.05)
    base_goal_offset: tuple[float, float, float] = (0.18, 0.04, 0.18)
    policy_config: str = "TeleopPIDPolicy"
    use_dual_arm: bool = False
    mobile_base: bool = False


_UNITREE_WBT = DemoCase(
    family="unitree_g1",
    variant="whole_body_wbt",
    runner_module="example.unitree_g1.run_unitree_g1_teleop",
    robot_config="UnitreeG1WholeBodyDynamic1Config",
    test_case="whole_goal_static_v1",
    camera_lookat=(0.35, 0.0, 0.95),
    camera_distance=3.1,
    camera_azimuth=None,
    camera_elevation=None,
    obstacle_far=(0.65, -0.62, 0.95),
    obstacle_near=(0.25, -0.29, 0.94),
    right_goal_offset=(0.10, -0.08, 0.10),
    policy_config="UnitreeG1WBTPolicy",
    use_dual_arm=True,
    mobile_base=True,
)
_AGIBOT_MOBILE = DemoCase(
    family="agibot_g1",
    variant="mobile_base",
    runner_module="example.agibot_g1.run_agibot_g1_teleop",
    robot_config="AgiBotG1MobileBaseDynamic1Config",
    test_case="whole_goal_static_v1",
    camera_lookat=(0.30, 0.0, 0.78),
    camera_distance=3.0,
    camera_azimuth=None,
    camera_elevation=None,
    obstacle_far=(0.75, -0.65, 0.80),
    obstacle_near=(0.30, -0.31, 0.48),
    right_goal_offset=(0.12, -0.08, 0.10),
    use_dual_arm=True,
    mobile_base=True,
)
_R1_MOBILE = DemoCase(
    family="galaxea_r1lite",
    variant="mobile_base",
    runner_module="example.galaxea_r1lite.run_galaxea_r1lite_teleop",
    robot_config="GalaxeaR1LiteMobileBaseDynamic1CollisionConfig",
    test_case="whole_goal_static_v1",
    camera_lookat=(0.30, 0.0, 0.78),
    camera_distance=3.0,
    camera_azimuth=None,
    camera_elevation=None,
    obstacle_far=(0.78, -0.68, 0.98),
    obstacle_near=(0.27, -0.34, 1.20),
    right_goal_offset=(0.10, -0.08, 0.08),
    use_dual_arm=True,
    mobile_base=True,
)
_KINOVA_SINGLE = DemoCase(
    family="kinova_gen3",
    variant="single_arm",
    runner_module="example.kinova_gen3.run_kinova_gen3_teleop",
    robot_config="KinovaGen3SingleArmDynamic1CollisionConfig",
    test_case="arm_goal_static_v1",
    # The shared default looks directly along this arm's primary bending
    # plane and loses the gripper during motion.  Rotate only enough to
    # keep the complete scenario visible.
    camera_lookat=(0.30, 0.0, 0.65),
    camera_distance=2.05,
    camera_azimuth=135.0,
    camera_elevation=-22.0,
    obstacle_far=(0.58, -0.48, 0.36),
    obstacle_near=(0.46, 0.05, 0.35),
    right_goal_offset=(0.10, 0.10, 0.08),
)
_KUKA_SINGLE = DemoCase(
    family="kuka_iiwa14",
    variant="single_arm",
    runner_module="example.kuka_iiwa14.run_kuka_iiwa14_teleop",
    robot_config="KukaIIWA14SingleArmDynamic1CollisionConfig",
    test_case="arm_goal_static_v1",
    camera_lookat=(0.30, 0.0, 0.55),
    camera_distance=1.90,
    camera_azimuth=135.0,
    camera_elevation=-22.0,
    obstacle_far=(0.62, -0.50, 0.52),
    obstacle_near=(0.45, 0.05, 0.54),
    right_goal_offset=(0.10, 0.10, 0.08),
)
_FANUC_SINGLE = DemoCase(
    family="fanuc_lrmate200id",
    variant="single_arm",
    runner_module="example.fanuc_lrmate200id.run_fanuc_lrmate200id_teleop",
    robot_config="FanucLRMate200iDSingleArmDynamic1CollisionConfig",
    test_case="arm_goal_static_v1",
    camera_lookat=(0.28, 0.0, 0.50),
    camera_distance=1.80,
    camera_azimuth=135.0,
    camera_elevation=-22.0,
    obstacle_far=(0.62, -0.50, 0.50),
    obstacle_near=(0.44, 0.05, 0.54),
    right_goal_offset=(0.08, 0.10, 0.08),
)


PROFILES = {
    "kinova_gen3__single_arm": _KINOVA_SINGLE,
    "kinova_gen3__dual_arm": replace(
        _KINOVA_SINGLE,
        variant="dual_arm",
        robot_config="KinovaGen3DualArmDynamic1CollisionConfig",
        camera_distance=2.45,
        obstacle_far=(0.62, -0.72, 1.42),
        obstacle_near=(0.48, -0.04, 0.46),
        use_dual_arm=True,
    ),
    "kuka_iiwa14__single_arm": _KUKA_SINGLE,
    "kuka_iiwa14__dual_arm": replace(
        _KUKA_SINGLE,
        variant="dual_arm",
        robot_config="KukaIIWA14DualArmDynamic1CollisionConfig",
        camera_distance=2.35,
        use_dual_arm=True,
    ),
    "fanuc_lrmate200id__single_arm": _FANUC_SINGLE,
    "fanuc_lrmate200id__dual_arm": replace(
        _FANUC_SINGLE,
        variant="dual_arm",
        robot_config="FanucLRMate200iDDualArmDynamic1CollisionConfig",
        camera_distance=2.30,
        use_dual_arm=True,
    ),
    "agibot_g1__right_arm": replace(
        _AGIBOT_MOBILE,
        variant="right_arm",
        robot_config="AgiBotG1RightArmDynamic1Config",
        test_case="arm_goal_static_v1",
        mobile_base=False,
        use_dual_arm=False,
        obstacle_near=(0.30, -0.31, 0.48),
    ),
    "agibot_g1__dual_arm": replace(
        _AGIBOT_MOBILE,
        variant="dual_arm",
        robot_config="AgiBotG1DualArmDynamic1Config",
        test_case="arm_goal_static_v1",
        mobile_base=False,
        obstacle_near=(0.18, -0.28, 0.52),
    ),
    "agibot_g1__fixed_base": replace(
        _AGIBOT_MOBILE,
        variant="fixed_base",
        robot_config="AgiBotG1FixedBaseDynamic1Config",
        test_case="arm_goal_static_v1",
        mobile_base=False,
        obstacle_near=(0.30, -0.31, 0.48),
    ),
    "agibot_g1__mobile_base": _AGIBOT_MOBILE,
    "galaxea_r1lite__right_arm": replace(
        _R1_MOBILE,
        variant="right_arm",
        robot_config="GalaxeaR1LiteRightArmDynamic1CollisionConfig",
        test_case="arm_goal_static_v1",
        mobile_base=False,
        use_dual_arm=False,
    ),
    "galaxea_r1lite__dual_arm": replace(
        _R1_MOBILE,
        variant="dual_arm",
        robot_config="GalaxeaR1LiteDualArmDynamic1CollisionConfig",
        test_case="arm_goal_static_v1",
        mobile_base=False,
    ),
    "galaxea_r1lite__fixed_base": replace(
        _R1_MOBILE,
        variant="fixed_base",
        robot_config="GalaxeaR1LiteFixedBaseDynamic1CollisionConfig",
        test_case="arm_goal_static_v1",
        mobile_base=False,
    ),
    "galaxea_r1lite__mobile_base": _R1_MOBILE,
    "unitree_g1__right_arm": replace(
        _UNITREE_WBT,
        variant="right_arm",
        robot_config="UnitreeG1RightArmDynamic1Config",
        test_case="arm_goal_static_v1",
        policy_config="TeleopPIDPolicy",
        mobile_base=False,
        use_dual_arm=False,
        obstacle_near=(0.34, -0.32, 0.98),
        isaac_obstacle_near=(0.34, -0.32, 0.98),
    ),
    "unitree_g1__dual_arm": replace(
        _UNITREE_WBT,
        variant="dual_arm",
        robot_config="UnitreeG1DualArmDynamic1Config",
        test_case="arm_goal_static_v1",
        policy_config="TeleopPIDPolicy",
        mobile_base=False,
        obstacle_near=(0.08, -0.44, 0.76),
        isaac_obstacle_near=(0.29, -0.27, 0.94),
        left_goal_offset=(0.08, 0.08, 0.08),
    ),
    "unitree_g1__fixed_base": replace(
        _UNITREE_WBT,
        variant="fixed_base",
        robot_config="UnitreeG1FixedBaseDynamic1Config",
        test_case="arm_goal_static_v1",
        policy_config="TeleopPIDPolicy",
        mobile_base=False,
        obstacle_near=(0.34, -0.32, 0.98),
        isaac_obstacle_near=(0.29, -0.27, 0.94),
        left_goal_offset=(0.08, 0.08, 0.08),
    ),
    "unitree_g1__mobile_base": replace(
        _UNITREE_WBT,
        variant="mobile_base",
        robot_config="UnitreeG1MobileBaseDynamic1Config",
        policy_config="TeleopPIDPolicy",
        obstacle_near=(0.34, -0.32, 0.98),
        isaac_obstacle_near=(0.29, -0.27, 0.94),
        left_goal_offset=(0.08, 0.08, 0.08),
    ),
    "unitree_g1__whole_body_wbt": _UNITREE_WBT,
    "unitree_g1__whole_body_sport": replace(
        _UNITREE_WBT,
        variant="whole_body_sport",
        robot_config="UnitreeG1WholeBodyWithHandDynamic1Config",
        policy_config="UnitreeG1SportSafePolicy",
        obstacle_near=(0.42, -0.42, 0.95),
        isaac_obstacle_near=(0.42, -0.42, 0.95),
        # Keep the fixed-camera documentation motion straight and compact.
        base_goal_offset=(0.18, 0.0, 0.0),
    ),
    "unitree_g1__whole_body_sonic": replace(
        _UNITREE_WBT,
        variant="whole_body_sonic",
        policy_config="UnitreeG1SonicPolicy",
    ),
}


MODES = {
    "mujoco_teleop": ("mujoco", "teleop", 1),
    "isaac_teleop": ("isaac", "teleop", 1),
    "mujoco_benchmark": ("mujoco", "benchmark", 1),
    "isaac_benchmark": ("isaac", "benchmark", 1),
    "isaac_parallel": ("isaac", "benchmark", 4),
}


def _smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def _segment(time_value: float, start: float, end: float) -> float:
    if end <= start:
        return float(time_value >= end)
    return _smoothstep((time_value - start) / (end - start))


def _smooth_waypoints(time_value: float, waypoints) -> np.ndarray:
    """Interpolate a deterministic path without velocity discontinuities."""

    if time_value <= waypoints[0][0]:
        return np.asarray(waypoints[0][1], dtype=float)
    for (start_time, start), (end_time, end) in zip(waypoints, waypoints[1:]):
        if time_value <= end_time:
            blend = _segment(time_value, start_time, end_time)
            start = np.asarray(start, dtype=float)
            return start + blend * (np.asarray(end, dtype=float) - start)
    return np.asarray(waypoints[-1][1], dtype=float)


class DocumentationRecordingPipeline:
    """Mixin created dynamically with the configured TeleopPipeline base."""

    demo_case: DemoCase
    demo_duration: float
    recording_backend: str

    def _initialize_documentation_recording(self) -> None:
        self._documentation_recorder = None
        self._demo_previous_obstacles = None
        self._demo_ee_positions = []
        self._demo_left_ee_positions = []
        self._demo_goal_tracking_samples = []
        self._demo_goal_tracking_snapshot = None
        self._demo_interaction_trigger_steps = [0, 0, 0]
        self._demo_interaction_max_correction = [0.0, 0.0, 0.0]
        self._documentation_started = time.perf_counter()
        self._apply_demo_state(0.0)
        if self.recording_backend == "isaac":
            from spark_agent.simulation.isaac.viewport_recorder import IsaacViewportRecorder

            self._documentation_recorder = IsaacViewportRecorder(
                video_path=getattr(self.cfg.env.agent, "record_video_path", None),
                gif_path=getattr(self.cfg.env.agent, "record_gif_path", None),
                width=int(self.cfg.env.agent.record_width),
                height=int(self.cfg.env.agent.record_height),
                fps=float(self.cfg.env.agent.record_fps),
            )

    def _apply_demo_state(self, simulated_time: float) -> None:
        agent = self.env.agent
        case = self.demo_case
        timeline = 16.0 * simulated_time / max(self.demo_duration, 1.0e-9)

        # Establish visibly tracked arm goals before the first obstacle
        # interaction.  Earlier recordings moved the goals and obstacles at
        # the same time, so a correctly cautious controller could appear not
        # to track at all.
        goal_out = _segment(timeline, 0.2, 1.0)
        goal_back = _segment(timeline, 13.8, 16.0)
        goal_scale = goal_out * (1.0 - 0.35 * goal_back)
        right_offset = goal_scale * np.asarray(case.right_goal_offset, dtype=float)
        left_scale = _segment(timeline, 0.4, 1.2) * (1.0 - 0.25 * _segment(timeline, 13.8, 16.0))
        left_offset = left_scale * np.asarray(case.left_goal_offset, dtype=float)
        base_scale = _segment(timeline, 0.3, 2.0) * (1.0 - 0.20 * _segment(timeline, 13.8, 16.0))
        base_offset = base_scale * np.asarray(case.base_goal_offset, dtype=float)

        agent.right_goal_debug_frame[:] = np.eye(4)
        agent.right_goal_debug_frame[:3, 3] = right_offset
        agent.left_goal_debug_frame[:] = np.eye(4)
        agent.left_goal_debug_frame[:3, 3] = left_offset
        agent.base_goal_debug_frame[:] = np.eye(4)
        agent.base_goal_debug_frame[:3, 3] = base_offset

        far = np.asarray(case.obstacle_far, dtype=float)
        near = np.asarray(case.obstacle_near, dtype=float)
        opposite_far = near - (far - near)
        opposite_near = near + np.array((0.0, -0.06, 0.04))
        overhead_far = near + np.array((0.0, 0.0, max(0.40, abs(far[2] - near[2]) + 0.30)))
        overhead_near = near + np.array((-0.04, 0.02, 0.04))
        right_path = (
            (0.0, far),
            (4.8, far),
            (6.5, near),
            (7.4, far),
            (7.8, opposite_far),
            (9.3, opposite_near),
            (10.2, opposite_far),
            (10.6, overhead_far),
            (12.3, overhead_near),
            (13.2, overhead_far),
            (13.8, far),
            (16.0, far),
        )
        obstacles = [_smooth_waypoints(timeline, right_path)]
        if case.use_dual_arm:
            mirrored_far = far.copy()
            mirrored_far[1] *= -1.0
            mirrored_near = near.copy()
            mirrored_near[1] *= -1.0
            center_near = near.copy()
            center_near[1] = abs(center_near[1]) * 0.35
            left_path = (
                (0.0, mirrored_far),
                (4.4, mirrored_far),
                (6.7, mirrored_near),
                (8.0, mirrored_far),
                (8.8, mirrored_far),
                (11.3, center_near),
                (12.7, mirrored_far),
                (16.0, mirrored_far),
            )
            obstacles.append(_smooth_waypoints(timeline, left_path))
        if len(agent.obstacle_debug_frame):
            obstacle_count = min(len(obstacles), len(agent.obstacle_debug_frame))
            previous = np.asarray(
                agent.obstacle_debug_frame[:obstacle_count, :3, 3], dtype=float
            ).copy()
            current = np.asarray(obstacles[:obstacle_count], dtype=float)
            agent.obstacle_debug_frame[:obstacle_count, :3, 3] = current
            control_period = float(self.cfg.env.agent.dt) * int(
                self.cfg.env.agent.control_decimation
            )
            velocity = (current - previous) / max(control_period, 1.0e-9)
            for obstacle_id in range(obstacle_count):
                spatial_velocity = np.r_[velocity[obstacle_id], np.zeros(3)]
                agent.obstacle_debug_velocity[obstacle_id] = spatial_velocity
            self._demo_previous_obstacles = current.copy()

    def post_physics_step(self, agent_feedback, task_info, action_info):
        control_period = float(self.cfg.env.agent.dt) * int(self.cfg.env.agent.control_decimation)
        self._apply_demo_state((self.pipeline_step + 1) * control_period)
        super().post_physics_step(agent_feedback, task_info, action_info)
        timeline = 16.0 * self.pipeline_step * control_period / max(self.demo_duration, 1.0e-9)
        if 3.3 <= timeline <= 4.8:
            self._demo_goal_tracking_samples.append(
                {
                    "right": (
                        None
                        if getattr(self, "dist_goal_right", None) is None
                        else float(self.dist_goal_right)
                    ),
                    "left": (
                        None
                        if getattr(self, "dist_goal_left", None) is None
                        else float(self.dist_goal_left)
                    ),
                }
            )
            if self._demo_goal_tracking_snapshot is None:
                right_goal = getattr(self, "goal_right_frame_base", None)
                left_goal = getattr(self, "goal_left_frame_base", None)
                self._demo_goal_tracking_snapshot = {
                    "right_goal_world": (
                        None
                        if right_goal is None
                        else np.asarray(right_goal[0, :3, 3], dtype=float).tolist()
                    ),
                    "left_goal_world": (
                        None
                        if left_goal is None
                        else np.asarray(left_goal[0, :3, 3], dtype=float).tolist()
                    ),
                    "right_ee_world": (
                        np.asarray(
                            self.robot_frames_world[int(self.robot_cfg.Frames.R_ee), :3, 3],
                            dtype=float,
                        ).tolist()
                        if hasattr(self.robot_cfg.Frames, "R_ee")
                        else None
                    ),
                    "left_ee_world": (
                        np.asarray(
                            self.robot_frames_world[int(self.robot_cfg.Frames.L_ee), :3, 3],
                            dtype=float,
                        ).tolist()
                        if hasattr(self.robot_cfg.Frames, "L_ee")
                        else None
                    ),
                    "dof_position_feedback": np.asarray(
                        agent_feedback["dof_pos_fbk"], dtype=float
                    ).tolist(),
                    "dof_position_target": np.asarray(
                        action_info.get("dof_pos_target", []), dtype=float
                    ).tolist(),
                    "ik_success": bool(action_info.get("ik_success", False)),
                }
        if hasattr(self.robot_cfg.Frames, "R_ee"):
            self._demo_ee_positions.append(
                np.asarray(
                    self.robot_frames_world[int(self.robot_cfg.Frames.R_ee), :3, 3],
                    dtype=float,
                ).copy()
            )
        if hasattr(self.robot_cfg.Frames, "L_ee"):
            self._demo_left_ee_positions.append(
                np.asarray(
                    self.robot_frames_world[int(self.robot_cfg.Frames.L_ee), :3, 3],
                    dtype=float,
                ).copy()
            )
        for interaction_id, (start, end) in enumerate(((4.8, 7.4), (7.8, 10.2), (10.6, 13.2))):
            if start <= timeline <= end:
                if bool(np.any(action_info.get("trigger_safe", False))):
                    self._demo_interaction_trigger_steps[interaction_id] += 1
                reference_key, safe_key = (
                    ("sport_u_ref", "sport_u_safe")
                    if "sport_u_ref" in action_info and "sport_u_safe" in action_info
                    else ("u_ref", "u_safe")
                )
                if reference_key in action_info and safe_key in action_info:
                    correction = float(
                        np.linalg.norm(
                            np.asarray(action_info[safe_key], dtype=float)
                            - np.asarray(action_info[reference_key], dtype=float)
                        )
                    )
                    self._demo_interaction_max_correction[interaction_id] = max(
                        self._demo_interaction_max_correction[interaction_id], correction
                    )
        if self._documentation_recorder is not None:
            self._documentation_recorder.schedule(self.pipeline_step * control_period)

    def run(self, save_path=None):
        try:
            return super().run(save_path=save_path)
        finally:
            recorder = self._documentation_recorder
            if recorder is not None and not recorder.closed:
                if recorder.pending:
                    recorder.drain(self.env.agent.sim.render)
                recorder.close()
            self._write_documentation_report()

    def _write_documentation_report(self) -> None:
        metadata = dict(self.cfg.documentation_metadata)
        output = Path(metadata["gif_path"])
        ee_positions = np.asarray(self._demo_ee_positions, dtype=float)
        left_ee_positions = np.asarray(self._demo_left_ee_positions, dtype=float)
        ee_motion_span = (
            float(np.max(np.ptp(ee_positions, axis=0))) if ee_positions.shape[0] > 1 else 0.0
        )
        safety_trigger_steps = int(self._safety_trigger_steps)
        safety_observed_steps = int(self._safety_observed_steps)
        left_ee_motion_span = (
            float(np.max(np.ptp(left_ee_positions, axis=0)))
            if left_ee_positions.shape[0] > 1
            else 0.0
        )
        interactions_passed = all(
            trigger_steps >= 2 and correction > 1.0e-5
            for trigger_steps, correction in zip(
                self._demo_interaction_trigger_steps,
                self._demo_interaction_max_correction,
            )
        )
        right_goal_errors = [
            sample["right"]
            for sample in self._demo_goal_tracking_samples
            if sample["right"] is not None
        ]
        left_goal_errors = [
            sample["left"]
            for sample in self._demo_goal_tracking_samples
            if sample["left"] is not None
        ]
        minimum_right_goal_error = min(right_goal_errors, default=None)
        minimum_left_goal_error = min(left_goal_errors, default=None)
        reduced_unitree_tracking_passed = True
        if (
            self.demo_case.family == "unitree_g1"
            and self.demo_case.policy_config == "TeleopPIDPolicy"
        ):
            reduced_unitree_tracking_passed = (
                minimum_right_goal_error is not None
                and minimum_right_goal_error <= 0.12
                and (
                    not self.demo_case.use_dual_arm
                    or (minimum_left_goal_error is not None and minimum_left_goal_error <= 0.12)
                )
            )
        passed = (
            output.is_file()
            and output.stat().st_size > 0
            and interactions_passed
            and ee_motion_span >= 0.04
            and (not self.demo_case.use_dual_arm or left_ee_motion_span >= 0.03)
            and reduced_unitree_tracking_passed
        )
        report = {
            **metadata,
            "status": "PASS" if passed else "FAIL",
            "wall_seconds": time.perf_counter() - self._documentation_started,
            "safety_trigger_steps": safety_trigger_steps,
            "safety_observed_steps": safety_observed_steps,
            "end_effector_motion_span": ee_motion_span,
            "left_end_effector_motion_span": left_ee_motion_span,
            "minimum_unobstructed_right_goal_error": minimum_right_goal_error,
            "minimum_unobstructed_left_goal_error": minimum_left_goal_error,
            "unobstructed_goal_tracking_passed": reduced_unitree_tracking_passed,
            "unobstructed_goal_tracking_snapshot": self._demo_goal_tracking_snapshot,
            "right_end_effector_bounds": (
                [
                    np.min(ee_positions, axis=0).tolist(),
                    np.max(ee_positions, axis=0).tolist(),
                ]
                if ee_positions.size
                else None
            ),
            "left_end_effector_bounds": (
                [
                    np.min(left_ee_positions, axis=0).tolist(),
                    np.max(left_ee_positions, axis=0).tolist(),
                ]
                if left_ee_positions.size
                else None
            ),
            "interaction_safety_trigger_steps": self._demo_interaction_trigger_steps,
            "interaction_max_safe_correction": self._demo_interaction_max_correction,
            "gif_bytes": output.stat().st_size if output.is_file() else 0,
        }
        report_path = Path(self.cfg.documentation_case_report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)


def _recording_pipeline_type(case: DemoCase, duration: float, backend: str):
    from spark_pipeline import TeleopPipeline

    recording_case = case
    if backend == "isaac" and case.isaac_obstacle_near is not None:
        # PhysX and MuJoCo settle floating-base Unitree configurations at
        # slightly different world heights.  Keep one choreography while
        # placing its contact waypoint at the measured backend-local arm pose.
        recording_case = replace(case, obstacle_near=case.isaac_obstacle_near)

    class RecordingPipeline(DocumentationRecordingPipeline, TeleopPipeline):
        def __init__(self, cfg):
            self.demo_case = recording_case
            self.demo_duration = duration
            self.recording_backend = backend
            super().__init__(cfg)
            self._initialize_documentation_recording()

    return RecordingPipeline


def _viewer_kwargs(case: DemoCase) -> dict:
    options = {
        "viewer_lookat": case.camera_lookat,
        "viewer_distance": case.camera_distance,
        "viewer_azimuth": case.camera_azimuth,
        "viewer_elevation": case.camera_elevation,
        "viewer_vertical_fov": 45.0,
    }
    return {name: value for name, value in options.items() if value is not None}


def _viewer_config(case: DemoCase, num_envs: int = 1) -> dict:
    from spark_agent.simulation.viewer_config import DEFAULT_VIEWER_CONFIG

    result = dict(DEFAULT_VIEWER_CONFIG)
    overrides = {
        "camera_lookat": case.camera_lookat,
        "camera_distance": case.camera_distance,
        "camera_azimuth": case.camera_azimuth,
        "camera_elevation": case.camera_elevation,
    }
    result.update({name: value for name, value in overrides.items() if value is not None})
    if num_envs > 1:
        grid_side = int(math.ceil(math.sqrt(num_envs)))
        grid_extent = max(2.5, (grid_side - 1) * 2.5)
        result.update(
            camera_lookat=(0.0, 0.0, 0.8),
            camera_distance=1.30 * grid_extent,
            camera_azimuth=135.0,
            camera_elevation=-47.0,
            grid_extent=max(5.0, 0.75 * grid_extent),
        )
    return result


def _build_teleop_config(
    case: DemoCase,
    *,
    backend: str,
    output: Path,
    duration: float,
    fps: float,
    width: int,
    height: int,
):
    from spark_robot import get_agent_class_name

    control_period = 0.02
    common = {
        "backend": backend,
        "robot_cfg": case.robot_config,
        "agent_cfg": get_agent_class_name(case.robot_config, backend=backend),
        "num_envs": 1,
        "dynamics_backend": "simulator",
        "safe_algo": "rssa",
        "minimum_distance": 0.05,
        "num_obstacle_task": 0,
        "num_obstacle_debug": 2 if case.use_dual_arm else 1,
        "enable_viewer": True,
        "enable_keyboard_control": False,
        "enable_camera": False,
        "real_time": True,
        "render_robot_collision_volumes": True,
        "viewer_show_simulation_info": False,
        "device": "cpu",
        "policy_config": case.policy_config,
        **_viewer_kwargs(case),
    }

    if case.runner_module == "example.unitree_g1.run_unitree_g1_teleop":
        common.update(
            robot_cfg=case.robot_config,
            base_goal_enable=case.mobile_base,
            arm_goal_enable=True,
            # Documentation offsets should start at the realized simulator
            # hand poses.  The reduced Unitree configs otherwise combine a
            # generic identity-orientation home target with the humanoid
            # articulation and spend the clip solving that unrelated pose.
            arm_goal_init_from_current_ee=(case.policy_config == "TeleopPIDPolicy"),
            max_num_reset=-1,
            # Reduced Unitree FOAM volumes intentionally overlap at adjacent
            # elbow/torso links in the nominal pose. Environment avoidance is
            # the behavior demonstrated here; physical contacts remain on.
            enable_self_collision=(case.use_dual_arm and case.policy_config != "TeleopPIDPolicy"),
        )
        if case.policy_config == "UnitreeG1WBTPolicy":
            from spark_pipeline.teleop.unitree_g1_whole_body_config import build_config

            common["use_isaac_tensor_backend"] = True
            cfg = build_config(**common)
        elif case.policy_config == "UnitreeG1SonicPolicy":
            from example.unitree_g1 import sonic_support

            deploy_root = (
                REPO_ROOT.parent / "sonic" / "GR00T-WholeBodyControl" / "gear_sonic_deploy"
            )
            common.update(
                auto_launch_sonic_server=True,
                sonic_deploy_root=str(deploy_root),
                use_isaac_tensor_backend=True,
            )
            common = sonic_support._with_sonic_runtime_defaults(common)
            cfg = sonic_support._pipeline_config_class(case.robot_config)()
            for function_name in (
                "config_pipeline",
                "config_task_module",
                "config_agent_module",
                "config_policy_module",
                "config_safety_module",
            ):
                cfg = getattr(sonic_support, function_name)(cfg, **common)
            cfg.documentation_sonic_kwargs = dict(common)
        elif case.policy_config == "UnitreeG1SportSafePolicy":
            from spark_pipeline.teleop.unitree_g1_whole_body_config import build_config
            from spark_policy.composed_policy.unitree_g1.sport_safe import (
                UnitreeG1SportSafePolicyConfig,
            )

            common.update(
                composed_policy_class="UnitreeG1SportSafePolicy",
                executor_policy_class="UnitreeG1SportExecutorAdapter",
                use_isaac_tensor_backend=True,
                # SPORT is most stable in its learned walking regime.  Bound
                # this fixed-camera documentation pass to a slow crawl so the
                # robot remains inside the shared teleop shot while the arm
                # and obstacles demonstrate the safety response.
                max_planar_speed=0.03,
                min_planar_speed=0.02,
                pre_safe_loco_command_limit=[0.03, 0.03, 0.30],
            )
            for name, value in UnitreeG1SportSafePolicyConfig().teleop_overrides().items():
                common.setdefault(name, value)
            cfg = build_config(**common)
        else:
            runner = importlib.import_module(case.runner_module)
            cfg = runner.build_config(**common)
    else:
        runner = importlib.import_module(case.runner_module)
        cfg = runner.PipelineConfig()
        for function_name in (
            "config_pipeline",
            "config_task_module",
            "config_agent_module",
            "config_policy_module",
            "config_safety_module",
        ):
            cfg = getattr(runner, function_name)(cfg, **common)

    cfg.env.task.use_dual_arm = case.use_dual_arm
    cfg.env.task.base_goal_enable = case.mobile_base
    if case.family == "unitree_g1" and case.policy_config == "TeleopPIDPolicy":
        cfg.env.task.arm_goal_init_from_current_ee = True

    control_period = float(cfg.env.agent.dt) * int(cfg.env.agent.control_decimation)
    cfg.max_num_steps = int(math.ceil(duration / control_period))
    cfg.max_num_reset = -1
    cfg.render_every = 1 if backend == "mujoco" else max(1, round(1.0 / (fps * control_period)))
    cfg.env.agent.record_gif_path = str(output)
    cfg.env.agent.record_video_path = None
    cfg.env.agent.record_duration = float(duration)
    cfg.env.agent.record_fps = float(fps)
    cfg.env.agent.record_width = int(width)
    cfg.env.agent.record_height = int(height)
    cfg.env.agent.viewer_config = _viewer_config(case)
    cfg.env.agent.enable_viewer = True
    cfg.env.agent.enable_keyboard_control = False
    cfg.env.agent.real_time = True
    return cfg


def _run_sonic_teleop(cfg, pipeline_type):
    """Launch the optional SONIC server around the normal configured pipeline."""

    from example.unitree_g1 import sonic_support
    from spark_pipeline.teleop.unitree_g1_whole_body_config import run_configured_pipeline

    kwargs = dict(cfg.documentation_sonic_kwargs)
    command, cwd = sonic_support._build_sonic_backend_command(**kwargs)
    backend = None

    def before_pipeline(pipeline):
        nonlocal backend

        def progress():
            if kwargs.get("backend") == "isaac":
                pipeline.env.agent.render()

        backend = sonic_support._launch_sonic_backend(
            command,
            endpoint=kwargs.get(
                "sonic_endpoint", getattr(cfg.policy.executor, "sonic_endpoint", None)
            ),
            startup_timeout_s=kwargs.get("sonic_server_startup_timeout", 90.0),
            cwd=cwd,
            progress_callback=progress,
        )

    def after_pipeline():
        nonlocal backend
        if backend is None:
            return
        process = backend
        backend = None
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)

    try:
        return run_configured_pipeline(
            cfg,
            kwargs,
            pipeline_class=pipeline_type,
            error_label="SONIC documentation recording",
            before_pipeline=before_pipeline,
            after_pipeline=after_pipeline,
        )
    finally:
        after_pipeline()


def _control_period(robot_config: str) -> float:
    import spark_robot

    timing = getattr(spark_robot, robot_config)().simulator_dynamics
    return float(timing.physics_dt) * int(timing.control_decimation)


def _benchmark_kwargs(
    case: DemoCase,
    *,
    backend: str,
    num_envs: int,
    output: Path,
    duration: float,
    fps: float,
    width: int,
    height: int,
) -> dict:
    control_period = _control_period(case.robot_config)
    max_num_steps = int(math.ceil(duration / control_period))
    viewer_config = _viewer_config(case, num_envs)
    if backend == "mujoco" and num_envs == 1 and case.family == "unitree_g1" and case.mobile_base:
        # Whole-body benchmark goals traverse farther than the keyboard demo.
        # Keep the user's normal viewing direction while centering the path so
        # reduced mobile, WBT, and SONIC robots remain fully visible after
        # walking away from the origin.
        viewer_config.update(camera_lookat=(0.75, 0.0, 0.90), camera_distance=4.2)
        if case.policy_config in {
            "UnitreeG1WBTPolicy",
            "UnitreeG1SportSafePolicy",
            "UnitreeG1SonicPolicy",
        }:
            # Learned whole-body policies can continue walking throughout a
            # recording, unlike the reduced PID model which settles near its
            # goal.  Retain the same default-like viewing direction but widen
            # the fixed shot so their full trajectories remain visible.
            viewer_config["camera_distance"] = (
                7.0 if case.policy_config == "UnitreeG1SonicPolicy" else 4.2
            )
    if (
        case.robot_config
        in {
            "UnitreeG1WholeBodyDynamic1Config",
            "UnitreeG1WholeBodyWithHandDynamic1Config",
        }
        and num_envs > 1
    ):
        # The WBT runtime spaces cloned humanoids farther apart than the
        # generic manipulator runtime. Its direct Kit camera does not apply the
        # generic grid extent automatically, so use a wider distance while
        # preserving the shared parallel-view orientation.
        viewer_config["camera_distance"] = 11.0
    if backend == "isaac" and num_envs == 1:
        # The mobile benchmarks traverse a much larger world-space span than
        # their keyboard demonstrations. Keep the example's default viewing
        # direction, but center the expected path and widen just enough that
        # the complete robot remains in frame after moving toward the goal.
        single_mobile_views = {
            # Keep the floor-level planar goal fully visible without changing
            # the example's default viewing direction.
            "UnitreeG1WholeBodyDynamic1Config": ((0.55, 0.0, 0.55), 5.2),
            "UnitreeG1WholeBodyWithHandDynamic1Config": ((0.75, 0.0, 0.90), 4.2),
            "AgiBotG1MobileBaseDynamic1Config": ((0.65, 0.0, 0.78), 3.8),
            "GalaxeaR1LiteMobileBaseDynamic1CollisionConfig": (
                (0.65, 0.0, 0.78),
                3.8,
            ),
        }
        if case.robot_config in single_mobile_views:
            lookat, distance = single_mobile_views[case.robot_config]
            viewer_config.update(camera_lookat=lookat, camera_distance=distance)
    # MuJoCo advances its recorder clock when a rendered pipeline frame is
    # submitted, so render every control step and let its recorder select the
    # requested capture FPS. Isaac schedules directly from simulated time.
    render_every = 1 if backend == "mujoco" else max(1, round(1.0 / (fps * control_period)))
    kwargs = {
        "backend": backend,
        "robot_config": case.robot_config,
        "test_case": case.test_case,
        "num_envs": num_envs,
        "num_resets": -1,
        "max_num_reset": -1,
        "max_episode_length": min(400, max_num_steps),
        "max_num_steps": max_num_steps,
        "dynamics_backend": "simulator",
        "viewer": True,
        "viewer_show_simulation_info": False,
        "real_time": True,
        "render_every": render_every,
        "safe_algo": "rssa",
        "minimum_distance": 0.05,
        "render_robot_collision_volumes": True,
        "render_safety_trigger_constraints": True,
        "render_safety_violations": True,
        "max_visualized_collision_envs": min(num_envs, 4),
        "seed": 0,
        "viewer_config": viewer_config,
        "record_fps": fps,
        "record_width": width,
        "record_height": height,
    }
    if backend == "isaac":
        kwargs["device"] = "cpu" if num_envs == 1 else "cuda:0"
    if case.runner_module == "example.unitree_g1.run_unitree_g1_teleop":
        reduced_unitree_pid = case.policy_config == "TeleopPIDPolicy"
        kwargs.update(
            policy_config=case.policy_config,
            arm_goal_mode="random",
            base_goal_mode="random" if case.mobile_base else "disabled",
            obstacle_mode="random",
            hold_arm_goals_during_locomotion=False,
            # Reduced Unitree PID profiles begin inside the conservative
            # self-safety activation band.  Keeping that optional constraint
            # enabled makes an obstacle-avoidance documentation clip static.
            # Physical contacts and robot/environment safety remain active;
            # WBT and SONIC retain their normal self-safety configuration.
            enable_self_collision=case.use_dual_arm and not reduced_unitree_pid,
        )
        if backend == "isaac":
            kwargs.pop("device", None)
            kwargs["isaac_device"] = "cpu" if num_envs == 1 else "cuda:0"
            kwargs["record_video_path"] = str(output.with_suffix(".mp4"))
        else:
            kwargs["record_gif_path"] = str(output)
        if case.policy_config == "UnitreeG1SonicPolicy":
            kwargs.update(
                auto_launch_sonic_server=True,
                sonic_deploy_root=str(
                    REPO_ROOT.parent / "sonic" / "GR00T-WholeBodyControl" / "gear_sonic_deploy"
                ),
            )
    else:
        kwargs["record_gif_path"] = str(output)
    return kwargs


def _run_mujoco_benchmark(case: DemoCase, kwargs: dict) -> None:
    if case.runner_module == "example.unitree_g1.run_unitree_g1_teleop":
        runner = importlib.import_module("example.unitree_g1.run_unitree_g1_benchmark")
        runner.run(**kwargs)
        return

    benchmark_module = case.runner_module.replace("_teleop", "_benchmark")
    runner = importlib.import_module(benchmark_module)
    cfg = runner.build_config(**kwargs)
    cfg.max_num_steps = int(kwargs["max_num_steps"])
    cfg.max_num_reset = -1
    cfg.render_every = int(kwargs["render_every"])
    cfg.env.agent.viewer_config = dict(kwargs["viewer_config"])
    for name in (
        "record_gif_path",
        "record_fps",
        "record_width",
        "record_height",
    ):
        setattr(cfg.env.agent, name, kwargs[name])
    cfg.env.agent.record_duration = float(kwargs["max_num_steps"]) * _control_period(
        case.robot_config
    )
    cfg.env.agent.real_time = True
    from spark_pipeline import TeleopPipeline, run_simulation_pipeline

    run_simulation_pipeline(cfg, backend="mujoco", pipeline_class=TeleopPipeline)


def _convert_unitree_video(output: Path, fps: float, width: int, height: int) -> None:
    source = output.with_suffix(".mp4")
    if not source.is_file():
        raise RuntimeError(f"Unitree Isaac recorder did not create {source}")
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-v",
            "error",
            "-i",
            str(source),
            "-vf",
            f"fps={fps},scale={width}:{height}:flags=lanczos",
            "-loop",
            "0",
            str(output),
        ],
        check=True,
    )
    source.unlink()


def _run_benchmark(
    case: DemoCase,
    *,
    backend: str,
    num_envs: int,
    output: Path,
    duration: float,
    fps: float,
    width: int,
    height: int,
) -> None:
    kwargs = _benchmark_kwargs(
        case,
        backend=backend,
        num_envs=num_envs,
        output=output,
        duration=duration,
        fps=fps,
        width=width,
        height=height,
    )
    if backend == "mujoco":
        _run_mujoco_benchmark(case, kwargs)
        return
    benchmark_module = case.runner_module.replace("_teleop", "_benchmark")
    runner = importlib.import_module(benchmark_module)
    runner.run(**kwargs)
    if case.runner_module == "example.unitree_g1.run_unitree_g1_teleop":
        _convert_unitree_video(output, fps, width, height)


def _gif_properties(output: Path) -> tuple[int, float]:
    from PIL import Image

    frames = 0
    duration_ms = 0.0
    with Image.open(output) as image:
        for frame_index in range(int(getattr(image, "n_frames", 1))):
            image.seek(frame_index)
            frames += 1
            duration_ms += float(image.info.get("duration", 0.0))
    return frames, duration_ms / 1000.0


def _optimize_gif(output: Path, fps: float, width: int, height: int) -> None:
    optimized = output.with_name(f"{output.stem}.optimized.gif")
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-v",
            "error",
            "-i",
            str(output),
            "-filter_complex",
            (
                f"[0:v]fps={fps},scale={width}:{height}:flags=lanczos,split[a][b];"
                "[a]palettegen=max_colors=128:stats_mode=diff[p];"
                "[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle"
            ),
            str(optimized),
        ],
        check=True,
    )
    optimized.replace(output)


def _run_inner(args) -> int:
    case = PROFILES[args.profile]
    backend, workflow, num_envs = MODES[args.mode]
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    metadata = {
        "robot": case.family,
        "profile": args.profile,
        "variant": case.variant,
        "robot_config": case.robot_config,
        "policy_config": case.policy_config,
        "mode": args.mode,
        "backend": backend,
        "workflow": workflow,
        "num_envs": num_envs,
        "duration_seconds": args.duration,
        "fps": args.fps,
        "resolution": [args.width, args.height],
        "real_time_playback": True,
        "gif_path": str(output),
    }
    if workflow == "teleop":
        if num_envs != 1:
            raise SystemExit("Scripted teleoperation recording requires one environment")
        cfg = _build_teleop_config(
            case,
            backend=backend,
            output=output,
            duration=args.duration,
            fps=args.fps,
            width=args.width,
            height=args.height,
        )
        cfg.documentation_case_report = str(args.case_report.expanduser().resolve())
        cfg.documentation_metadata = metadata
        pipeline_type = _recording_pipeline_type(case, args.duration, backend)
        from spark_pipeline import run_simulation_pipeline

        if case.policy_config == "UnitreeG1SonicPolicy":
            pipeline = _run_sonic_teleop(cfg, pipeline_type)
        else:
            pipeline = run_simulation_pipeline(cfg, backend=backend, pipeline_class=pipeline_type)
        del pipeline
    else:
        started = time.perf_counter()
        _run_benchmark(
            case,
            backend=backend,
            num_envs=num_envs,
            output=output,
            duration=args.duration,
            fps=args.fps,
            width=args.width,
            height=args.height,
        )
        if output.is_file():
            frame_count, gif_duration = _gif_properties(output)
            gif_bytes = output.stat().st_size
        else:
            frame_count, gif_duration, gif_bytes = 0, 0.0, 0
        expected_frames = int(math.floor(args.duration * args.fps))
        # GIF encoders merge consecutive identical frames and accumulate their
        # display duration.  Validate both useful temporal sampling and the
        # full real-time playback span instead of rejecting a mostly static
        # but correctly timed benchmark clip.
        passed = (
            output.is_file()
            and frame_count >= max(1, int(0.5 * expected_frames))
            and gif_duration >= 0.9 * args.duration
        )
        report = {
            **metadata,
            "status": "PASS" if passed else "FAIL",
            "wall_seconds": time.perf_counter() - started,
            "frame_count": frame_count,
            "gif_duration_seconds": gif_duration,
            "gif_bytes": gif_bytes,
        }
        args.case_report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    report = json.loads(args.case_report.read_text())
    if workflow == "teleop":
        frame_count, gif_duration = _gif_properties(output)
        report.update(
            frame_count=frame_count,
            gif_duration_seconds=gif_duration,
            gif_bytes=output.stat().st_size,
        )
        args.case_report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return 0 if report["status"] == "PASS" else 1


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--report", type=Path)
    parser.add_argument(
        "--profiles",
        "--robots",
        dest="profiles",
        nargs="+",
        choices=tuple(PROFILES),
    )
    parser.add_argument(
        "--families",
        nargs="+",
        choices=tuple(sorted({case.family for case in PROFILES.values()})),
    )
    parser.add_argument("--modes", nargs="+", choices=tuple(MODES))
    parser.add_argument("--duration", type=float, default=16.0)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--mujoco-python", type=Path, default=Path(sys.executable))
    parser.add_argument("--isaac-python", type=Path, default=Path(sys.executable))
    parser.add_argument("--clean", action="store_true")
    parser.add_argument("--keep-going", action="store_true")
    parser.add_argument(
        "--reuse-passing",
        action="store_true",
        help="Reuse passing case outputs and rerun only parent-side postprocessing.",
    )
    parser.add_argument("--run-case", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--profile",
        "--robot",
        dest="profile",
        choices=tuple(PROFILES),
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--mode", choices=tuple(MODES), help=argparse.SUPPRESS)
    parser.add_argument("--output", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--case-report", type=Path, help=argparse.SUPPRESS)
    return parser


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    if args.duration <= 0.0 or args.fps <= 0.0 or args.width < 1 or args.height < 1:
        raise SystemExit("duration, fps, width, and height must be positive")
    if args.run_case:
        if None in (args.profile, args.mode, args.output, args.case_report):
            raise SystemExit("internal case execution requires robot, mode, output, and report")
        return _run_inner(args)

    profiles = args.profiles or list(PROFILES)
    if args.families:
        selected_families = set(args.families)
        profiles = [
            profile for profile in profiles if PROFILES[profile].family in selected_families
        ]
    modes = args.modes or list(MODES)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.clean:
        for path in args.output_dir.glob("*.gif"):
            path.unlink()
        for path in args.output_dir.glob("*.json"):
            path.unlink()

    results = []
    for profile in profiles:
        for mode in modes:
            backend = MODES[mode][0]
            executable = args.isaac_python if backend == "isaac" else args.mujoco_python
            output = args.output_dir / f"{profile}__{mode}.gif"
            case_report = args.output_dir / f"{profile}__{mode}.json"
            command = [
                str(executable),
                "-B",
                str(Path(__file__).resolve()),
                "--run-case",
                "--profile",
                profile,
                "--mode",
                mode,
                "--output",
                str(output),
                "--case-report",
                str(case_report),
                "--duration",
                str(args.duration),
                "--fps",
                str(args.fps),
                "--width",
                str(args.width),
                "--height",
                str(args.height),
            ]
            print(f"[GIF] {profile} / {mode}", flush=True)
            reusable = False
            if args.reuse_passing and output.is_file() and case_report.is_file():
                previous = json.loads(case_report.read_text())
                reusable = previous.get("status") == "PASS"
            completed = None
            if reusable:
                print("[GIF] reusing passing simulation output", flush=True)
            else:
                completed = subprocess.run(command, cwd=REPO_ROOT, check=False)
            if case_report.is_file():
                result = json.loads(case_report.read_text())
            else:
                result = {
                    "status": "ERROR",
                    "robot": PROFILES[profile].family,
                    "profile": profile,
                    "mode": mode,
                    "returncode": completed.returncode if completed is not None else 1,
                }
            if output.is_file() and result.get("status") == "PASS":
                if not reusable:
                    _optimize_gif(output, args.fps, args.width, args.height)
                frame_count, gif_duration = _gif_properties(output)
                expected_frames = int(math.floor(args.duration * args.fps))
                try:
                    displayed_output = output.relative_to(REPO_ROOT)
                except ValueError:
                    displayed_output = output
                optimized_passed = (
                    frame_count >= max(1, int(0.9 * expected_frames))
                    and gif_duration >= 0.9 * args.duration
                )
                result.update(
                    frame_count=frame_count,
                    gif_duration_seconds=gif_duration,
                    gif_bytes=output.stat().st_size,
                    gif_path=str(displayed_output),
                    palette_optimized=True,
                    status="PASS" if optimized_passed else "FAIL",
                )
                case_report.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
            results.append(result)
            returncode = completed.returncode if completed is not None else 0
            if (returncode or result.get("status") != "PASS") and not args.keep_going:
                break
        else:
            continue
        break

    summary = {
        "schema_version": 1,
        "description": "SPARK v2 real-time documentation demonstrations",
        "expected_case_count": len(profiles) * len(modes),
        "case_count": len(results),
        "pass_count": sum(result.get("status") == "PASS" for result in results),
        "duration_seconds": args.duration,
        "fps": args.fps,
        "resolution": [args.width, args.height],
        "results": results,
    }
    report = args.report or args.output_dir / "manifest.json"
    report.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    passed = summary["pass_count"] == summary["expected_case_count"]
    if passed:
        # Per-case reports support interrupted-run recovery, but the aggregate
        # manifest is the only release artifact.  Keep failed/interrupted
        # reports for --reuse-passing and clean them after a complete run.
        for profile in profiles:
            for mode in modes:
                case_report = args.output_dir / f"{profile}__{mode}.json"
                if case_report != report:
                    case_report.unlink(missing_ok=True)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
