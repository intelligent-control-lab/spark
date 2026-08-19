"""Simulator-independent dynamics agent with an attachable renderer service."""

from __future__ import annotations

import numpy as np

from spark_agent.simulation.dynamics_model_agent import DynamicsModelAgent
from spark_robot.unitree_g1.actuation import UNITREE_G1_GRIPPER_SPECS
from spark_utils import Geometry, VizColor


class UnitreeG1DynamicsVisualizationAgent(DynamicsModelAgent):
    """Optional visualization/teleoperation adapter for model-only dynamics.

    Dynamics execution remains owned by :class:`DynamicsModelAgent`; this
    Isaac/Unitree adapter only adds debug state, hand commands, and transport
    to the asynchronous renderer process.
    """

    def __init__(self, robot_cfg, **kwargs):
        timing = robot_cfg.simulator_dynamics
        requested_dt = kwargs.get("dt")
        self.dt = float(timing.physics_dt if requested_dt is None else requested_dt)
        requested_decimation = kwargs.get("control_decimation")
        self.control_decimation = int(
            timing.control_decimation if requested_decimation is None else requested_decimation
        )
        super().__init__(
            robot_cfg,
            dt=self.dt * self.control_decimation,
            integrator=kwargs.get("model_integrator"),
            substeps=int(kwargs.get("model_substeps", 1)),
            seed=kwargs.get("dynamics_seed"),
        )
        self.real_time = bool(kwargs.get("real_time", True))
        self.enable_viewer = bool(kwargs.get("enable_viewer", True))
        self.enable_keyboard_control = bool(kwargs.get("enable_keyboard_control", False))
        self.enable_hand_control = bool(kwargs.get("enable_hand_control", False))
        self.robot_base_frame = np.eye(4, dtype=float)
        self.robot_base_frame[:3, 3] = np.asarray(
            kwargs.get("robot_base_position", (0.0, 0.0, 0.793)), dtype=float
        )

        debug = dict(kwargs.get("obstacle_debug") or {})
        self.num_obstacle_debug = int(debug.get("num_obstacle", 0))
        self.obstacle_debug_frame = np.repeat(
            np.eye(4, dtype=float)[None], self.num_obstacle_debug, axis=0
        )
        for frame in self.obstacle_debug_frame:
            frame[:3, 3] = np.array([0.6, 0.0, 0.793]) + np.random.uniform(-0.2, 0.2, 3)
        self.obstacle_debug_geom = [
            Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug)
            for _ in range(self.num_obstacle_debug)
        ]
        self.obstacle_debug_velocity = np.zeros((self.num_obstacle_debug, 6), dtype=float)
        self.left_goal_debug_frame = np.eye(4, dtype=float)
        self.right_goal_debug_frame = np.eye(4, dtype=float)
        self.base_goal_debug_frame = np.eye(4, dtype=float)
        self.left_gripper_goal = False
        self.right_gripper_goal = False
        self.left_gripper_debug_state = False
        self.right_gripper_debug_state = False
        self._debug_primitives = []
        self._renderer_process = None
        self.renderer_process_status = None

    def send_control(self, control, **kwargs):
        action_info = dict(kwargs.get("action_info") or {})
        super().send_control(control, **kwargs)
        if not self.enable_hand_control:
            return
        for side in ("left", "right"):
            goal_key = f"{side}_gripper_goal"
            control_key = f"{side}_gripper_control"
            if control_key in action_info:
                target = np.asarray(action_info[control_key], dtype=float).reshape(-1)
                open_target = UNITREE_G1_GRIPPER_SPECS[side].target(False)
                setattr(
                    self,
                    goal_key,
                    bool(np.linalg.norm(target - open_target) > 1e-6),
                )
            elif goal_key in action_info:
                setattr(self, goal_key, bool(action_info[goal_key]))
            elif side == "right" and "gripper_goal" in action_info:
                setattr(self, goal_key, bool(action_info["gripper_goal"]))
            setattr(self, f"{side}_gripper_debug_state", bool(getattr(self, goal_key)))

    def reset(self, agent_reset_info=None, **kwargs):
        super().reset(agent_reset_info, **kwargs)
        self._poll_renderer_input()

    def get_feedback(self):
        self._poll_renderer_input()
        feedback = super().get_feedback()
        feedback.update(
            {
                "robot_base_frame": self.robot_base_frame.copy(),
                "obstacle_debug_frame": self.obstacle_debug_frame.copy(),
                "obstacle_debug_geom": self.obstacle_debug_geom,
                "obstacle_debug_velocity": self.obstacle_debug_velocity.copy(),
                "robot_goal_left_offset": self.left_goal_debug_frame.copy(),
                "robot_goal_right_offset": self.right_goal_debug_frame.copy(),
                "robot_goal_base_offset": self.base_goal_debug_frame.copy(),
                "left_gripper_goal": self.left_gripper_goal,
                "right_gripper_goal": self.right_gripper_goal,
                "left_gripper_debug_state": self.left_gripper_debug_state,
                "right_gripper_debug_state": self.right_gripper_debug_state,
            }
        )
        return feedback

    def attach_renderer_process(self, renderer_process):
        self._renderer_process = renderer_process
        self._poll_renderer_input()

    def _poll_renderer_input(self):
        if self._renderer_process is None:
            return
        state = self._renderer_process.poll_input()
        if state is None:
            return
        self.obstacle_debug_frame = np.asarray(state["obstacle_debug_frame"], dtype=float).copy()
        self.obstacle_debug_geom = list(state["obstacle_debug_geom"])
        self.num_obstacle_debug = len(self.obstacle_debug_geom)
        self.obstacle_debug_velocity = np.zeros((self.num_obstacle_debug, 6), dtype=float)
        self.left_goal_debug_frame = np.asarray(state["robot_goal_left_offset"], dtype=float).copy()
        self.right_goal_debug_frame = np.asarray(
            state["robot_goal_right_offset"], dtype=float
        ).copy()
        self.base_goal_debug_frame = np.asarray(state["robot_goal_base_offset"], dtype=float).copy()
        for name in (
            "left_gripper_goal",
            "right_gripper_goal",
            "left_gripper_debug_state",
            "right_gripper_debug_state",
        ):
            setattr(self, name, bool(state[name]))

    def render(self):
        if self._renderer_process is None:
            self._debug_primitives = []
            return
        primitives = self._debug_primitives
        self._debug_primitives = []
        self._renderer_process.submit(
            {
                "sequence_id": self.dynamics_executor.step_index,
                "dof_pos": self.dof_pos_fbk.copy(),
                "robot_base_frame": self.robot_base_frame.copy(),
                "debug_primitives": primitives,
                "obstacle_debug_colors": [geom.color for geom in self.obstacle_debug_geom],
                "left_gripper_goal": bool(self.left_gripper_goal),
                "right_gripper_goal": bool(self.right_gripper_goal),
            }
        )

    def close_viewer(self):
        self.enable_viewer = False
        if self._renderer_process is not None:
            renderer = self._renderer_process
            self._renderer_process = None
            self.renderer_process_status = renderer.close(force=True)

    def close(self):
        self.close_viewer()

    def is_running(self) -> bool:
        """Propagate closure or failure of the isolated renderer process."""
        if self._renderer_process is None:
            return True
        return bool(self._renderer_process.is_alive)

    def render_sphere(self, pos, mat, size, color):
        self._debug_primitives.append(
            {
                "type": "sphere",
                "pos": np.asarray(pos, float),
                "size": np.asarray(size, float),
                "color": color,
            }
        )

    def render_box(self, pos, mat, size, color):
        self._debug_primitives.append(
            {
                "type": "box",
                "pos": np.asarray(pos, float),
                "mat": np.asarray(mat, float).reshape(3, 3),
                "size": np.asarray(size, float),
                "color": color,
            }
        )

    def render_line_segment(self, pos1, pos2, radius=0.002, color=VizColor.goal):
        self._debug_primitives.append(
            {
                "type": "line",
                "start": np.asarray(pos1, float),
                "end": np.asarray(pos2, float),
                "radius": float(radius),
                "color": color,
            }
        )

    def render_pixel_line_segment(self, pos1, pos2, width=2.0, color=VizColor.goal, **kwargs):
        self.render_line_segment(pos1, pos2, radius=max(float(width), 1.0) * 0.001, color=color)

    def render_coordinate_frame(self, frame, size=0.1):
        frame = np.asarray(frame, dtype=float).reshape(4, 4)
        origin = frame[:3, 3]
        self.render_line_segment(origin, origin + size * frame[:3, 0], color=VizColor.x_axis)
        self.render_line_segment(origin, origin + size * frame[:3, 1], color=VizColor.y_axis)
        self.render_line_segment(origin, origin + size * frame[:3, 2], color=VizColor.z_axis)

    def render_surface(self, triangles, color):
        for triangle in np.asarray(triangles, dtype=float).reshape(-1, 3, 3):
            self.render_line_segment(triangle[0], triangle[1], color=color)
            self.render_line_segment(triangle[1], triangle[2], color=color)
            self.render_line_segment(triangle[2], triangle[0], color=color)
