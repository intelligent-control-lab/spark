"""Order-2 dynamic-bicycle model for the physical AgiBot G1 embodiment.

``Dynamic2`` means the complete standard robot configuration uses position and
velocity state with acceleration-like control.  This class inherits the
upper-body, simulator, and full-state contracts from
``AgiBotG1MobileBaseDynamic2Config`` and replaces only the planar base
derivative with a nonlinear dynamic-bicycle equation.  Its default reduced
analytical view follows the course/MATLAB convention
``(x, y, vx_body, vy_body, yaw_rate, yaw)``.

The two reduced base inputs are longitudinal acceleration and steering angle.
The inherited ``aRotYaw`` enum slot currently carries the steering angle; it is
not yaw acceleration in this model.  ``force_x = mass * acceleration`` is an
internal conversion used by the tire-force balance, not a force-valued public
control.  Front/rear lateral forces are generated from slip angles and
cornering stiffness.  A first-order/kinematic bicycle is not implemented here.
"""

from dataclasses import dataclass

import numpy as np

from .agibot_g1_mobile_base_dynamic_2_config import AgiBotG1MobileBaseDynamic2Config
from spark_robot.base.dynamics_model import RobotDynamicsModel


@dataclass(frozen=True)
class BicycleParams:
    """Tunable planar model parameters; defaults approximate a 150 kg G1."""

    mass: float = 150.0
    yaw_inertia: float = 30.0
    front_length: float = 0.30
    rear_length: float = 0.30
    front_cornering_stiffness: float = 4.5e3
    rear_cornering_stiffness: float = 4.5e3
    driven_front_fraction: float = 0.5
    minimum_forward_speed: float = 0.1
    steering_limit: float = 0.6


class _AgiBotG1MobileBaseBicycleDynamicsModel(RobotDynamicsModel):
    """Six-state MATLAB-order view over the full physical robot state."""

    _state_names = ("X", "Y", "v_x", "v_y", "r", "psi")

    def __init__(self, robot_cfg, state_dof_names=None, control_names=None):
        expected_state_dofs = tuple(robot_cfg.default_dynamics_state_dof_names)
        expected_controls = tuple(robot_cfg.default_dynamics_control_names)
        if state_dof_names is not None and tuple(state_dof_names) != expected_state_dofs:
            raise ValueError("The dynamic bicycle requires its complete planar state layout.")
        if control_names is not None and tuple(control_names) != expected_controls:
            raise ValueError("The dynamic bicycle requires acceleration followed by steering.")
        super().__init__(robot_cfg, expected_state_dofs, expected_controls)
        self.position_indices = (0, 1, 5)
        self.velocity_indices = (2, 3, 4)

    @property
    def state_names(self):
        return self._state_names

    def extract_state(self, dof_pos, dof_vel=None):
        if dof_vel is None:
            raise ValueError("The dynamic bicycle requires velocity feedback.")
        dof_pos = np.asarray(dof_pos, dtype=float).reshape(-1)
        dof_vel = np.asarray(dof_vel, dtype=float).reshape(-1)
        x = float(dof_pos[self.robot_cfg.DoFs.LinearX])
        y = float(dof_pos[self.robot_cfg.DoFs.LinearY])
        yaw = float(dof_pos[self.robot_cfg.DoFs.RotYaw])
        cosine, sine = np.cos(yaw), np.sin(yaw)
        vx_world = float(dof_vel[self.robot_cfg.DoFs.LinearX])
        vy_world = float(dof_vel[self.robot_cfg.DoFs.LinearY])
        vx_body = cosine * vx_world + sine * vy_world
        vy_body = -sine * vx_world + cosine * vy_world
        yaw_rate = float(dof_vel[self.robot_cfg.DoFs.RotYaw])
        return np.array([x, y, vx_body, vy_body, yaw_rate, yaw], dtype=float)

    def merge_state(self, state, into=None):
        state = np.asarray(state, dtype=float).reshape(self.state_dim)
        if into is not None and np.asarray(into).size == self.state_dim:
            return state.copy()
        if into is None:
            full_state = self._nominal_full_state()
        else:
            full_state = np.asarray(into, dtype=float).reshape(-1).copy()
            if full_state.size != self.robot_cfg.num_state:
                raise ValueError(
                    f"Cannot merge bicycle state into state of size {full_state.size}."
                )

        position = np.asarray(
            self.robot_cfg.decompose_state_to_dof_pos(full_state), dtype=float
        ).copy()
        velocity = np.asarray(
            self.robot_cfg.decompose_state_to_dof_vel(full_state), dtype=float
        ).copy()
        x, y, vx_body, vy_body, yaw_rate, yaw = state
        cosine, sine = np.cos(yaw), np.sin(yaw)
        position[self.robot_cfg.DoFs.LinearX] = x
        position[self.robot_cfg.DoFs.LinearY] = y
        position[self.robot_cfg.DoFs.RotYaw] = yaw
        velocity[self.robot_cfg.DoFs.LinearX] = cosine * vx_body - sine * vy_body
        velocity[self.robot_cfg.DoFs.LinearY] = sine * vx_body + cosine * vy_body
        velocity[self.robot_cfg.DoFs.RotYaw] = yaw_rate
        return np.asarray(
            self.robot_cfg.compose_state_from_dof(position, velocity), dtype=float
        ).reshape(self.robot_cfg.num_state)

    def expand_state(self, state):
        return self.merge_state(state)

    def _embed_state(self, reduced_state):
        return self.merge_state(reduced_state)

    def derivative(self, state, control, context=None):
        del context
        return self.robot_cfg.bicycle_state_derivative(state, control)


class AgiBotG1MobileBaseBicycleDynamic2Config(AgiBotG1MobileBaseDynamic2Config):
    """Physical AgiBot with a reduced six-state dynamic-bicycle model.

    The full robot retains the same arms, torso, kinematics, collision model,
    MuJoCo agent, and Isaac agent as the other mobile-base configurations. Its
    default dynamics view is ``(x, y, vx_body, vy_body, yaw_rate, yaw)``
    controlled by longitudinal acceleration and steering angle.
    """

    dynamics_variant = "bicycle"
    conformance_position_kp = 30.0
    conformance_velocity_kd = 10.0
    dynamics_order = 2
    dynamics_is_linear = False
    default_dynamics_state_dof_names = ("LinearX", "LinearY", "RotYaw")
    default_dynamics_control_names = ("aLinearX", "aRotYaw")
    ControlLimit = dict(AgiBotG1MobileBaseDynamic2Config.ControlLimit)
    ControlLimit[AgiBotG1MobileBaseDynamic2Config.Control.aLinearX] = 4.0
    ControlLimit[AgiBotG1MobileBaseDynamic2Config.Control.aRotYaw] = 0.6

    def __init__(self, bicycle_params=None):
        self.bicycle_params = BicycleParams() if bicycle_params is None else bicycle_params
        super().__init__()

    def create_dynamics_model(self, state_dof_names=None, control_names=None):
        return _AgiBotG1MobileBaseBicycleDynamicsModel(
            self,
            state_dof_names=state_dof_names,
            control_names=control_names,
        )

    def _body_frame_acceleration(self, vx, vy, yaw_rate, acceleration, steering):
        """Return ``(v_x_dot, v_y_dot, r_dot)`` in the vehicle body frame."""

        p = self.bicycle_params
        vx_effective = (
            np.copysign(max(abs(vx), p.minimum_forward_speed), vx)
            if vx != 0.0
            else p.minimum_forward_speed
        )
        steering = float(np.clip(steering, -p.steering_limit, p.steering_limit))
        force_x = p.mass * float(acceleration)
        alpha_front = np.arctan2(vy + p.front_length * yaw_rate, vx_effective) - steering
        alpha_rear = np.arctan2(vy - p.rear_length * yaw_rate, vx_effective)
        force_y_front = -p.front_cornering_stiffness * alpha_front
        force_y_rear = -p.rear_cornering_stiffness * alpha_rear
        force_x_front = p.driven_front_fraction * force_x
        force_x_rear = (1.0 - p.driven_front_fraction) * force_x

        dvx = (
            force_x_front * np.cos(steering) - force_y_front * np.sin(steering) + force_x_rear
        ) / p.mass + yaw_rate * vy
        dvy = (
            force_x_front * np.sin(steering) + force_y_front * np.cos(steering) + force_y_rear
        ) / p.mass - yaw_rate * vx
        d_yaw_rate = (
            p.front_length * (force_x_front * np.sin(steering) + force_y_front * np.cos(steering))
            - p.rear_length * force_y_rear
        ) / p.yaw_inertia
        return dvx, dvy, d_yaw_rate

    def bicycle_state_derivative(self, state, control):
        """Evaluate the six-state MATLAB-order analytical bicycle model."""

        x, y, vx, vy, yaw_rate, yaw = np.asarray(state, dtype=float).reshape(6)
        acceleration, steering = np.asarray(control, dtype=float).reshape(2)
        dvx, dvy, d_yaw_rate = self._body_frame_acceleration(
            vx, vy, yaw_rate, acceleration, steering
        )
        cosine, sine = np.cos(yaw), np.sin(yaw)
        return np.array(
            [
                vx * cosine - vy * sine,
                vx * sine + vy * cosine,
                dvx,
                dvy,
                d_yaw_rate,
                yaw_rate,
            ],
            dtype=float,
        )

    def dynamics_derivative(self, state, control, **kwargs):
        """Evaluate full-state dynamics while replacing the planar equations."""

        del kwargs
        state = np.asarray(state, dtype=float).reshape(self.num_state)
        control = np.asarray(control, dtype=float).reshape(len(self.Control))
        derivative = super().dynamics_xdot(state, control)
        position = state[: self.num_dof]
        velocity = state[self.num_dof :]
        yaw = float(position[self.DoFs.RotYaw])
        cosine, sine = np.cos(yaw), np.sin(yaw)
        vx_world = float(velocity[self.DoFs.LinearX])
        vy_world = float(velocity[self.DoFs.LinearY])
        yaw_rate = float(velocity[self.DoFs.RotYaw])
        vx = cosine * vx_world + sine * vy_world
        vy = -sine * vx_world + cosine * vy_world
        acceleration = float(control[self.Control.aLinearX])
        steering = float(control[self.Control.aRotYaw])
        dvx, dvy, d_yaw_rate = self._body_frame_acceleration(
            vx,
            vy,
            yaw_rate,
            acceleration,
            steering,
        )

        derivative[self.DoFs.LinearX] = vx_world
        derivative[self.DoFs.LinearY] = vy_world
        derivative[self.DoFs.RotYaw] = yaw_rate
        derivative[self.num_dof + self.DoFs.LinearX] = (
            cosine * dvx - sine * dvy - yaw_rate * (sine * vx + cosine * vy)
        )
        derivative[self.num_dof + self.DoFs.LinearY] = (
            sine * dvx + cosine * dvy + yaw_rate * (cosine * vx - sine * vy)
        )
        derivative[self.num_dof + self.DoFs.RotYaw] = d_yaw_rate
        return derivative
