from enum import IntEnum

import numpy as np

from spark_robot.agibot_g1.config.agibot_g1_mobile_base_dynamic_1_config import (
    AgiBotG1MobileBaseDynamic1Config,
)


class AgiBotG1MobileBaseDynamic2Config(AgiBotG1MobileBaseDynamic1Config):
    """Acceleration-controlled, double-integrator AgiBot G1 configuration."""

    dynamics_variant = "double_integrator"
    dynamics_order = 2
    dynamics_is_linear = True
    conformance_position_kp = 20.0
    conformance_velocity_kd = 8.0

    class Control(IntEnum):
        aLiftBody = 0
        aBodyPitch = 1

        aLeftJoint1 = 2
        aLeftJoint2 = 3
        aLeftJoint3 = 4
        aLeftJoint4 = 5
        aLeftJoint5 = 6
        aLeftJoint6 = 7
        aLeftJoint7 = 8

        aRightJoint1 = 9
        aRightJoint2 = 10
        aRightJoint3 = 11
        aRightJoint4 = 12
        aRightJoint5 = 13
        aRightJoint6 = 14
        aRightJoint7 = 15

        aLinearX = 16
        aLinearY = 17
        aRotYaw = 18

    ControlLimit = {
        Control.aLiftBody: 10.0,
        Control.aBodyPitch: 10.0,
        Control.aLeftJoint1: 150.0,
        Control.aLeftJoint2: 150.0,
        Control.aLeftJoint3: 150.0,
        Control.aLeftJoint4: 150.0,
        Control.aLeftJoint5: 150.0,
        Control.aLeftJoint6: 150.0,
        Control.aLeftJoint7: 150.0,
        Control.aRightJoint1: 150.0,
        Control.aRightJoint2: 150.0,
        Control.aRightJoint3: 150.0,
        Control.aRightJoint4: 150.0,
        Control.aRightJoint5: 150.0,
        Control.aRightJoint6: 150.0,
        Control.aRightJoint7: 150.0,
        Control.aLinearX: 1.0,
        Control.aLinearY: 1.0,
        Control.aRotYaw: 3.0,
    }

    NormalControl = list(Control)
    WeakControl = []
    DelicateControl = []

    @property
    def num_state(self):
        return 2 * self.num_dof

    def compose_state_from_dof(self, dof_pos, dof_vel):
        return np.concatenate(
            [
                np.asarray(dof_pos, dtype=float).reshape(-1),
                np.asarray(dof_vel, dtype=float).reshape(-1),
            ]
        )

    def decompose_state_to_dof_pos(self, state):
        return np.asarray(state, dtype=float).reshape(-1)[: self.num_dof]

    def decompose_state_to_dof_vel(self, state):
        return np.asarray(state, dtype=float).reshape(-1)[self.num_dof :]

    def dynamics_f(self, state):
        state = np.asarray(state, dtype=float).reshape(-1, 1)
        position = state[: self.num_dof]
        velocity = state[self.num_dof :]
        f_x = np.zeros_like(state)
        f_x[: self.num_dof] = velocity

        yaw = float(position[self.DoFs.RotYaw, 0])
        vx_global = float(velocity[self.DoFs.LinearX, 0])
        vy_global = float(velocity[self.DoFs.LinearY, 0])
        yaw_rate = float(velocity[self.DoFs.RotYaw, 0])
        vx_local = vx_global * np.cos(yaw) + vy_global * np.sin(yaw)
        vy_local = -vx_global * np.sin(yaw) + vy_global * np.cos(yaw)
        f_x[self.num_dof + self.DoFs.LinearX, 0] = -vx_local * yaw_rate * np.sin(
            yaw
        ) - vy_local * yaw_rate * np.cos(yaw)
        f_x[self.num_dof + self.DoFs.LinearY, 0] = vx_local * yaw_rate * np.cos(
            yaw
        ) - vy_local * yaw_rate * np.sin(yaw)
        return f_x

    def dynamics_g(self, state):
        state = np.asarray(state, dtype=float).reshape(-1)
        g_x = np.zeros((self.num_state, len(self.Control)), dtype=float)
        g_x[self.num_dof :, :] = np.eye(self.num_dof)

        yaw = state[self.DoFs.RotYaw]
        g_x[self.num_dof + self.DoFs.LinearX, self.Control.aLinearX] = np.cos(yaw)
        g_x[self.num_dof + self.DoFs.LinearY, self.Control.aLinearX] = np.sin(yaw)
        g_x[self.num_dof + self.DoFs.LinearX, self.Control.aLinearY] = -np.sin(yaw)
        g_x[self.num_dof + self.DoFs.LinearY, self.Control.aLinearY] = np.cos(yaw)
        return g_x

    MujocoMotor_to_Control = {
        motor: int(control)
        for motor, control in AgiBotG1MobileBaseDynamic1Config.MujocoMotor_to_Control.items()
    }
    RealMotor_to_Control = {
        motor: int(control)
        for motor, control in AgiBotG1MobileBaseDynamic1Config.RealMotor_to_Control.items()
    }
