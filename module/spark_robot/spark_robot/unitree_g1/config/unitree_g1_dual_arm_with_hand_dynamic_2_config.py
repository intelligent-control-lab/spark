from enum import IntEnum
import numpy as np
from spark_robot.unitree_g1.config.unitree_g1_dual_arm_with_hand_dynamic_1_config import (
    UnitreeG1DualArmWithHandDynamic1Config,
)


class UnitreeG1DualArmWithHandDynamic2Config(UnitreeG1DualArmWithHandDynamic1Config):
    dynamics_variant = "double_integrator"
    dynamics_order = 2
    dynamics_is_linear = True

    class Control(IntEnum):
        aLeftShoulderPitch = 0
        aLeftShoulderRoll = 1
        aLeftShoulderYaw = 2
        aLeftElbow = 3
        aLeftWristRoll = 4
        aLeftWristPitch = 5
        aLeftWristYaw = 6
        aRightShoulderPitch = 7
        aRightShoulderRoll = 8
        aRightShoulderYaw = 9
        aRightElbow = 10
        aRightWristRoll = 11
        aRightWristPitch = 12
        aRightWristYaw = 13

    ControlLimit = {control: 150.0 for control in Control}
    NormalControl = []
    WeakControl = [
        Control.aLeftShoulderPitch,
        Control.aLeftShoulderRoll,
        Control.aLeftShoulderYaw,
        Control.aLeftElbow,
        Control.aRightShoulderPitch,
        Control.aRightShoulderRoll,
        Control.aRightShoulderYaw,
        Control.aRightElbow,
    ]
    DelicateControl = [
        Control.aLeftWristRoll,
        Control.aLeftWristPitch,
        Control.aLeftWristYaw,
        Control.aRightWristRoll,
        Control.aRightWristPitch,
        Control.aRightWristYaw,
    ]

    @property
    def num_state(self):
        return int(len(self.DoFs) * 2)

    def compose_state_from_dof(self, dof_pos, dof_vel):
        return np.concatenate((dof_pos.reshape(-1), dof_vel.reshape(-1)), axis=0)

    def decompose_state_to_dof_pos(self, state):
        return state.reshape(-1)[: self.num_dof]

    def decompose_state_to_dof_vel(self, state):
        return state.reshape(-1)[self.num_dof :]

    def dynamics_f(self, state):
        f_x = np.zeros_like(state)
        f_x[: self.num_dof] = state[self.num_dof :]
        return f_x

    def dynamics_g(self, state):
        g_x = np.zeros((2 * self.num_dof, self.num_dof))
        g_x[self.num_dof :] = np.eye(self.num_dof)
        return g_x


UnitreeG1DualArmWithHandDynamic2Config.MujocoMotor_to_Control = {
    motor: getattr(
        UnitreeG1DualArmWithHandDynamic2Config.Control,
        "a" + UnitreeG1DualArmWithHandDynamic1Config.MujocoMotor_to_Control[motor].name[1:],
    )
    for motor in UnitreeG1DualArmWithHandDynamic1Config.MujocoMotors
}
