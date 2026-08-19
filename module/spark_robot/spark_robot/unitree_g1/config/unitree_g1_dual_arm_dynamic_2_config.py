from enum import IntEnum

import numpy as np

from .unitree_g1_dual_arm_dynamic_1_config import UnitreeG1DualArmDynamic1Config


class UnitreeG1DualArmDynamic2Config(UnitreeG1DualArmDynamic1Config):
    """Acceleration-control dual-arm model used by D2 WBT safety layers."""

    dynamics_variant = "double_integrator"
    dynamics_order = 2

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
    WeakControl = list(Control)
    DelicateControl = []

    @property
    def num_state(self):
        return int(2 * len(self.DoFs))

    def compose_state_from_dof(self, dof_pos, dof_vel):
        return np.concatenate((np.ravel(dof_pos), np.ravel(dof_vel)))

    def decompose_state_to_dof_pos(self, state):
        return np.ravel(state)[: self.num_dof]

    def decompose_state_to_dof_vel(self, state):
        return np.ravel(state)[self.num_dof :]

    def dynamics_f(self, state):
        state = np.asarray(state)
        result = np.zeros_like(state)
        result[: self.num_dof] = state[self.num_dof :]
        return result

    def dynamics_g(self, state):
        result = np.zeros((2 * self.num_dof, self.num_dof))
        result[self.num_dof :] = np.eye(self.num_dof)
        return result
