"""Second-order Unitree G1 whole-body configuration with articulated hands."""

import numpy as np

from spark_robot.unitree_g1.config.unitree_g1_whole_body_dynamic_2_config import (
    UnitreeG1WholeBodyDynamic2Config,
)
from spark_robot.unitree_g1.config.unitree_g1_whole_body_with_hand_dynamic_1_config import (
    _BODY_DOF_NAMES,
    _HAND_DOF_NAMES,
    _MODEL_DOFS,
    UnitreeG1WholeBodyWithHandDynamic1Config,
)


class UnitreeG1WholeBodyWithHandDynamic2Config(UnitreeG1WholeBodyDynamic2Config):
    """Floating-base acceleration model whose qpos matches the hand MJCF."""

    mujoco_model_path = "unitree_g1/mjcf/g1_29dof_whole_body_with_hand.xml"

    DoFs = _MODEL_DOFS
    DefaultDoFVal = {
        getattr(_MODEL_DOFS, name): UnitreeG1WholeBodyDynamic2Config.DefaultDoFVal[
            getattr(UnitreeG1WholeBodyDynamic2Config.DoFs, name)
        ]
        for name in (
            "LinearX",
            "LinearY",
            "LinearZ",
            "QuaternionW",
            "QuaternionX",
            "QuaternionY",
            "QuaternionZ",
            *_BODY_DOF_NAMES,
        )
    }
    DefaultDoFVal.update({getattr(_MODEL_DOFS, name): 0.0 for name in _HAND_DOF_NAMES})

    # Reuse the qpos/actuator layout already validated for this exact MJCF.
    MujocoDoFs = UnitreeG1WholeBodyWithHandDynamic1Config.MujocoDoFs
    MujocoMotors = UnitreeG1WholeBodyWithHandDynamic1Config.MujocoMotors
    MujocoDoF_to_DoF = UnitreeG1WholeBodyWithHandDynamic1Config.MujocoDoF_to_DoF
    DoF_to_MujocoDoF = UnitreeG1WholeBodyWithHandDynamic1Config.DoF_to_MujocoDoF
    DoF_to_MujocoVel = UnitreeG1WholeBodyWithHandDynamic1Config.DoF_to_MujocoVel
    MujocoMotorKps = UnitreeG1WholeBodyWithHandDynamic1Config.MujocoMotorKps
    MujocoMotorKds = UnitreeG1WholeBodyWithHandDynamic1Config.MujocoMotorKds
    MujocoMotor_to_Control = {
        motor: UnitreeG1WholeBodyDynamic2Config.MujocoMotor_to_Control[
            getattr(UnitreeG1WholeBodyDynamic2Config.MujocoMotors, motor.name)
        ]
        for motor in UnitreeG1WholeBodyWithHandDynamic1Config.MujocoMotors
    }

    def dynamics_g(self, state):
        """Map the 29 body accelerations around the passive hand velocities."""
        del state
        g_x = np.zeros((self.num_state, len(self.Control)))
        for motor, control in self.MujocoMotor_to_Control.items():
            qpos_index = int(getattr(self.DoFs, motor.name))
            qvel_index = qpos_index - 1
            g_x[self.num_dof + qvel_index, int(control)] = 1.0
        return g_x
