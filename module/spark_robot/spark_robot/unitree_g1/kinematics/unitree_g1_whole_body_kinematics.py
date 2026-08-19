import numpy as np
import pinocchio as pin
from spark_robot.kinematics import build_robot_from_mjcf
import os
import warnings

from spark_robot.unitree_g1.kinematics.unitree_g1_fixed_base_kinematics import (
    UnitreeG1FixedBaseKinematics,
)
from spark_robot.unitree_g1.config.unitree_g1_whole_body_dynamic_1_config import (
    UnitreeG1WholeBodyDynamic1Config,
)
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot import SPARK_ROBOT_RESOURCE_DIR


class UnitreeG1WholeBodyKinematics(UnitreeG1FixedBaseKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)
        body_dofs = UnitreeG1WholeBodyDynamic1Config.DoFs
        self._body_position_indices = np.asarray(
            [int(getattr(self.robot_cfg.DoFs, dof.name)) for dof in body_dofs],
            dtype=int,
        )
        self._body_velocity_indices = np.asarray(
            [
                *range(6),
                *(int(getattr(self.robot_cfg.DoFs, dof.name)) - 1 for dof in list(body_dofs)[7:]),
            ],
            dtype=int,
        )

    def _pin_position(self, dof_pos):
        dof_pos = np.asarray(dof_pos, dtype=float).reshape(-1)
        if dof_pos.shape[0] == self.model.nq:
            return dof_pos.copy()
        if dof_pos.shape[0] != len(self.robot_cfg.DoFs):
            raise ValueError(
                f"Expected {len(self.robot_cfg.DoFs)} configured positions or "
                f"{self.model.nq} Pinocchio positions, got {dof_pos.shape[0]}"
            )
        return dof_pos[self._body_position_indices].copy()

    def _pin_velocity(self, dof_vel):
        dof_vel = np.asarray(dof_vel, dtype=float).reshape(-1)
        if dof_vel.shape[0] == self.model.nv:
            return dof_vel.copy()
        expected = len(self.robot_cfg.DoFs) - 1
        if dof_vel.shape[0] != expected:
            raise ValueError(
                f"Expected {expected} configured velocities or "
                f"{self.model.nv} Pinocchio velocities, got {dof_vel.shape[0]}"
            )
        return dof_vel[self._body_velocity_indices].copy()

    def _init_fixed_base_kinematics(self):
        original_model_path = self.kinematics_model_path
        self.kinematics_model_path = "unitree_g1/mjcf/g1_29dof_fixed_base.xml"
        super()._init_fixed_base_kinematics()
        self.kinematics_model_path = original_model_path

    def _init_whole_body_kinematics(self):
        self.kinematics_model_path = "unitree_g1/mjcf/g1_29dof_whole_body_fixed.xml"
        self.robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path),
            pin.JointModelFreeFlyer(),
        )

        self.model = self.robot.model
        self.add_extra_frames(self.model)
        self.data = pin.Data(self.model)
        self.pin_frame_dict = {}
        for frame in self.robot_cfg.Frames:
            for j in range(self.model.nframes):
                pin_frame = self.model.frames[j]
                pin_frame_id = self.model.getFrameId(pin_frame.name)
                if frame.name == pin_frame.name:
                    self.pin_frame_dict[frame.name] = pin_frame_id

    def pre_computation(self, dof_pos, dof_vel=None):
        pin_dof_pos = self._pin_position(dof_pos)
        quat = pin_dof_pos[3:7].copy()
        pin_quat = np.array([quat[1], quat[2], quat[3], quat[0]])
        pin_dof_pos[3:7] = pin_quat
        pin.forwardKinematics(self.model, self.data, pin_dof_pos)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        pin.computeCentroidalMap(self.model, self.data, pin_dof_pos)
        if dof_vel is not None:
            pin_dof_vel = self._pin_velocity(dof_vel)
            pin.computeJointJacobiansTimeVariation(self.model, self.data, pin_dof_pos, pin_dof_vel)
        return

    def inverse_kinematics(self, T, current_lr_arm_motor_q=None, current_lr_arm_motor_dq=None):
        ik_dof_pos = np.zeros(self.num_dof)
        if current_lr_arm_motor_q is not None:
            current_lr_arm_motor_q = current_lr_arm_motor_q[-17:]
        if current_lr_arm_motor_dq is not None:
            current_lr_arm_motor_dq = current_lr_arm_motor_dq[-17:]
        fixed_base_ik, info = super().inverse_kinematics(
            T, current_lr_arm_motor_q, current_lr_arm_motor_dq
        )
        ik_dof_pos[-17:] = fixed_base_ik
        return ik_dof_pos, info

    def forward_kinematics(self, dof):
        # in robot base frame
        self.pre_computation(dof)
        frames = self.get_forward_kinematics()
        base_id = self.model.getFrameId("robot")
        frames = np.linalg.inv(self.data.oMf[base_id].homogeneous) @ frames
        return frames

    def get_dynamics(self, dof_pos, dof_vel, contact_frames):
        pin_dof_pos = self._pin_position(dof_pos)
        pin_dof_vel = self._pin_velocity(dof_vel)
        quat = pin_dof_pos[3:7].copy()
        pin_quat = np.array([quat[1], quat[2], quat[3], quat[0]])
        pin_dof_pos[3:7] = pin_quat
        pin.forwardKinematics(self.model, self.data, pin_dof_pos)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, pin_dof_pos, pin_dof_vel)
        M = pin.crba(self.model, self.data, pin_dof_pos)  # Compute mass matrix
        nle = pin.nonLinearEffects(
            self.model, self.data, pin_dof_pos, pin_dof_vel
        )  # Compute Coriolis and gravity terms
        J_c = np.vstack([self.get_jacobian(frame_name)[:3, :] for frame_name in contact_frames])
        dJ_c = np.vstack(
            [self.get_jacobian_dot(frame_name)[:3, :] for frame_name in contact_frames]
        )
        frame_c = np.vstack(
            [self.get_frame_transformation(frame_name)[:3, 3] for frame_name in contact_frames]
        )

        self.compute_centroidal_dynamics(
            pin_dof_pos, pin_dof_vel, np.ones(self.robot.model.nv) * 0.0
        )

        return M, nle, J_c, dJ_c, frame_c

    def compute_centroidal_dynamics(self, q, v, a):
        """Compute centroidal dynamics."""
        pin.computeCentroidalMap(self.robot.model, self.robot.data, q)
        pin.computeCentroidalMomentumTimeVariation(self.robot.model, self.robot.data, q, v, a)

        Ag = self.robot.data.Ag.copy()  # Should now be filled
        dAg = (self.robot.data.dhg.vector[:, np.newaxis] - Ag @ a[:, np.newaxis]) @ np.linalg.pinv(
            v[:, np.newaxis]
        )
        if (Ag @ a + dAg @ v - self.robot.data.dhg.vector).sum() >= 1e-6:
            warnings.warn(
                "Centroidal dynamics computation failed; check the input values.",
                RuntimeWarning,
                stacklevel=2,
            )

        pin.computeCentroidalMomentum(self.robot.model, self.robot.data, q, v)
        pin.computeCentroidalMap(self.robot.model, self.robot.data, q)

        return Ag, dAg

    def get_jacobian(self, frame_name):
        """Compute the Jacobian of a given frame in the robot base frame"""
        return pin.getFrameJacobian(
            self.model, self.data, self.model.getFrameId(frame_name), pin.LOCAL_WORLD_ALIGNED
        )

    def get_jacobian_dot(self, frame_name):
        """Compute the Jacobian of a given frame in the robot base frame"""
        return pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.model.getFrameId(frame_name), pin.LOCAL_WORLD_ALIGNED
        )

    def get_frame_transformation(self, frame_name):
        """Compute the transformation matrix of a given frame in the robot base frame"""
        return self.data.oMf[self.model.getFrameId(frame_name)].homogeneous


if __name__ == "__main__":
    arm_ik = UnitreeG1WholeBodyKinematics()
