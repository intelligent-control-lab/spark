import numpy as np
import pinocchio as pin
import os

from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_robot.agibot_g1.kinematics.agibot_g1_fixed_base_kinematics import (
    AgiBotG1FixedBaseKinematics,
)
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.kinematics import build_reduced_robot, build_robot_from_mjcf, solve_robot_ik


class AgiBotG1MobileBaseKinematics(AgiBotG1FixedBaseKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)
        self.init_data = self.init_data[:-3].copy()

    def _init_whole_body_kinematics(self):
        joint_composite = pin.JointModelComposite(3)
        joint_composite.addJoint(pin.JointModelPX())
        joint_composite.addJoint(pin.JointModelPY())
        joint_composite.addJoint(pin.JointModelRZ())

        self.robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path),
            joint_composite,
        )

        self.reduced_robot = build_reduced_robot(
            self.robot,
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.nq),
        )

        self.model = self.reduced_robot.model
        self.add_extra_frames(self.model)
        self.data = pin.Data(self.model)

        self.pin_frame_dict = {}
        for frame in self.robot_cfg.Frames:
            pin_frame_id = self.model.getFrameId(frame.name)
            self.pin_frame_dict[frame.name] = pin_frame_id

    def inverse_kinematics(
        self,
        T,
        current_lr_arm_motor_q=None,
        current_lr_arm_motor_dq=None,
        target_options=None,
    ):
        arm_q = None
        if current_lr_arm_motor_q is not None:
            arm_q = np.asarray(current_lr_arm_motor_q)[:-3]
        sol_q, info = solve_robot_ik(
            self,
            [(self.R_hand_id, T[0]), (self.L_hand_id, T[1])],
            arm_q,
            target_options=target_options,
        )
        dof = np.zeros(self.num_dof)
        dof[:-3] = sol_q
        if current_lr_arm_motor_q is not None:
            dof[-3:] = np.asarray(current_lr_arm_motor_q)[-3:]
        return dof, info

    def inverse_kinematics_active_arm(
        self,
        T,
        current_lr_arm_motor_q=None,
        current_lr_arm_motor_dq=None,
        *,
        active_side,
        target_options=None,
    ):
        arm_q = None
        if current_lr_arm_motor_q is not None:
            arm_q = np.asarray(current_lr_arm_motor_q)[:-3]
        sol_q, info = super().inverse_kinematics_active_arm(
            T,
            arm_q,
            current_lr_arm_motor_dq,
            active_side=active_side,
            target_options=target_options,
        )
        dof = np.zeros(self.num_dof)
        dof[:-3] = sol_q
        if current_lr_arm_motor_q is not None:
            dof[-3:] = np.asarray(current_lr_arm_motor_q)[-3:]
        return dof, info

    def pre_computation(self, dof, ddof=None):
        q = np.zeros(self.model.nq)
        q[:3] = dof[-3:]
        q[3:] = dof[:-3]
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        if ddof is not None:
            dq = np.zeros(self.model.nv)
            dq[:3] = ddof[-3:]
            dq[3:] = ddof[:-3]
            pin.computeJointJacobiansTimeVariation(self.model, self.data, q, dq)

    def forward_kinematics(self, dof):
        self.pre_computation(dof)
        frames = self.get_forward_kinematics()
        base_id = self.model.getFrameId("robot")
        return np.linalg.inv(self.data.oMf[base_id].homogeneous) @ frames

    def get_jacobian(self, frame_id):
        pin_frame_id = self.pin_frame_dict[frame_id]
        pin_J = pin.getFrameJacobian(
            self.model,
            self.data,
            pin_frame_id,
            pin.LOCAL_WORLD_ALIGNED,
        )
        J = np.zeros_like(pin_J)
        J[:, :-3] = pin_J[:, 3:]
        J[:, -3:] = pin_J[:, :3]
        return J

    def get_jacobian_dot(self, frame_id):
        pin_frame_id = self.pin_frame_dict[frame_id]
        pin_dJ = pin.getFrameJacobianTimeVariation(
            self.model,
            self.data,
            pin_frame_id,
            pin.LOCAL_WORLD_ALIGNED,
        )
        dJ = np.zeros_like(pin_dJ)
        dJ[:, :-3] = pin_dJ[:, 3:]
        dJ[:, -3:] = pin_dJ[:, :3]
        return dJ

    def update_base_frame(self, trans_world2base, dof):
        try:
            rot = trans_world2base[:3, :3]
            current_yaw = np.arctan2(rot[1, 0], rot[0, 0])
            delta_yaw = dof[self.robot_cfg.DoFs.RotYaw] - current_yaw

            rot_z = np.array(
                [
                    [np.cos(delta_yaw), -np.sin(delta_yaw), 0.0],
                    [np.sin(delta_yaw), np.cos(delta_yaw), 0.0],
                    [0.0, 0.0, 1.0],
                ]
            )
            trans_world2base[:3, :3] = rot_z @ rot
            trans_world2base[:2, 3] = (
                dof[self.robot_cfg.DoFs.LinearX],
                dof[self.robot_cfg.DoFs.LinearY],
            )
        except Exception:
            return trans_world2base

        return trans_world2base
