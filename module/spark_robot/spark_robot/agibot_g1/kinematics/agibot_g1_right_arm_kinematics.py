import os

import numpy as np
import pinocchio as pin

from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.base.base_robot_kinematics import RobotKinematics
from spark_robot.agibot_g1.kinematics.agibot_g1_fixed_base_kinematics import (
    AgiBotG1FixedBaseKinematics,
)
from spark_robot.kinematics import (
    build_reduced_robot,
    build_robot_from_mjcf,
    configure_robot_ik,
    solve_robot_ik,
)


class AgiBotG1RightArmKinematics(RobotKinematics):
    """AgiBot G1 fixed-base right-arm-only kinematics."""

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg)
        self.robot_cfg = robot_cfg
        self.mixed_jointsToLockIDs = self.robot_cfg.joint_to_lock
        self.kinematics_model_path = "agibot_g1/agibot_g1_fixed_base.xml"
        self._init_fixed_base_kinematics()
        self._init_whole_body_kinematics()

    def _init_fixed_base_kinematics(self):
        self.fixed_base_robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
        )
        self.add_extra_frames(self.fixed_base_robot.model)
        self.reduced_fixed_base_robot = build_reduced_robot(
            self.fixed_base_robot,
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=AgiBotG1FixedBaseKinematics._reference_configuration(
                self.fixed_base_robot.model
            ),
        )
        self.reduced_fixed_base_model = self.reduced_fixed_base_robot.model
        self.reduced_fixed_base_data = self.reduced_fixed_base_robot.data

        self.R_hand_id = self.reduced_fixed_base_model.getFrameId("R_ee")
        self.init_data = np.array(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs]
        )
        configure_robot_ik(
            self,
            position_weight=40.0,
            regularization_weight=0.02,
            smoothness_weight=0.1,
            max_evaluations=40,
        )
        return

    def _init_whole_body_kinematics(self):
        self.robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
        )
        self.reduced_robot = build_reduced_robot(
            self.robot,
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=AgiBotG1FixedBaseKinematics._reference_configuration(
                self.robot.model
            ),
        )

        self.model = self.reduced_robot.model
        self.add_extra_frames(self.model)
        self.data = pin.Data(self.model)

        self.pin_frame_dict = {}
        for frame in self.robot_cfg.Frames:
            pin_frame_id = self.model.getFrameId(frame.name)
            self.pin_frame_dict[frame.name] = pin_frame_id

    def add_extra_frames(self, model):
        AgiBotG1FixedBaseKinematics.add_extra_frames(self, model)

    def update_base_frame(self, trans_world2base, dof):
        return trans_world2base

    def forward_kinematics(self, dof):
        self.pre_computation(dof)
        return self.get_forward_kinematics()

    def inverse_kinematics(
        self,
        T,
        current_lr_arm_motor_q=None,
        current_lr_arm_motor_dq=None,
        target_options=None,
    ):
        return solve_robot_ik(
            self,
            [(self.R_hand_id, T[0])],
            current_lr_arm_motor_q,
            target_options=target_options,
        )

    def pre_computation(self, dof_pos, dof_vel=None):
        q = dof_pos
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        if dof_vel is not None:
            pin.computeJointJacobiansTimeVariation(self.model, self.data, q, dof_vel)

    def _frame_name(self, frame_name):
        return frame_name.name if hasattr(frame_name, "name") else frame_name

    def get_jacobian(self, frame_name):
        return pin.getFrameJacobian(
            self.model,
            self.data,
            self.model.getFrameId(self._frame_name(frame_name)),
            pin.LOCAL_WORLD_ALIGNED,
        )

    def get_jacobian_dot(self, frame_name):
        return pin.getFrameJacobianTimeVariation(
            self.model,
            self.data,
            self.model.getFrameId(self._frame_name(frame_name)),
            pin.LOCAL_WORLD_ALIGNED,
        )

    def get_forward_kinematics(self):
        frames = np.zeros((len(self.robot_cfg.Frames), 4, 4))
        for frame in self.robot_cfg.Frames:
            frames[frame] = self.data.oMf[self.pin_frame_dict[frame.name]].homogeneous
        return frames
