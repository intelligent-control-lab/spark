import numpy as np
import pinocchio as pin
import os

from spark_robot.unitree_g1.kinematics.unitree_g1_fixed_base_kinematics import (
    UnitreeG1FixedBaseKinematics,
)
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_robot.kinematics import (
    build_reduced_robot,
    build_robot_from_mjcf,
    configure_robot_ik,
    solve_robot_ik,
)


class UnitreeG1RightArmKinematics(UnitreeG1FixedBaseKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)

    def _init_fixed_base_kinematics(self):
        self.fixed_base_robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
        )
        self.add_extra_frames(self.fixed_base_robot.model)
        self.reduced_fixed_base_robot = build_reduced_robot(
            self.fixed_base_robot,
            list_of_joints_to_lock=self._valid_joints_to_lock(self.fixed_base_robot.model),
            reference_configuration=np.array([0.0] * self.fixed_base_robot.nq),
        )
        self.reduced_fixed_base_model = self.reduced_fixed_base_robot.model
        self.reduced_fixed_base_data = self.reduced_fixed_base_robot.data

        self.R_hand_id = self.reduced_fixed_base_model.getFrameId("R_ee")
        self.init_data = np.zeros(self.reduced_fixed_base_model.nq)
        configure_robot_ik(
            self,
            position_weight=10.0,
            orientation_weight=20.0,
            regularization_weight=0.0,
            smoothness_weight=0.0,
            max_evaluations=20,
        )
        return

    def _init_whole_body_kinematics(self):
        self.robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
        )

        self.reduced_robot = build_reduced_robot(
            self.robot,
            list_of_joints_to_lock=self._valid_joints_to_lock(self.robot.model),
            reference_configuration=np.array([0.0] * self.robot.nq),
        )

        self.model = self.reduced_robot.model
        self.add_extra_frames(self.model)
        self.data = pin.Data(self.model)

        self.pin_frame_dict = {}
        for frame in self.robot_cfg.Frames:
            for j in range(self.model.nframes):
                pin_frame = self.model.frames[j]
                pin_frame_id = self.model.getFrameId(pin_frame.name)
                if frame.name == pin_frame.name:
                    self.pin_frame_dict[frame.name] = pin_frame_id

    def add_extra_frames(self, model):
        model.addFrame(
            pin.Frame(
                "R_ee",
                model.getJointId("right_wrist_yaw_joint"),
                pin.SE3(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]), np.array([0.1, 0, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )

    def inverse_kinematics(self, T, current_lr_arm_motor_q=None, current_lr_arm_motor_dq=None):
        sol_q, info = solve_robot_ik(self, [(self.R_hand_id, T[0])], current_lr_arm_motor_q)
        dof = np.zeros(self.num_dof)
        dof[: len(sol_q)] = sol_q
        return dof, info


if __name__ == "__main__":
    arm_ik = UnitreeG1RightArmKinematics()
