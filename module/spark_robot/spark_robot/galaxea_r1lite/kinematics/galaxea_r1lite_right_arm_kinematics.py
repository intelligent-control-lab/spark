import numpy as np
import os
import pinocchio as pin

from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.kinematics import (
    build_reduced_robot,
    build_robot_from_mjcf,
    configure_robot_ik,
    solve_robot_ik,
)
from spark_robot.galaxea_r1lite.kinematics.galaxea_r1lite_dual_arm_kinematics import (
    GalaxeaR1LiteDualArmKinematics,
)


class GalaxeaR1LiteRightArmKinematics(GalaxeaR1LiteDualArmKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)

    def _init_fixed_base_kinematics(self):
        self.fixed_base_robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
        )
        self.add_extra_frames(self.fixed_base_robot.model)
        self.reduced_fixed_base_robot = build_reduced_robot(
            self.fixed_base_robot,
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.fixed_base_robot.nq),
        )
        self.reduced_fixed_base_model = self.reduced_fixed_base_robot.model
        self.reduced_fixed_base_data = self.reduced_fixed_base_robot.data

        self.R_hand_id = self.reduced_fixed_base_model.getFrameId("R_ee")
        self.init_data = np.zeros(self.reduced_fixed_base_model.nq)
        configure_robot_ik(
            self,
            position_weight=40.0,
            regularization_weight=0.02,
            smoothness_weight=0.1,
            max_evaluations=40,
        )
        return

    def add_extra_frames(self, model):
        ee_rotation = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        ee_translation = np.array([0.15, 0, 0.0]).T

        model.addFrame(
            pin.Frame(
                "R_ee",
                model.getJointId("right_arm_joint6"),
                pin.SE3(ee_rotation, ee_translation),
                pin.FrameType.OP_FRAME,
            )
        )
        self._add_configured_collision_frames(model)

    def inverse_kinematics(
        self,
        T,
        current_lr_arm_motor_q=None,
        current_lr_arm_motor_dq=None,
        target_options=None,
    ):
        sol_q, info = solve_robot_ik(
            self,
            [(self.R_hand_id, T[0])],
            current_lr_arm_motor_q,
            target_options=target_options,
        )
        dof = np.zeros(self.num_dof)
        dof[: len(sol_q)] = sol_q
        return dof, info
