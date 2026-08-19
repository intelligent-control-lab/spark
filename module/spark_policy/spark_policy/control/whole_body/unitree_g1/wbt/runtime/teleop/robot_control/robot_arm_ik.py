"""Numeric Pinocchio arm IK used by the legacy WBT teleoperation runtime.

Adapted and modified by the SPARK project from Apache-2.0 OpenWBT.
"""

from __future__ import annotations

import os

import numpy as np
import pinocchio as pin

from spark_robot.kinematics import (
    BoundedLeastSquaresIK,
    IKProblem,
    IKSolverConfig,
    IKTarget,
    build_reduced_robot,
)

from teleop.utils.weighted_moving_filter import WeightedMovingFilter


class _DualArmIK:
    urdf_path = ""
    unit_test_urdf_path = ""
    package_path = ""
    unit_test_package_path = ""
    joints_to_lock = ()

    def __init__(self, Unit_Test=False, Visualization=False):
        self.Unit_Test = Unit_Test
        self.Visualization = Visualization
        urdf_path = self.unit_test_urdf_path if Unit_Test else self.urdf_path
        package_path = self.unit_test_package_path if Unit_Test else self.package_path
        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_path)
        self.reduced_robot = build_reduced_robot(
            self.robot,
            self.joints_to_lock,
            np.zeros(self.robot.model.nq),
        )
        model = self.reduced_robot.model
        model.addFrame(
            pin.Frame(
                "L_ee",
                model.getJointId("left_wrist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.05, 0.0, 0.0])),
                pin.FrameType.OP_FRAME,
            )
        )
        model.addFrame(
            pin.Frame(
                "R_ee",
                model.getJointId("right_wrist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.05, 0.0, 0.0])),
                pin.FrameType.OP_FRAME,
            )
        )
        self.L_hand_id = model.getFrameId("L_ee")
        self.R_hand_id = model.getFrameId("R_ee")
        self.ik_solver = BoundedLeastSquaresIK(
            model,
            IKSolverConfig(
                regularization_weight=0.02,
                smoothness_weight=0.1,
                max_evaluations=50,
                function_tolerance=1e-8,
            ),
        )
        self.init_data = np.zeros(model.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), model.nq)
        self.vis = None
        if Visualization:
            from pinocchio.visualize import MeshcatVisualizer

            self.vis = MeshcatVisualizer(model)
            self.vis.initViewer(open=True)
            self.vis.loadViewerModel("pinocchio")
            self.vis.display(pin.neutral(model))

    def scale_arms(self, human_left_pose, human_right_pose, human_arm_length=0.60, robot_arm_length=0.75):
        scale_factor = robot_arm_length / human_arm_length
        robot_left_pose = human_left_pose.copy()
        robot_right_pose = human_right_pose.copy()
        robot_left_pose[:3, 3] *= scale_factor
        robot_right_pose[:3, 3] *= scale_factor
        return robot_left_pose, robot_right_pose

    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q=None, current_lr_arm_motor_dq=None):
        if current_lr_arm_motor_q is not None:
            self.init_data = np.asarray(current_lr_arm_motor_q, dtype=float).copy()
        result = self.ik_solver.solve(
            IKProblem(
                targets=[
                    IKTarget(self.L_hand_id, left_wrist, position_weight=50.0, orientation_weight=1.0),
                    IKTarget(self.R_hand_id, right_wrist, position_weight=50.0, orientation_weight=1.0),
                ],
                initial_configuration=self.init_data,
                previous_configuration=self.init_data,
            )
        )
        self.smooth_filter.add_data(result.configuration)
        sol_q = self.smooth_filter.filtered_data.copy()
        self.init_data = sol_q
        velocity = np.zeros(self.reduced_robot.model.nv)
        torque = pin.rnea(
            self.reduced_robot.model,
            self.reduced_robot.data,
            sol_q,
            velocity,
            np.zeros(self.reduced_robot.model.nv),
        )
        if self.vis is not None:
            self.vis.display(sol_q)
        return sol_q, torque


class G1_29_ArmIK(_DualArmIK):
    urdf_path = "deploy/assets/g1/g1_body29_hand14.urdf"
    unit_test_urdf_path = "../assets/g1/g1_body29_hand14.urdf"
    package_path = "deploy/assets/g1/"
    unit_test_package_path = "../assets/g1/"
    joints_to_lock = (
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
        "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
        "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", "left_hand_thumb_0_joint",
        "left_hand_thumb_1_joint", "left_hand_thumb_2_joint", "left_hand_middle_0_joint",
        "left_hand_middle_1_joint", "left_hand_index_0_joint", "left_hand_index_1_joint",
        "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
        "right_hand_index_0_joint", "right_hand_index_1_joint", "right_hand_middle_0_joint",
        "right_hand_middle_1_joint",
    )


class H1_2_ArmIK(_DualArmIK):
    urdf_path = "../assets/h1_2/h1_2.urdf"
    unit_test_urdf_path = "../../assets/h1_2/h1_2.urdf"
    package_path = "../assets/h1_2/"
    unit_test_package_path = "../../assets/h1_2/"
    joints_to_lock = (
        "left_hip_yaw_joint", "left_hip_pitch_joint", "left_hip_roll_joint", "left_knee_joint",
        "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_yaw_joint", "right_hip_pitch_joint",
        "right_hip_roll_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "torso_joint", "L_index_proximal_joint", "L_index_intermediate_joint", "L_middle_proximal_joint",
        "L_middle_intermediate_joint", "L_pinky_proximal_joint", "L_pinky_intermediate_joint",
        "L_ring_proximal_joint", "L_ring_intermediate_joint", "L_thumb_proximal_yaw_joint",
        "L_thumb_proximal_pitch_joint", "L_thumb_intermediate_joint", "L_thumb_distal_joint",
        "R_index_proximal_joint", "R_index_intermediate_joint", "R_middle_proximal_joint",
        "R_middle_intermediate_joint", "R_pinky_proximal_joint", "R_pinky_intermediate_joint",
        "R_ring_proximal_joint", "R_ring_intermediate_joint", "R_thumb_proximal_yaw_joint",
        "R_thumb_proximal_pitch_joint", "R_thumb_intermediate_joint", "R_thumb_distal_joint",
    )
