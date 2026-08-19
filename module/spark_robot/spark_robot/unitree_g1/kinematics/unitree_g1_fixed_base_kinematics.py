import numpy as np
import pinocchio as pin
import os

from spark_robot.base.base_robot_kinematics import RobotKinematics
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.kinematics import (
    build_reduced_robot,
    build_robot_from_mjcf,
    BoundedLeastSquaresIK,
    IKProblem,
    IKSolverConfig,
    IKTarget,
)
from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_utils import pos_quat_to_transformation, rpy2quat, transformation_to_pos_quat, quat2rpy
from scipy.spatial.transform import Rotation as R


class UnitreeG1FixedBaseKinematics(RobotKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg)
        self.robot_cfg = robot_cfg
        self.mixed_jointsToLockIDs = self.robot_cfg.joint_to_lock
        self.kinematics_model_path = getattr(
            self.robot_cfg,
            "kinematics_model_path",
            getattr(self.robot_cfg, "mujoco_model_path", "unitree_g1/mjcf/g1_29dof_fixed_base.xml"),
        )
        self._init_fixed_base_kinematics()
        self._init_whole_body_kinematics()

    def _valid_joints_to_lock(self, model):
        return [
            joint
            for joint in self.mixed_jointsToLockIDs
            if (isinstance(joint, str) and model.existJointName(joint))
            or (not isinstance(joint, str) and int(joint) < model.njoints)
        ]

    def _waist_regularization_indices(self, model):
        indices = []
        for joint_name in ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"):
            if not model.existJointName(joint_name):
                continue
            joint_id = model.getJointId(joint_name)
            start = model.idx_qs[joint_id]
            for offset in range(model.nqs[joint_id]):
                indices.append(start + offset)
        if indices:
            return indices
        if model.nq >= 17:
            return [0, 1, 2]
        return []

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

        self.L_hand_id = self.reduced_fixed_base_model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_fixed_base_model.getFrameId("R_ee")
        regularization_indices = self._waist_regularization_indices(self.reduced_fixed_base_model)
        self.init_data = np.zeros(self.reduced_fixed_base_model.nq)
        self.ik_solver = BoundedLeastSquaresIK(
            self.reduced_fixed_base_model,
            IKSolverConfig(
                regularization_weight=0.1,
                smoothness_weight=0.1,
                regularization_indices=tuple(regularization_indices),
                max_evaluations=40,
            ),
        )

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
                "L_ee",
                model.getJointId("left_wrist_yaw_joint"),
                pin.SE3(np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]), np.array([0.1, 0, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )

        model.addFrame(
            pin.Frame(
                "R_ee",
                model.getJointId("right_wrist_yaw_joint"),
                pin.SE3(np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]), np.array([0.1, 0, 0]).T),
                pin.FrameType.OP_FRAME,
            )
        )

        model.addFrame(
            pin.Frame(
                "torso_link_1",
                model.getJointId("waist_pitch_joint"),
                pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.1]).T),
                pin.FrameType.OP_FRAME,
            )
        )

        model.addFrame(
            pin.Frame(
                "torso_link_2",
                model.getJointId("waist_pitch_joint"),
                pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.2]).T),
                pin.FrameType.OP_FRAME,
            )
        )

        model.addFrame(
            pin.Frame(
                "torso_link_3",
                model.getJointId("waist_pitch_joint"),
                pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.4]).T),
                pin.FrameType.OP_FRAME,
            )
        )

        model.addFrame(
            pin.Frame(
                "pelvis_link_1",
                model.getJointId("waist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.0]).T),
                pin.FrameType.OP_FRAME,
            )
        )

        model.addFrame(
            pin.Frame(
                "pelvis_link_2",
                model.getJointId("waist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.0, 0.15, 0.0]).T),
                pin.FrameType.OP_FRAME,
            )
        )

        model.addFrame(
            pin.Frame(
                "pelvis_link_3",
                model.getJointId("waist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.0, -0.15, 0.0]).T),
                pin.FrameType.OP_FRAME,
            )
        )

    def update_base_frame(self, trans_world2base, dof):
        try:
            R = trans_world2base[:3, :3]
            current_yaw = np.arctan2(R[1, 0], R[0, 0])

            # Compute yaw difference
            delta_yaw = dof[self.robot_cfg.DoFs.RotYaw] - current_yaw

            # Create new yaw rotation matrix
            Rz_new = np.array(
                [
                    [np.cos(delta_yaw), -np.sin(delta_yaw), 0],
                    [np.sin(delta_yaw), np.cos(delta_yaw), 0],
                    [0, 0, 1],
                ]
            )

            # Compute new rotation matrix
            R_new = Rz_new @ R

            trans_world2base[:3, :3] = R_new
            trans_world2base[:2, 3] = (
                dof[self.robot_cfg.DoFs.LinearX],
                dof[self.robot_cfg.DoFs.LinearY],
            )
            if hasattr(self.robot_cfg.DoFs, "LinearZ"):
                trans_world2base[2, 3] = dof[self.robot_cfg.DoFs.LinearZ]
        except:
            return trans_world2base

        return trans_world2base

    def forward_kinematics(self, dof):
        # in robot base frame
        self.pre_computation(dof)
        frames = self.get_forward_kinematics()
        return frames

    def inverse_kinematics(self, T, current_lr_arm_motor_q=None, current_lr_arm_motor_dq=None):
        right_wrist, left_wrist = T[0], T[1]
        if current_lr_arm_motor_q is not None:
            self.init_data = np.asarray(current_lr_arm_motor_q, dtype=float)
        result = self.ik_solver.solve(
            IKProblem(
                targets=[
                    IKTarget(self.R_hand_id, right_wrist),
                    IKTarget(self.L_hand_id, left_wrist),
                ],
                initial_configuration=self.init_data,
                previous_configuration=self.init_data,
            )
        )
        sol_q = result.configuration
        self.init_data = sol_q
        v = np.zeros(self.reduced_fixed_base_model.nv)
        sol_tauff = pin.rnea(
            self.reduced_fixed_base_model,
            self.reduced_fixed_base_data,
            sol_q,
            v,
            np.zeros(self.reduced_fixed_base_model.nv),
        )
        sol_tauff = np.pad(sol_tauff, (0, max(0, self.num_dof - len(sol_tauff))))
        info = {
            "sol_tauff": sol_tauff,
            "success": result.success,
            "ik_result": result,
            "ik_backend": result.backend,
        }
        return sol_q, info

    def pre_computation(self, dof_pos, dof_vel=None):
        q = dof_pos
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        if dof_vel is not None:
            dq = dof_vel
            pin.computeJointJacobiansTimeVariation(self.model, self.data, q, dq)
        return

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

    def get_forward_kinematics(self):
        frames = np.zeros((len(self.robot_cfg.Frames), 4, 4))
        for frame in self.robot_cfg.Frames:
            pin_frame = self.pin_frame_dict[frame.name]
            frames[frame] = self.data.oMf[pin_frame].homogeneous
        return frames


if __name__ == "__main__":
    arm_ik = UnitreeG1FixedBaseKinematics()
