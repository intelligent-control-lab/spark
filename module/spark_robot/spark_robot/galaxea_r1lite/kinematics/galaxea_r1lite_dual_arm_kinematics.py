import numpy as np
import pinocchio as pin
import os

from spark_robot.base.base_robot_kinematics import RobotKinematics
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.kinematics import (
    build_reduced_robot,
    build_robot_from_mjcf,
    configure_robot_ik,
    solve_robot_ik,
)
from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_utils import pos_quat_to_transformation, rpy2quat, transformation_to_pos_quat, quat2rpy
from scipy.spatial.transform import Rotation as R


class GalaxeaR1LiteDualArmKinematics(RobotKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg)
        self.robot_cfg = robot_cfg
        self.mixed_jointsToLockIDs = self.robot_cfg.joint_to_lock
        self.kinematics_model_path = "galaxea_r1lite/r1lite_dual_arm.xml"
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
            reference_configuration=np.array([0.0] * self.fixed_base_robot.nq),
        )
        self.reduced_fixed_base_model = self.reduced_fixed_base_robot.model
        self.reduced_fixed_base_data = self.reduced_fixed_base_robot.data

        self.L_hand_id = self.reduced_fixed_base_model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_fixed_base_model.getFrameId("R_ee")
        self.init_data = np.zeros(self.reduced_fixed_base_model.nq)
        configure_robot_ik(
            self, regularization_weight=0.02, smoothness_weight=0.1, max_evaluations=20
        )
        return

    def _init_whole_body_kinematics(self):
        self.robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
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
            for j in range(self.model.nframes):
                pin_frame = self.model.frames[j]
                pin_frame_id = self.model.getFrameId(pin_frame.name, pin.FrameType.OP_FRAME)
                if frame.name == pin_frame.name:
                    self.pin_frame_dict[frame.name] = pin_frame_id

    def add_extra_frames(self, model):
        ee_rotation = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        ee_translation = np.array([0.15, 0, 0.0]).T

        model.addFrame(
            pin.Frame(
                "L_ee",
                model.getJointId("left_arm_joint6"),
                pin.SE3(ee_rotation, ee_translation),
                pin.FrameType.OP_FRAME,
            )
        )

        model.addFrame(
            pin.Frame(
                "R_ee",
                model.getJointId("right_arm_joint6"),
                pin.SE3(ee_rotation, ee_translation),
                pin.FrameType.OP_FRAME,
            )
        )

        self._add_configured_collision_frames(model)

    def _add_configured_collision_frames(self, model):
        body_names = getattr(self.robot_cfg, "CollisionVolBodyNames", {})
        local_offsets = getattr(self.robot_cfg, "CollisionVolLocalOffsets", {})
        for frame in self.robot_cfg.CollisionVol:
            frame_name = frame.name
            if frame_name in {"L_ee", "R_ee"} or model.existFrame(frame_name):
                continue
            body_name = body_names.get(frame)
            if body_name is None or not model.existFrame(body_name):
                continue
            body_frame = model.frames[model.getFrameId(body_name)]
            placement = body_frame.placement * pin.SE3(
                np.eye(3), np.asarray(local_offsets[frame], dtype=float)
            )
            model.addFrame(
                pin.Frame(
                    frame_name,
                    body_frame.parentJoint,
                    placement,
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
        except:
            return trans_world2base

        return trans_world2base

    def forward_kinematics(self, dof):
        # in robot base frame
        self.pre_computation(dof)
        frames = self.get_forward_kinematics()
        return frames

    def inverse_kinematics(
        self,
        T,
        current_lr_arm_motor_q=None,
        current_lr_arm_motor_dq=None,
        target_options=None,
    ):
        return solve_robot_ik(
            self,
            [(self.R_hand_id, T[0]), (self.L_hand_id, T[1])],
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
    from spark_robot.galaxea_r1lite.config.galaxea_r1lite_dual_arm_dynamic_1_config import (
        GalaxeaR1LiteDualArmDynamic1Config,
    )

    arm_ik = GalaxeaR1LiteDualArmKinematics(GalaxeaR1LiteDualArmDynamic1Config())
