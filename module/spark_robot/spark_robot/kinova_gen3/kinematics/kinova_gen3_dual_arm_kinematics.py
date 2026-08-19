import os

import numpy as np
import pinocchio as pin

from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.kinematics import (
    build_reduced_robot,
    build_robot_from_mjcf,
    configure_robot_ik,
    solve_robot_ik,
)
from spark_robot.base.base_robot_kinematics import RobotKinematics


class KinovaGen3DualArmKinematics(RobotKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg)
        self.robot_cfg = robot_cfg
        self.mixed_jointsToLockIDs = self.robot_cfg.joint_to_lock
        self.kinematics_model_path = "kinova_gen3/gen3_2f85_dual.xml"
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
            self,
            regularization_weight=0.01,
            smoothness_weight=0.01,
            use_robot_config_position_limits=True,
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
            for idx in range(self.model.nframes):
                pin_frame = self.model.frames[idx]
                if frame.name == pin_frame.name:
                    self.pin_frame_dict[frame.name] = self.model.getFrameId(pin_frame.name)

    def add_extra_frames(self, model):
        frame_specs = (
            ("R_ee", "right/joint_7"),
            ("L_ee", "left/joint_7"),
        )
        for frame_name, joint_name in frame_specs:
            if model.existFrame(frame_name):
                continue
            model.addFrame(
                pin.Frame(
                    frame_name,
                    model.getJointId(joint_name),
                    pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.2]).T),
                    pin.FrameType.OP_FRAME,
                )
            )

        self._add_configured_collision_frames(model)

    def _add_configured_collision_frames(self, model):
        """Attach configured spheres using the same link-local data as Isaac."""

        body_names = getattr(self.robot_cfg, "CollisionVolBodyNames", {})
        local_offsets = getattr(self.robot_cfg, "CollisionVolLocalOffsets", {})
        link_to_joint = {
            "shoulder_link": "joint_1",
            "half_arm_1_link": "joint_2",
            "half_arm_2_link": "joint_3",
            "forearm_link": "joint_4",
            "spherical_wrist_1_link": "joint_5",
            "spherical_wrist_2_link": "joint_6",
            "bracelet_link": "joint_7",
        }
        mount_y = {"right": -0.35, "left": 0.35}
        for frame in self.robot_cfg.CollisionVol:
            frame_name = frame.name
            if frame_name in {"R_ee", "L_ee"} or model.existFrame(frame_name):
                continue
            body_name = body_names.get(frame)
            if body_name is None:
                continue
            side, link_name = body_name.split("_", 1)
            origin = np.asarray(local_offsets[frame], dtype=float)
            if link_name == "base_link":
                joint_id = model.getJointId("universe")
                origin = origin + np.array([0.0, mount_y[side], 0.0])
            else:
                joint_id = model.getJointId(f"{side}/{link_to_joint[link_name]}")
            model.addFrame(
                pin.Frame(
                    frame_name,
                    joint_id,
                    pin.SE3(np.eye(3), origin),
                    pin.FrameType.OP_FRAME,
                )
            )

    def update_base_frame(self, trans_world2base, dof):
        try:
            rotation = trans_world2base[:3, :3]
            current_yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
            delta_yaw = dof[self.robot_cfg.DoFs.RotYaw] - current_yaw
            yaw_rotation = np.array(
                [
                    [np.cos(delta_yaw), -np.sin(delta_yaw), 0.0],
                    [np.sin(delta_yaw), np.cos(delta_yaw), 0.0],
                    [0.0, 0.0, 1.0],
                ]
            )
            trans_world2base[:3, :3] = yaw_rotation @ rotation
            trans_world2base[:2, 3] = (
                dof[self.robot_cfg.DoFs.LinearX],
                dof[self.robot_cfg.DoFs.LinearY],
            )
        except Exception:
            return trans_world2base

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
            pin.computeJointJacobiansTimeVariation(self.model, self.data, q, dof_vel)

    def get_jacobian(self, frame_name):
        return pin.getFrameJacobian(
            self.model, self.data, self.model.getFrameId(frame_name), pin.LOCAL_WORLD_ALIGNED
        )

    def get_jacobian_dot(self, frame_name):
        return pin.getFrameJacobianTimeVariation(
            self.model, self.data, self.model.getFrameId(frame_name), pin.LOCAL_WORLD_ALIGNED
        )

    def get_forward_kinematics(self):
        frames = np.zeros((len(self.robot_cfg.Frames), 4, 4))
        for frame in self.robot_cfg.Frames:
            pin_frame = self.pin_frame_dict[frame.name]
            frames[frame] = self.data.oMf[pin_frame].homogeneous
        return frames
