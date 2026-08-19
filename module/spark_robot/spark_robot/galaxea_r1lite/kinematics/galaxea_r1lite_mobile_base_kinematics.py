import numpy as np
import pinocchio as pin
import os

from spark_robot.base.base_robot_kinematics import RobotKinematics
from spark_robot.galaxea_r1lite.kinematics.galaxea_r1lite_fixed_base_kinematics import (
    GalaxeaR1LiteFixedBaseKinematics,
)
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_robot.kinematics import (
    build_reduced_robot,
    build_robot_from_mjcf,
    configure_robot_ik,
    solve_robot_ik,
)
from spark_utils import pos_quat_to_transformation, rpy2quat, transformation_to_pos_quat, quat2rpy
from scipy.spatial.transform import Rotation as R


class GalaxeaR1LiteMobileBaseKinematics(GalaxeaR1LiteFixedBaseKinematics):
    _GRIPPER_JOINTS = (
        "left_gripper_finger_joint1",
        "left_gripper_finger_joint2",
        "right_gripper_finger_joint1",
        "right_gripper_finger_joint2",
    )

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)

    def _init_fixed_base_kinematics(self):
        original_locks = self.mixed_jointsToLockIDs
        self.mixed_jointsToLockIDs = [*original_locks, *self._GRIPPER_JOINTS]
        try:
            super()._init_fixed_base_kinematics()
        finally:
            self.mixed_jointsToLockIDs = original_locks

        # A planar mobile-base command moves the platform while the two arms
        # track their goals.  Do not let position-only arm IK bend the torso to
        # enlarge the sampled workspace: those poses make the physical robot
        # appear to fall over and are inconsistent with the stable upright
        # posture used by the mobile teleoperation workflow.  The fixed-base
        # kinematics deliberately remains unrestricted for whole-upper-body
        # tasks.
        torso_indices = tuple(
            self.reduced_fixed_base_model.joints[
                self.reduced_fixed_base_model.getJointId(name)
            ].idx_q
            for name in ("torso_joint1", "torso_joint2", "torso_joint3")
        )
        configure_robot_ik(
            self,
            regularization_weight=0.02,
            smoothness_weight=0.1,
            max_evaluations=20,
            fixed_indices=torso_indices,
        )

    def _init_whole_body_kinematics(self):
        jointComposite = pin.JointModelComposite(3)
        jointComposite.addJoint(pin.JointModelPX())
        jointComposite.addJoint(pin.JointModelPY())
        jointComposite.addJoint(pin.JointModelRZ())
        self.robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path), jointComposite
        )

        self.add_extra_frames(self.robot.model)
        lock_ids = [
            self.robot.model.getJointId(name)
            for name in self._GRIPPER_JOINTS
            if self.robot.model.existJointName(name)
        ]
        self.reduced_robot = build_reduced_robot(
            self.robot,
            list_of_joints_to_lock=lock_ids,
            reference_configuration=np.zeros(self.robot.nq),
        )
        self.model = self.reduced_robot.model
        # Reduction may discard operational frames attached to locked/static
        # parents, so attach the configured collision centers to the reduced
        # link frames before building the lookup table.
        self._add_configured_collision_frames(self.model)
        self.data = pin.Data(self.model)

        self.pin_frame_dict = {}
        for frame in self.robot_cfg.Frames:
            for j in range(self.model.nframes):
                pin_frame = self.model.frames[j]
                pin_frame_id = self.model.getFrameId(pin_frame.name)
                if frame.name == pin_frame.name:
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
            current = np.asarray(current_lr_arm_motor_q)
            arm_q = current[3:] if current.size == self.num_dof else current
        sol_q, info = solve_robot_ik(
            self,
            [(self.R_hand_id, T[0]), (self.L_hand_id, T[1])],
            arm_q,
            target_options=target_options,
        )
        dof = np.zeros(self.num_dof)
        dof[3:] = sol_q
        if (
            current_lr_arm_motor_q is not None
            and np.asarray(current_lr_arm_motor_q).size == self.num_dof
        ):
            dof[:3] = np.asarray(current_lr_arm_motor_q)[:3]
        return dof, info

    def pre_computation(self, dof, ddof=None):
        q = np.asarray(dof, dtype=float)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        if ddof is not None:
            dq = np.asarray(ddof, dtype=float)
            pin.computeJointJacobiansTimeVariation(self.model, self.data, q, dq)
        return

    def forward_kinematics(self, dof):
        # in robot base frame
        self.pre_computation(dof)
        frames = self.get_forward_kinematics()
        base_id = self.model.getFrameId("robot")
        frames = np.linalg.inv(self.data.oMf[base_id].homogeneous) @ frames
        return frames

    def get_jacobian(self, frame_id):
        pin_frame_id = self.pin_frame_dict[frame_id]
        pin_J = pin.getFrameJacobian(self.model, self.data, pin_frame_id, pin.LOCAL_WORLD_ALIGNED)
        return pin_J

    def get_jacobian_dot(self, frame_id):
        pin_frame_id = self.pin_frame_dict[frame_id]
        pin_dJ = pin.getFrameJacobianTimeVariation(
            self.model, self.data, pin_frame_id, pin.LOCAL_WORLD_ALIGNED
        )
        return pin_dJ


if __name__ == "__main__":
    from spark_robot.galaxea_r1lite.config.galaxea_r1lite_mobile_base_dynamic_1_config import (
        GalaxeaR1LiteMobileBaseDynamic1Config,
    )

    arm_ik = GalaxeaR1LiteMobileBaseKinematics(GalaxeaR1LiteMobileBaseDynamic1Config())
