import os

import numpy as np
import pinocchio as pin

from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.kinematics import build_geometry_from_mjcf
from spark_robot.unitree_g1.kinematics.unitree_g1_fixed_base_kinematics import (
    UnitreeG1FixedBaseKinematics,
)


class UnitreeG1DualArmKinematics(UnitreeG1FixedBaseKinematics):
    """Dual-arm IK implemented as fixed-base G1 IK with waist joints locked.

    The public state remains 14 DoFs: left arm followed by right arm. Internally
    the Pinocchio/CasADi model is the same fixed-base model used by
    UnitreeG1FixedBaseKinematics, reduced by locking waist yaw/roll/pitch at
    the reference configuration. This keeps the dual-arm IK numerically aligned
    with the fixed-base IK instead of using a separate dual-arm XML/objective.
    """

    _WAIST_JOINTS_TO_LOCK = (
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
    )

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)
        self.load_collision_geometry = bool(kwargs.get("load_collision_geometry", True))
        if self.load_collision_geometry:
            self._init_collision_model()

    def _valid_joints_to_lock(self, model):
        lock_joints = [*self.mixed_jointsToLockIDs, *self._WAIST_JOINTS_TO_LOCK]
        valid_joints = []
        seen = set()
        for joint in lock_joints:
            if isinstance(joint, str):
                if not model.existJointName(joint):
                    continue
                joint_id = int(model.getJointId(joint))
            else:
                joint_id = int(joint)
                if joint_id >= model.njoints:
                    continue
            if joint_id in seen:
                continue
            seen.add(joint_id)
            valid_joints.append(joint_id)
        return valid_joints

    def add_extra_frames(self, model):
        model.addFrame(
            pin.Frame(
                "L_ee",
                model.getJointId("left_wrist_yaw_joint"),
                pin.SE3(
                    np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]),
                    np.array([0.1, 0.0, 0.0]).T,
                ),
                pin.FrameType.OP_FRAME,
            )
        )

        model.addFrame(
            pin.Frame(
                "R_ee",
                model.getJointId("right_wrist_yaw_joint"),
                pin.SE3(
                    np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]),
                    np.array([0.1, 0.0, 0.0]).T,
                ),
                pin.FrameType.OP_FRAME,
            )
        )

    def _init_collision_model(self):
        self.collision_model = build_geometry_from_mjcf(
            self.model,
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path),
            pin.GeometryType.COLLISION,
        )
        self.visual_model = build_geometry_from_mjcf(
            self.model,
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path),
            pin.GeometryType.VISUAL,
        )
        self.collision_model.addAllCollisionPairs()
        self.remove_adjacent_collision_pairs(self.model, self.collision_model)
        self.collision_data = self.collision_model.createData()

    def collision_free(self, full_q):
        pin.computeCollisions(
            self.model,
            self.data,
            self.collision_model,
            self.collision_data,
            np.asarray(full_q, dtype=float),
            True,
        )
        return not any(result.isCollision() for result in self.collision_data.collisionResults)

    def remove_adjacent_collision_pairs(self, model, collision_model):
        pairs_to_remove = []
        remove_name_elements = ["coupler", "spring", "follower", "driver", "pad", "gripper", "2f85"]
        for i, pair in enumerate(collision_model.collisionPairs):
            geom1 = collision_model.geometryObjects[pair.first]
            geom2 = collision_model.geometryObjects[pair.second]
            joint1 = geom1.parentJoint
            joint2 = geom2.parentJoint
            if model.parents[joint1] == joint2 or model.parents[joint2] == joint1:
                pairs_to_remove.append(i)
                continue
            for name in remove_name_elements:
                if name in geom1.name or name in geom2.name:
                    pairs_to_remove.append(i)
                    break
        for i in reversed(pairs_to_remove):
            collision_model.removeCollisionPair(collision_model.collisionPairs[i])


if __name__ == "__main__":
    from spark_robot import UnitreeG1DualArmDynamic1Config

    arm_ik = UnitreeG1DualArmKinematics(UnitreeG1DualArmDynamic1Config())
