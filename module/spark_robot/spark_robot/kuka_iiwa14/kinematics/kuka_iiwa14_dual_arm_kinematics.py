import numpy as np
import pinocchio as pin
import os

from spark_robot.base.base_robot_kinematics import RobotKinematics
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.kinematics import (
    build_geometry_from_mjcf,
    build_reduced_robot,
    build_robot_from_mjcf,
    configure_robot_ik,
    solve_robot_ik,
)
from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_utils import pos_quat_to_transformation, rpy2quat, transformation_to_pos_quat, quat2rpy
from scipy.spatial.transform import Rotation as R


class KukaIIWA14DualArmKinematics(RobotKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg)
        self.robot_cfg = robot_cfg
        self.mixed_jointsToLockIDs = self.robot_cfg.joint_to_lock
        self.kinematics_model_path = "kuka_iiwa14/iiwa14_2f85_dual.xml"
        self._init_fixed_base_kinematics()
        self._init_whole_body_kinematics()
        self._init_collision_model()

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
        self.robot = self.fixed_base_robot
        self.reduced_robot = self.reduced_fixed_base_robot

        self.model = self.reduced_robot.model
        self.data = pin.Data(self.model)

        self.pin_frame_dict = {}
        for frame in self.robot_cfg.Frames:
            for j in range(self.model.nframes):
                pin_frame = self.model.frames[j]
                pin_frame_id = self.model.getFrameId(pin_frame.name)
                if frame.name == pin_frame.name:
                    self.pin_frame_dict[frame.name] = pin_frame_id

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
        self.visualizer = None
        self.collision_model.addAllCollisionPairs()
        self.remove_adjacent_collision_pairs(self.model, self.collision_model)
        self.collision_data = self.collision_model.createData()
        return

    def visualize(self, dof):
        if self.visualizer is None:
            try:
                import meshcat
                from pinocchio.visualize import MeshcatVisualizer
            except ImportError as exc:
                raise RuntimeError(
                    "KUKA collision visualization requires the optional meshcat package"
                ) from exc
            self.visualizer = MeshcatVisualizer(
                self.model,
                self.collision_model,
                self.visual_model,
                meshcat.Visualizer(),
            )
            self.visualizer.initViewer(open=False)
            self.visualizer.loadViewerModel()
            self.visualizer.displayVisuals(False)
            self.visualizer.displayCollisions(True)
        self.visualizer.display(dof)
        return

    def add_extra_frames(self, model):
        model.addFrame(
            pin.Frame(
                "L_ee",
                model.getJointId("left/joint7"),
                pin.SE3(
                    np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]),
                    np.array([0.0, 0, 0.18]).T,
                ),
                pin.FrameType.OP_FRAME,
            )
        )

        self._add_configured_collision_frames(model)

    def _add_configured_collision_frames(self, model):
        """Attach configured spheres using the same link-local data as Isaac."""

        body_names = getattr(self.robot_cfg, "CollisionVolBodyNames", {})
        local_offsets = getattr(self.robot_cfg, "CollisionVolLocalOffsets", {})
        for frame in self.robot_cfg.CollisionVol:
            frame_name = frame.name
            if frame_name in {"R_ee", "L_ee"} or model.existFrame(frame_name):
                continue
            body_name = body_names.get(frame)
            if body_name is None:
                continue
            side, link_name = body_name.split("_", 1)
            joint_number = link_name.removeprefix("iiwa_link_")
            model.addFrame(
                pin.Frame(
                    frame_name,
                    model.getJointId(f"{side}/joint{joint_number}"),
                    pin.SE3(np.eye(3), np.asarray(local_offsets[frame], dtype=float)),
                    pin.FrameType.OP_FRAME,
                )
            )

        model.addFrame(
            pin.Frame(
                "R_ee",
                model.getJointId("right/joint7"),
                pin.SE3(
                    np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]),
                    np.array([0.0, 0, 0.18]).T,
                ),
                pin.FrameType.OP_FRAME,
            )
        )

    def collision_free(self, full_q):
        pin.computeCollisions(
            self.model, self.data, self.collision_model, self.collision_data, full_q, True
        )
        return not any(r.isCollision() for r in self.collision_data.collisionResults)

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
    arm_ik = KukaIIWA14DualArmKinematics()
