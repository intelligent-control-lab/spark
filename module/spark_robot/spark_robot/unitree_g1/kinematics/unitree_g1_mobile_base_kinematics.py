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
    BoundedLeastSquaresIK,
    IKProblem,
    IKSolverConfig,
    IKTarget,
)


class UnitreeG1MobileBaseKinematics(UnitreeG1FixedBaseKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)

    def _mobile_base_dof_names(self):
        names = ["LinearX", "LinearY"]
        if hasattr(self.robot_cfg.DoFs, "LinearZ"):
            names.append("LinearZ")
        names.append("RotYaw")
        return names

    def _init_fixed_base_kinematics(self):
        fixed_base_model_path = "unitree_g1/mjcf/g1_29dof_fixed_base.xml"
        self.fixed_base_robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, fixed_base_model_path)
        )
        self.add_extra_frames(self.fixed_base_robot.model)
        self.reduced_fixed_base_robot = build_reduced_robot(
            self.fixed_base_robot,
            list_of_joints_to_lock=[],
            reference_configuration=np.array([0.0] * self.fixed_base_robot.nq),
        )
        self.reduced_fixed_base_model = self.reduced_fixed_base_robot.model
        self.reduced_fixed_base_data = self.reduced_fixed_base_robot.data

        self.L_hand_id = self.reduced_fixed_base_model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_fixed_base_model.getFrameId("R_ee")
        self.init_data = np.zeros(self.reduced_fixed_base_model.nq)
        self.ik_solver = BoundedLeastSquaresIK(
            self.reduced_fixed_base_model,
            IKSolverConfig(
                regularization_weight=100.0,
                regularization_indices=(0, 1, 2),
                smoothness_weight=0.1,
                max_evaluations=20,
            ),
        )

    def _valid_joints_to_lock(self, model):
        mobile_joints_to_lock = [
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "left_hand_thumb_0_joint",
            "left_hand_thumb_1_joint",
            "left_hand_thumb_2_joint",
            "left_hand_index_0_joint",
            "left_hand_index_1_joint",
            "left_hand_middle_0_joint",
            "left_hand_middle_1_joint",
            "right_hand_thumb_0_joint",
            "right_hand_thumb_1_joint",
            "right_hand_thumb_2_joint",
            "right_hand_index_0_joint",
            "right_hand_index_1_joint",
            "right_hand_middle_0_joint",
            "right_hand_middle_1_joint",
        ]
        # ``build_robot_from_mjcf(..., jointComposite)`` replaces the MJCF's
        # floating root with the synthetic PX/PY[/PZ]/RZ joint used by this
        # abstraction.  Locking ``floating_base_joint`` here therefore locked
        # that newly-created planar root as well, leaving only the 17
        # upper-body coordinates while ``pre_computation`` still supplied the
        # base coordinates.  Keep the synthetic root in the reduced model.
        valid_joints = []
        for joint in [*self.mixed_jointsToLockIDs, *mobile_joints_to_lock]:
            if isinstance(joint, str) and model.existJointName(joint):
                valid_joints.append(model.getJointId(joint))
            elif not isinstance(joint, str) and int(joint) < model.njoints:
                valid_joints.append(int(joint))
        return valid_joints

    def _init_whole_body_kinematics(self):
        if "mobile_base" in os.path.basename(self.kinematics_model_path):
            self.robot = build_robot_from_mjcf(
                os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
            )
        else:
            # A whole-body MJCF already contains a free root.  Supplying a
            # Pinocchio root joint while parsing it does not replace that root
            # consistently across Pinocchio 3/4.  Build the abstract planar
            # base on the fixed-base articulation instead; legs are reduced
            # below and hands do not participate in this kinematic model.
            fixed_base_model_path = "unitree_g1/mjcf/g1_29dof_fixed_base.xml"
            jointComposite = pin.JointModelComposite(len(self._mobile_base_dof_names()))
            jointComposite.addJoint(pin.JointModelPX())
            jointComposite.addJoint(pin.JointModelPY())
            if hasattr(self.robot_cfg.DoFs, "LinearZ"):
                jointComposite.addJoint(pin.JointModelPZ())
            jointComposite.addJoint(pin.JointModelRZ())

            self.robot = build_robot_from_mjcf(
                os.path.join(SPARK_ROBOT_RESOURCE_DIR, fixed_base_model_path),
                jointComposite,
            )
        self.add_extra_frames(self.robot.model)
        joints_to_lock = self._valid_joints_to_lock(self.robot.model)
        if joints_to_lock:
            self.reduced_robot = build_reduced_robot(
                self.robot,
                list_of_joints_to_lock=joints_to_lock,
                reference_configuration=np.array([0.0] * self.robot.nq),
            )
        else:
            self.reduced_robot = self.robot

        self.model = self.reduced_robot.model
        self.data = pin.Data(self.model)

        self.pin_frame_dict = {}
        for frame in self.robot_cfg.Frames:
            for j in range(self.model.nframes):
                pin_frame = self.model.frames[j]
                if frame.name == pin_frame.name:
                    self.pin_frame_dict[frame.name] = j

    def inverse_kinematics(self, T, current_lr_arm_motor_q=None, current_lr_arm_motor_dq=None):
        right_wrist, left_wrist = T[0], T[1]
        if current_lr_arm_motor_q is not None:
            self.init_data = np.asarray(current_lr_arm_motor_q[:17], dtype=float)
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
        dof = np.zeros(self.num_dof)
        dof[: len(sol_q)] = sol_q
        info = {
            "sol_tauff": sol_tauff,
            "success": result.success,
            "ik_result": result,
            "ik_backend": result.backend,
        }
        return dof, info

    def pre_computation(self, dof, ddof=None):
        self._last_dof_pos = np.asarray(dof, dtype=float).reshape(-1).copy()
        base_dof_names = self._mobile_base_dof_names()
        base_dim = len(base_dof_names)
        q = np.zeros(self.model.nq)
        q[:base_dim] = [dof[getattr(self.robot_cfg.DoFs, name)] for name in base_dof_names]
        q[base_dim:] = dof[:17]
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        if ddof is not None:
            dq = np.zeros(self.model.nv)
            dq[:base_dim] = [ddof[getattr(self.robot_cfg.DoFs, name)] for name in base_dof_names]
            dq[base_dim:] = ddof[:17]
            pin.computeJointJacobiansTimeVariation(self.model, self.data, q, dq)
        return

    def _apply_planar_base_jacobian(self, J, pin_frame_id):
        dof = getattr(self, "_last_dof_pos", None)
        if dof is None:
            return J

        dofs = self.robot_cfg.DoFs.__members__
        frame_pos = self.data.oMf[pin_frame_id].translation

        if "LinearX" in dofs:
            J[:3, dofs["LinearX"]] = np.array([1.0, 0.0, 0.0])
        if "LinearY" in dofs:
            J[:3, dofs["LinearY"]] = np.array([0.0, 1.0, 0.0])
        if "LinearZ" in dofs:
            J[:3, dofs["LinearZ"]] = np.array([0.0, 0.0, 1.0])
        if "RotYaw" in dofs:
            origin = np.array(
                [
                    dof[dofs["LinearX"]] if "LinearX" in dofs else 0.0,
                    dof[dofs["LinearY"]] if "LinearY" in dofs else 0.0,
                    frame_pos[2],
                ]
            )
            J[:3, dofs["RotYaw"]] = np.cross(np.array([0.0, 0.0, 1.0]), frame_pos - origin)
        return J

    def forward_kinematics(self, dof):
        # in robot base frame
        self.pre_computation(dof)
        frames = self.get_forward_kinematics()
        base_id = next(
            j
            for j, frame in enumerate(self.model.frames)
            if frame.name == "robot" and frame.type == pin.BODY
        )
        frames = np.linalg.inv(self.data.oMf[base_id].homogeneous) @ frames
        return frames

    def get_jacobian(self, frame_id):
        pin_frame_id = self.pin_frame_dict[frame_id]
        pin_J = pin.getFrameJacobian(self.model, self.data, pin_frame_id, pin.LOCAL_WORLD_ALIGNED)
        base_dof_names = self._mobile_base_dof_names()
        base_dim = len(base_dof_names)
        J = np.zeros((pin_J.shape[0], len(self.robot_cfg.DoFs)))
        J[:, :17] = pin_J[:, base_dim:]
        for pin_col, name in enumerate(base_dof_names):
            J[:, getattr(self.robot_cfg.DoFs, name)] = pin_J[:, pin_col]
        return self._apply_planar_base_jacobian(J, pin_frame_id)

    def get_jacobian_dot(self, frame_id):
        pin_frame_id = self.pin_frame_dict[frame_id]
        pin_dJ = pin.getFrameJacobianTimeVariation(
            self.model, self.data, pin_frame_id, pin.LOCAL_WORLD_ALIGNED
        )
        base_dof_names = self._mobile_base_dof_names()
        base_dim = len(base_dof_names)
        dJ = np.zeros((pin_dJ.shape[0], len(self.robot_cfg.DoFs)))
        dJ[:, :17] = pin_dJ[:, base_dim:]
        for pin_col, name in enumerate(base_dof_names):
            dJ[:, getattr(self.robot_cfg.DoFs, name)] = pin_dJ[:, pin_col]
        return dJ


if __name__ == "__main__":
    arm_ik = UnitreeG1MobileBaseKinematics()
