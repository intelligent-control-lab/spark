import os

import numpy as np
import pinocchio as pin

from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.base.base_robot_kinematics import RobotKinematics
from spark_robot.agibot_g1.config.collision_geometry import (
    add_agibot_g1_collision_frames,
)
from spark_robot.agibot_g1.config.joint_defaults import (
    AGIBOT_G1_LOCKED_JOINT_DEFAULTS,
)
from spark_robot.kinematics import (
    build_reduced_robot,
    build_robot_from_mjcf,
    configure_robot_ik,
    solve_robot_ik,
)


class AgiBotG1FixedBaseKinematics(RobotKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg)
        self.robot_cfg = robot_cfg
        self.mixed_jointsToLockIDs = self.robot_cfg.joint_to_lock
        self.kinematics_model_path = "agibot_g1/agibot_g1_fixed_base.xml"
        self._init_fixed_base_kinematics()
        self._init_whole_body_kinematics()

    @staticmethod
    def _reference_configuration(model):
        """Match reduced-model locked joints to the simulated nominal posture."""
        reference = pin.neutral(model)
        for joint_name, value in AGIBOT_G1_LOCKED_JOINT_DEFAULTS.items():
            if not model.existJointName(joint_name):
                continue
            joint = model.joints[model.getJointId(joint_name)]
            if joint.nq == 1:
                reference[joint.idx_q] = value
        return reference

    def _init_fixed_base_kinematics(self):
        self.fixed_base_robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
        )
        self.add_extra_frames(self.fixed_base_robot.model)
        self.reduced_fixed_base_robot = build_reduced_robot(
            self.fixed_base_robot,
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=self._reference_configuration(self.fixed_base_robot.model),
        )
        self.reduced_fixed_base_model = self.reduced_fixed_base_robot.model
        self.reduced_fixed_base_data = self.reduced_fixed_base_robot.data

        self.L_hand_id = self.reduced_fixed_base_model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_fixed_base_model.getFrameId("R_ee")
        self.init_data = np.array(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs]
        )
        fixed_indices = ()
        if "LiftBody" in self.robot_cfg.DoFs.__members__:
            fixed_indices = (int(self.robot_cfg.DoFs.LiftBody),)
        configure_robot_ik(
            self,
            position_weight=40.0,
            regularization_weight=0.02,
            smoothness_weight=0.1,
            max_evaluations=40,
            fixed_indices=fixed_indices,
        )
        return

    def _init_whole_body_kinematics(self):
        self.robot = build_robot_from_mjcf(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
        )
        self.reduced_robot = build_reduced_robot(
            self.robot,
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=self._reference_configuration(self.robot.model),
        )

        self.model = self.reduced_robot.model
        self.add_extra_frames(self.model)
        self.data = pin.Data(self.model)

        self.pin_frame_dict = {}
        for frame in self.robot_cfg.Frames:
            pin_frame_id = self.model.getFrameId(frame.name)
            self.pin_frame_dict[frame.name] = pin_frame_id

    def add_extra_frames(self, model):
        ee_rotation = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
        # The grasp center is approximately 0.10 m past each wrist.  The
        # contributor URDF's 0.227/0.230 m helper frames lie well beyond the
        # 0.083 m gripper base and finger linkage, which made task goals appear
        # detached from the visible grippers in both simulators.
        for frame_name, source_name in (("L_ee", "Link7_l"), ("R_ee", "Link7_r")):
            if model.existFrame(frame_name):
                continue
            source_frame = model.frames[model.getFrameId(source_name)]
            model.addFrame(
                pin.Frame(
                    frame_name,
                    source_frame.parentJoint,
                    source_frame.placement * pin.SE3(ee_rotation, np.array([0.0, 0.0, 0.10])),
                    pin.FrameType.OP_FRAME,
                )
            )
        add_agibot_g1_collision_frames(model)

    def update_base_frame(self, trans_world2base, dof):
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

    def inverse_kinematics_active_arm(
        self,
        T,
        current_lr_arm_motor_q=None,
        current_lr_arm_motor_dq=None,
        *,
        active_side,
        target_options=None,
    ):
        """Solve one keyboard-selected arm and recruit the lift when necessary.

        The arm-only stage fixes the inactive arm and shared torso.  If the
        selected hand remains outside that workspace, a second dual-target
        solve unlocks ``LiftBody`` while keeping ``BodyPitch`` fixed.  Both
        arms can then compensate for the common vertical motion and continue
        tracking their own Cartesian targets.  Autonomous tasks still call
        :meth:`inverse_kinematics` and retain the full dual-arm solution.
        """

        if active_side not in ("right", "left"):
            raise ValueError(f"Unsupported active arm side: {active_side!r}")
        inactive_prefix = "Left" if active_side == "right" else "Right"
        shared_torso_names = {"LiftBody", "BodyPitch"}
        fixed_indices = tuple(
            int(dof)
            for name, dof in self.robot_cfg.DoFs.__members__.items()
            if (name.startswith(inactive_prefix) or name in shared_torso_names)
            and int(dof) < self.reduced_fixed_base_model.nq
        )
        target_index = 0 if active_side == "right" else 1
        hand_id = self.R_hand_id if active_side == "right" else self.L_hand_id
        selected_options = None
        if target_options is not None:
            selected_options = [target_options[target_index]]
        arm_only_solution, arm_only_info = solve_robot_ik(
            self,
            [(hand_id, T[target_index])],
            current_lr_arm_motor_q,
            target_options=selected_options,
            fixed_indices=fixed_indices,
        )
        arm_only_info["torso_assist"] = False
        arm_only_error = float(arm_only_info["ik_result"].position_error)
        if (
            arm_only_error <= 0.01
            or "LiftBody" not in self.robot_cfg.DoFs.__members__
            or current_lr_arm_motor_q is None
        ):
            return arm_only_solution, arm_only_info

        current_q = np.asarray(current_lr_arm_motor_q, dtype=float).copy()
        pin.framesForwardKinematics(
            self.reduced_fixed_base_model,
            self.reduced_fixed_base_data,
            current_q,
        )
        current_hand_z = float(self.reduced_fixed_base_data.oMf[hand_id].translation[2])
        requested_lift_delta = float(T[target_index][2, 3]) - current_hand_z
        if abs(requested_lift_delta) <= 0.01:
            return arm_only_solution, arm_only_info

        # Seed the shared lift at the requested vertical displacement.  The
        # common solver's configured LiftBody lock then holds this new height
        # while both arms compensate to retain their individual targets.
        lift_index = int(self.robot_cfg.DoFs.LiftBody)
        assisted_q = current_q.copy()
        lower = float(self.reduced_fixed_base_model.lowerPositionLimit[lift_index])
        upper = float(self.reduced_fixed_base_model.upperPositionLimit[lift_index])
        assisted_q[lift_index] = np.clip(
            assisted_q[lift_index] + requested_lift_delta,
            lower,
            upper,
        )

        # Body pitch also moves both shoulders but is not needed for vertical
        # workspace extension. Keep it fixed so the lift is the only shared
        # torso coordinate recruited by this fallback.
        torso_assist_fixed_indices = ()
        if "BodyPitch" in self.robot_cfg.DoFs.__members__:
            torso_assist_fixed_indices = (int(self.robot_cfg.DoFs.BodyPitch),)
        torso_target_options = []
        for target_index_option in range(2):
            option = dict(target_options[target_index_option]) if target_options is not None else {}
            option["position_weight"] = max(
                float(option.get("position_weight", self._ik_position_weight)),
                400.0,
            )
            torso_target_options.append(option)
        torso_solution, torso_info = solve_robot_ik(
            self,
            [(self.R_hand_id, T[0]), (self.L_hand_id, T[1])],
            assisted_q,
            target_options=torso_target_options,
            fixed_indices=torso_assist_fixed_indices,
        )
        # Recruit the lift only when it improves the worst Cartesian error
        # across both hands. This prevents a high target for one hand from
        # sacrificing an otherwise well-tracked inactive hand.
        if float(torso_info["ik_result"].position_error) >= arm_only_error:
            return arm_only_solution, arm_only_info
        torso_info["torso_assist"] = True
        torso_info["arm_only_ik_result"] = arm_only_info["ik_result"]
        return torso_solution, torso_info

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
            self.model,
            self.data,
            self.model.getFrameId(frame_name),
            pin.LOCAL_WORLD_ALIGNED,
        )

    def get_jacobian_dot(self, frame_name):
        return pin.getFrameJacobianTimeVariation(
            self.model,
            self.data,
            self.model.getFrameId(frame_name),
            pin.LOCAL_WORLD_ALIGNED,
        )

    def get_forward_kinematics(self):
        frames = np.zeros((len(self.robot_cfg.Frames), 4, 4))
        for frame in self.robot_cfg.Frames:
            frames[frame] = self.data.oMf[self.pin_frame_dict[frame.name]].homogeneous
        return frames
