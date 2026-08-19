from contextlib import redirect_stdout
from io import StringIO
import unittest

import numpy as np
import spark_robot as sr


class CpuKinematicsTests(unittest.TestCase):
    """Smoke-test every exported CPU robot model through its public API."""

    KINEMATICS_CONFIGS = (
        (sr.UnitreeG1RightArmKinematics, sr.UnitreeG1RightArmDynamic1Config),
        (sr.UnitreeG1FixedBaseKinematics, sr.UnitreeG1FixedBaseDynamic1Config),
        (sr.UnitreeG1DualArmKinematics, sr.UnitreeG1DualArmDynamic1Config),
        (sr.UnitreeG1MobileBaseKinematics, sr.UnitreeG1MobileBaseDynamic1Config),
        (sr.UnitreeG1WholeBodyKinematics, sr.UnitreeG1WholeBodyDynamic1Config),
        (sr.KukaIIWA14SingleArmKinematics, sr.KukaIIWA14SingleArmDynamic1Config),
        (sr.KukaIIWA14DualArmKinematics, sr.KukaIIWA14DualArmDynamic1Config),
        (sr.GalaxeaR1LiteRightArmKinematics, sr.GalaxeaR1LiteRightArmDynamic1Config),
        (sr.GalaxeaR1LiteFixedBaseKinematics, sr.GalaxeaR1LiteFixedBaseDynamic1Config),
        (
            sr.GalaxeaR1LiteFixedBaseKinematics,
            sr.GalaxeaR1LiteFixedBaseDynamic1CollisionConfig,
        ),
        (sr.GalaxeaR1LiteDualArmKinematics, sr.GalaxeaR1LiteDualArmDynamic1Config),
        (sr.GalaxeaR1LiteMobileBaseKinematics, sr.GalaxeaR1LiteMobileBaseDynamic1Config),
        (sr.GalaxeaR1LiteFlatKinematics, sr.GalaxeaR1LiteFixedBaseDynamic1Config),
        (sr.FanucLRMate200iDSingleArmKinematics, sr.FanucLRMate200iDSingleArmDynamic1Config),
        (sr.FanucLRMate200iDDualArmKinematics, sr.FanucLRMate200iDDualArmDynamic1Config),
        (sr.KinovaGen3SingleArmKinematics, sr.KinovaGen3SingleArmDynamic1Config),
        (sr.KinovaGen3DualArmKinematics, sr.KinovaGen3DualArmDynamic1Config),
        (sr.AgiBotG1RightArmKinematics, sr.AgiBotG1RightArmDynamic1Config),
        (sr.AgiBotG1FixedBaseKinematics, sr.AgiBotG1FixedBaseDynamic1Config),
        (sr.AgiBotG1DualArmKinematics, sr.AgiBotG1DualArmDynamic1Config),
        (sr.AgiBotG1MobileBaseKinematics, sr.AgiBotG1MobileBaseDynamic1Config),
    )

    def test_every_cpu_kinematics_model_computes_finite_default_fk(self):
        for kinematics_type, config_type in self.KINEMATICS_CONFIGS:
            with self.subTest(kinematics=kinematics_type.__name__):
                config = config_type()
                output = StringIO()
                with redirect_stdout(output):
                    kinematics = kinematics_type(config)
                self.assertEqual(output.getvalue(), "")
                position = np.asarray(
                    [config.DefaultDoFVal[dof] for dof in config.DoFs], dtype=float
                )
                frames = np.asarray(kinematics.forward_kinematics(position))
                self.assertEqual(frames.shape, (len(config.Frames), 4, 4))
                self.assertTrue(np.isfinite(frames).all())

    def test_agibot_reduced_models_share_the_simulator_nominal_posture(self):
        configurations = (
            (sr.AgiBotG1FixedBaseKinematics, sr.AgiBotG1FixedBaseDynamic1Config),
            (sr.AgiBotG1DualArmKinematics, sr.AgiBotG1DualArmDynamic1Config),
            (sr.AgiBotG1RightArmKinematics, sr.AgiBotG1RightArmDynamic1Config),
        )
        hand_positions = []
        for kinematics_type, config_type in configurations:
            config = config_type()
            kinematics = kinematics_type(config)
            position = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs], dtype=float)
            frames = kinematics.forward_kinematics(position)
            hand_positions.append(
                np.stack(
                    [
                        frames[int(config.Frames.L_ee), :3, 3],
                        frames[int(config.Frames.R_ee), :3, 3],
                    ]
                )
            )

        np.testing.assert_allclose(hand_positions[1], hand_positions[0], atol=1e-10)
        np.testing.assert_allclose(hand_positions[2], hand_positions[0], atol=1e-10)

    def test_agibot_mobile_selected_arm_ik_keeps_other_arm_and_torso_stationary(self):
        config = sr.AgiBotG1MobileBaseDynamic1Config()
        kinematics = sr.AgiBotG1MobileBaseKinematics(config)
        current = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs], dtype=float)
        frames = kinematics.forward_kinematics(current)
        for active_side, target_index, active_slice, inactive_slice in (
            ("right", 0, slice(9, 16), slice(2, 9)),
            ("left", 1, slice(2, 9), slice(9, 16)),
        ):
            with self.subTest(active_side=active_side):
                goals = [
                    frames[int(config.Frames.R_ee)].copy(),
                    frames[int(config.Frames.L_ee)].copy(),
                ]
                goals[target_index][0, 3] += 0.05

                solution, _ = kinematics.inverse_kinematics_active_arm(
                    goals,
                    current,
                    active_side=active_side,
                )

                np.testing.assert_allclose(solution[:2], current[:2], atol=1e-10)
                np.testing.assert_allclose(
                    solution[inactive_slice], current[inactive_slice], atol=1e-10
                )
                np.testing.assert_allclose(solution[-3:], current[-3:], atol=1e-10)
                self.assertGreater(
                    np.linalg.norm(solution[active_slice] - current[active_slice]), 1e-3
                )

    def test_agibot_mobile_selected_arm_ik_recruits_lift_for_low_goal(self):
        config = sr.AgiBotG1MobileBaseDynamic1Config()
        kinematics = sr.AgiBotG1MobileBaseKinematics(config)
        current = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs], dtype=float)
        frames = kinematics.forward_kinematics(current)

        for active_side, target_index in (("right", 0), ("left", 1)):
            with self.subTest(active_side=active_side):
                goals = [
                    frames[int(config.Frames.R_ee)].copy(),
                    frames[int(config.Frames.L_ee)].copy(),
                ]
                goals[target_index][2, 3] -= 0.10
                solution, info = kinematics.inverse_kinematics_active_arm(
                    goals,
                    current,
                    active_side=active_side,
                )
                solved_frames = kinematics.forward_kinematics(solution)

                self.assertTrue(info["torso_assist"])
                self.assertLess(solution[int(config.DoFs.LiftBody)], current[0] - 0.05)
                np.testing.assert_allclose(
                    solution[int(config.DoFs.BodyPitch)],
                    current[int(config.DoFs.BodyPitch)],
                    atol=1e-10,
                )
                for frame, goal in (
                    (config.Frames.R_ee, goals[0]),
                    (config.Frames.L_ee, goals[1]),
                ):
                    self.assertLess(
                        np.linalg.norm(solved_frames[int(frame), :3, 3] - goal[:3, 3]),
                        0.01,
                    )

    def test_agibot_mobile_arm_teleop_owns_lift_until_base_is_selected(self):
        from spark_policy.control.pid.base import BasePIDPolicy

        config = sr.AgiBotG1MobileBaseDynamic1Config()
        kinematics = sr.AgiBotG1MobileBaseKinematics(config)
        policy = BasePIDPolicy(config, kinematics)
        current = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs], dtype=float)
        frames = kinematics.forward_kinematics(current)
        right_goal = frames[int(config.Frames.R_ee)].copy()
        left_goal = frames[int(config.Frames.L_ee)].copy()
        right_goal[2, 3] -= 0.10
        base_goal = np.eye(4)
        base_goal[2, 3] = 0.793
        feedback = {
            "dof_pos_cmd": current.copy(),
            "dof_vel_cmd": np.zeros_like(current),
            "dof_pos_fbk": current.copy(),
            "dof_vel_fbk": np.zeros_like(current),
            "robot_base_frame": np.eye(4),
        }
        task_info = {
            "goal_teleop": {
                "right": right_goal[None],
                "left": left_goal[None],
                "base": base_goal[None],
            },
            "arm_goal_enable": True,
            "base_goal_enable": True,
            "active_arm_goal": "right",
        }

        _, arm_info = policy.act(feedback, task_info)
        self.assertLess(
            arm_info["dof_pos_target"][int(config.DoFs.LiftBody)],
            current[int(config.DoFs.LiftBody)] - 0.05,
        )

        task_info["active_arm_goal"] = None
        task_info["arm_goal_enable"] = False
        _, base_info = policy.act(feedback, task_info)
        np.testing.assert_allclose(
            base_info["dof_pos_target"][int(config.DoFs.LiftBody)],
            current[int(config.DoFs.LiftBody)],
            atol=1e-10,
        )

    def test_r1lite_mobile_arm_ik_keeps_the_torso_posture(self):
        config = sr.GalaxeaR1LiteMobileBaseDynamic1Config()
        kinematics = sr.GalaxeaR1LiteMobileBaseKinematics(config)
        current = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs], dtype=float)
        current[3:6] = (0.08, -0.06, 0.04)
        frames = kinematics.forward_kinematics(current)

        solution, _ = kinematics.inverse_kinematics(
            [
                frames[int(config.Frames.R_ee)],
                frames[int(config.Frames.L_ee)],
            ],
            current,
            target_options=[
                {"orientation_mask": (False, False, False)},
                {"orientation_mask": (False, False, False)},
            ],
        )

        np.testing.assert_allclose(solution[3:6], current[3:6], atol=1e-8)

    def test_kinova_identity_goal_is_gripper_down_and_respects_joint_limits(self):
        config = sr.KinovaGen3SingleArmDynamic1Config()
        kinematics = sr.KinovaGen3SingleArmKinematics(config)
        current = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs], dtype=float)
        goal = np.eye(4)
        goal[:3, 3] = (0.4, 0.0, 0.3)

        solution, info = kinematics.inverse_kinematics([goal], current)
        end_effector = kinematics.forward_kinematics(solution)[int(config.Frames.R_ee)]

        for dof in config.DoFs:
            lower, upper = config.RealMotorPosLimit[config.RealMotors[dof.name]]
            self.assertGreaterEqual(solution[int(dof)], lower - 1e-9)
            self.assertLessEqual(solution[int(dof)], upper + 1e-9)
        self.assertLess(info["ik_result"].position_error, 0.01)
        self.assertLess(info["ik_result"].orientation_error, 0.01)
        np.testing.assert_allclose(end_effector[:3, :3], np.eye(3), atol=5e-3)
        # The gripper's flange maps its positive tool axis to bracelet -Z.
        tool_axis = end_effector[:3, :3] @ np.array([0.0, 0.0, -1.0])
        np.testing.assert_allclose(tool_axis, [0.0, 0.0, -1.0], atol=5e-3)


if __name__ == "__main__":
    unittest.main()
