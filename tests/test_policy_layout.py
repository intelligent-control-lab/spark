import importlib
import unittest
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch

import spark_pipeline

from spark_policy.composed_policy.unitree_g1.sonic_safe import UnitreeG1SonicSafePolicy
from spark_policy.composed_policy.unitree_g1.wbt_safe import UnitreeG1WBTSafePolicy
from spark_policy.control.pid.benchmark import BenchmarkPIDPolicy
from spark_policy.control.pid.teleop import TeleopPIDPolicy
from spark_policy.control.whole_body.unitree_g1.sonic import UnitreeG1SonicPolicy
from spark_policy.control.whole_body.unitree_g1.sport import (
    UnitreeG1SportPolicy,
    UnitreeG1SportPolicyConfig,
)
from spark_policy.control.whole_body.unitree_g1.wbt import (
    UnitreeG1BatchedWBTPolicy,
    UnitreeG1WBTPolicy,
)
from spark_policy.estimation.filtering.kalman import KalmanFilterEstimator
from spark_policy.planning.trajectory.ilqr import ILQRPolicy
from spark_policy.safety.filtering.basic_cbf import BasicControlBarrierFunction
from spark_policy.safety.monitoring.collision.first_order import FirstOrderCollisionSafetyIndex
from spark_robot import UnitreeG1WholeBodyDynamic1Config


class PolicyLayoutTests(unittest.TestCase):
    @staticmethod
    def _load_unitree_benchmark_example():
        path = (
            Path(__file__).resolve().parents[1]
            / "example"
            / "unitree_g1"
            / "run_unitree_g1_benchmark.py"
        )
        spec = importlib.util.spec_from_file_location("run_unitree_g1_benchmark", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_canonical_classes_resolve_from_role_based_packages(self):
        expected_prefixes = {
            BenchmarkPIDPolicy: "spark_policy.control.pid.benchmark",
            TeleopPIDPolicy: "spark_policy.control.pid.teleop",
            UnitreeG1WBTPolicy: "spark_policy.control.whole_body.unitree_g1.wbt",
            UnitreeG1BatchedWBTPolicy: "spark_policy.control.whole_body.unitree_g1.wbt",
            UnitreeG1SportPolicy: "spark_policy.control.whole_body.unitree_g1.sport",
            UnitreeG1SonicPolicy: "spark_policy.control.whole_body.unitree_g1.sonic",
            UnitreeG1WBTSafePolicy: "spark_policy.composed_policy.unitree_g1.wbt_safe",
            UnitreeG1SonicSafePolicy: "spark_policy.composed_policy.unitree_g1.sonic_safe",
            KalmanFilterEstimator: "spark_policy.estimation.filtering.kalman",
            ILQRPolicy: "spark_policy.planning.trajectory.ilqr",
            BasicControlBarrierFunction: "spark_policy.safety.filtering.basic_cbf",
            FirstOrderCollisionSafetyIndex: "spark_policy.safety.monitoring.collision.first_order",
        }
        for implementation, prefix in expected_prefixes.items():
            with self.subTest(implementation=implementation.__name__):
                self.assertTrue(implementation.__module__.startswith(prefix))

    def test_every_canonical_policy_file_has_adjacent_config(self):
        package_root = Path(importlib.import_module("spark_policy").__file__).resolve().parent
        for role in ("control", "planning", "estimation", "safety", "composed_policy"):
            for policy_file in (package_root / role).rglob("policy.py"):
                with self.subTest(policy_file=policy_file):
                    self.assertTrue((policy_file.parent / "config.py").is_file())

    def test_pipeline_configs_expose_only_direct_policy_entry_point(self):
        for name in dir(spark_pipeline):
            config_class = getattr(spark_pipeline, name)
            if not isinstance(config_class, type) or not name.endswith("PipelineConfig"):
                continue
            if name == "BasePipelineConfig":
                continue
            config = config_class()
            with self.subTest(config=name):
                self.assertFalse(hasattr(config, "algo"))
                self.assertTrue(getattr(config.policy, "class_name", None))

    def test_obsolete_pipeline_level_algo_references_are_absent(self):
        repo_root = Path(__file__).resolve().parents[1]
        obsolete = ("cfg.algo", "self.algo", "class algo", "BasePipelineConfig.algo")
        for source in (repo_root / "pipeline").rglob("*.py"):
            text = source.read_text()
            with self.subTest(source=source):
                self.assertFalse(any(token in text for token in obsolete))

    def test_concrete_teleop_configs_directly_inherit_teleop_base(self):
        teleop_base = spark_pipeline.TeleopPipelineConfig
        for name in dir(spark_pipeline):
            config_class = getattr(spark_pipeline, name)
            if (
                not isinstance(config_class, type)
                or not name.endswith("TeleopPipelineConfig")
                or name == "TeleopPipelineConfig"
            ):
                continue
            with self.subTest(config=name):
                self.assertIn(teleop_base, config_class.__bases__)

    def test_agibot_keyboard_goals_are_offsets_from_current_grippers(self):
        for config_class in (
            spark_pipeline.AgiBotG1TeleopPipelineConfig,
            spark_pipeline.AgiBotG1DualArmTeleopPipelineConfig,
            spark_pipeline.AgiBotG1RightArmTeleopPipelineConfig,
        ):
            with self.subTest(config=config_class.__name__):
                self.assertTrue(config_class().env.task.arm_goal_init_from_current_ee)

    def test_unitree_runner_and_pipeline_observability_are_policy_neutral(self):
        repo_root = Path(__file__).resolve().parents[1]
        teleop_root = repo_root / "pipeline/spark_pipeline/teleop"
        self.assertEqual(list(teleop_root.glob("unitree_g1*runner.py")), [])
        pipeline_source = (
            repo_root / "pipeline/spark_pipeline/base/base_goal_pipeline.py"
        ).read_text()
        self.assertNotIn("self.policy.safety_monitor", pipeline_source)
        self.assertNotIn("self.policy.safety_filter", pipeline_source)

    def test_safety_composition_uses_nominal_controller_name(self):
        cfg = spark_pipeline.TeleopPipelineConfig()
        self.assertTrue(hasattr(cfg.policy, "nominal_controller"))
        self.assertFalse(hasattr(cfg.policy, "nominal"))

    def test_sport_tensor_policy_matches_legacy_model_and_batches(self):
        config = UnitreeG1SportPolicyConfig()
        legacy = torch.jit.load(config.model_path, map_location="cpu")
        policy = UnitreeG1SportPolicy(UnitreeG1WholeBodyDynamic1Config(), config=config, num_envs=4)
        observation = torch.randn(1, 47)
        expected = legacy(observation)
        memory, _ = policy.memory(
            observation.unsqueeze(0),
            (policy.hidden_state[:, :1], policy.cell_state[:, :1]),
        )
        torch.testing.assert_close(policy.actor(memory.squeeze(0)), expected, atol=1e-7, rtol=1e-7)

        q = torch.zeros(4, 29)
        target, info = policy.infer_tensor(
            body_joint_pos=q,
            body_joint_vel=q,
            root_quat_xyzw=torch.tensor([[0.0, 0.0, 0.0, 1.0]]).repeat(4, 1),
            root_angular_velocity=torch.zeros(4, 3),
            velocity_command=torch.zeros(4, 3),
        )
        self.assertEqual(target.shape, (4, 29))
        self.assertEqual(info["observation"].shape, (4, 47))

    def test_wbt_tensor_gru_matches_fixed_batch_onnx(self):
        policy = UnitreeG1BatchedWBTPolicy(
            UnitreeG1WholeBodyDynamic1Config(), num_envs=1, device="cpu"
        )
        rng = np.random.default_rng(7)
        observation = rng.normal(size=(1, 47)).astype(np.float32)
        hidden = rng.normal(size=(1, 1, 256)).astype(np.float32)
        policy.loco_network.hidden.copy_(torch.from_numpy(hidden))

        actual = policy.loco_network(torch.from_numpy(observation)).detach().numpy()
        session = ort.InferenceSession(
            policy.loco_config.policy_path, providers=["CPUExecutionProvider"]
        )
        expected, expected_hidden = session.run(
            None, {"obs": observation, "input_hidden_states": hidden}
        )
        np.testing.assert_allclose(actual, expected, atol=3e-6, rtol=3e-6)
        np.testing.assert_allclose(
            policy.loco_network.hidden.detach().numpy(),
            expected_hidden,
            atol=3e-6,
            rtol=3e-6,
        )

    def test_random_arm_benchmark_uses_visible_hand_model_without_obstacles(self):
        example = self._load_unitree_benchmark_example()
        cfg = example.build_config(**vars(example.build_parser().parse_args([])))

        self.assertEqual(cfg.robot.cfg.class_name, "UnitreeG1FixedBaseWithHandDynamic1Config")
        self.assertTrue(cfg.env.agent.enable_viewer)
        self.assertEqual(cfg.env.task.task_name, "arm_goal_static_v0")
        self.assertEqual(cfg.env.task.num_obstacle_task, 0)
        self.assertEqual(cfg.env.agent.obstacle_debug["num_obstacle"], 0)
        self.assertTrue(cfg.env.task.arm_goal_enable)
        self.assertTrue(cfg.env.task.use_dual_arm)
        self.assertFalse(cfg.env.task.base_goal_enable)
        self.assertEqual(cfg.policy.safe_controller.safe_algo.class_name, "ByPassSafeControl")
        self.assertFalse(cfg.metric_selection.dist_self)
        self.assertFalse(cfg.metric_selection.dist_robot_to_env)
        self.assertFalse(cfg.render_robot_collision_volumes)
        self.assertEqual(cfg.env.agent.viewer_config["ground_style"], "grid")
        self.assertEqual(cfg.env.agent.viewer_config["camera_lookat"], (0.5, 0.0, 1.3))
        self.assertEqual(cfg.env.agent.viewer_config["camera_azimuth"], 180.0)

    def test_legacy_benchmark_case_keeps_selected_safety_algorithm(self):
        example = self._load_unitree_benchmark_example()
        cfg = example.build_config(
            mode="test-case",
            test_case_name="UnitreeG1FixedBase_D1_AG_SO_v0",
            with_hand=False,
            safe_algo="rssa",
            safety_index="si1",
        )

        self.assertEqual(cfg.robot.cfg.class_name, "UnitreeG1FixedBaseDynamic1Config")
        self.assertEqual(cfg.env.task.num_obstacle_task, 5)
        self.assertEqual(
            cfg.policy.safe_controller.safe_algo.class_name,
            "RelaxedSafeSetAlgorithm",
        )
        self.assertEqual(len(cfg.policy.safe_controller.safe_algo.control_weight), 17)


if __name__ == "__main__":
    unittest.main()
