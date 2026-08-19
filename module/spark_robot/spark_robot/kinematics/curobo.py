"""Optional cuRobo GPU IK adapter.

cuRobo remains an optional dependency: importing :mod:`spark_robot` does not
import CUDA or allocate GPU memory.  Robot-specific integrations can construct
this adapter from a cuRobo robot YAML or configuration dictionary.
"""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import Any

import numpy as np


@dataclass(frozen=True)
class CuroboConfig:
    robot_config: str | dict[str, Any]
    position_threshold: float = 0.002
    rotation_threshold: float = 0.02
    num_seeds: int = 32
    max_batch_size: int = 1024
    use_cuda_graph: bool = True


class CuroboIK:
    """Batched GPU IK with lazy imports and an explicit availability check."""

    @staticmethod
    def available() -> bool:
        try:
            import torch
            import curobo  # noqa: F401
        except ImportError:
            return False
        return bool(torch.cuda.is_available())

    def __init__(self, config: CuroboConfig):
        if not self.available():
            raise RuntimeError(
                "cuRobo IK requires cuRobo, a CUDA-enabled PyTorch build, and an available NVIDIA GPU"
            )
        from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg

        solver_config = InverseKinematicsCfg.create(
            robot=config.robot_config,
            position_tolerance=config.position_threshold,
            orientation_tolerance=config.rotation_threshold,
            num_seeds=config.num_seeds,
            max_batch_size=config.max_batch_size,
            use_cuda_graph=config.use_cuda_graph,
        )
        self.solver = InverseKinematics(solver_config)

    def solve(self, positions: np.ndarray, quaternions_wxyz: np.ndarray):
        """Solve a batch of poses; the leading dimensions must match."""
        import torch
        from curobo.types import GoalToolPose, Pose

        position = torch.as_tensor(positions, device="cuda", dtype=torch.float32)
        quaternion = torch.as_tensor(quaternions_wxyz, device="cuda", dtype=torch.float32)
        if position.ndim == 1:
            position = position[None, :]
            quaternion = quaternion[None, :]
        pose = Pose(position=position, quaternion=quaternion)
        goal = GoalToolPose.from_poses({self.solver.tool_frames[0]: pose}, num_goalset=1)
        started = perf_counter()
        result = self.solver.solve_pose(goal)
        torch.cuda.synchronize()
        elapsed = perf_counter() - started
        return {
            "configuration": result.js_solution.position.detach().cpu().numpy(),
            "success": result.success.detach().cpu().numpy(),
            "position_error": result.position_error.detach().cpu().numpy(),
            "rotation_error": result.rotation_error.detach().cpu().numpy(),
            "solve_time": elapsed,
            "backend": "curobo_gpu",
            "raw_result": result,
        }


class UnitreeG1CuroboDualArmIK:
    """GPU dual-arm IK aligned with SPARK's 14-DoF G1 arm convention."""

    joint_names = [
        f"{side}_{joint}_joint"
        for side in ("left", "right")
        for joint in (
            "shoulder_pitch",
            "shoulder_roll",
            "shoulder_yaw",
            "elbow",
            "wrist_roll",
            "wrist_pitch",
            "wrist_yaw",
        )
    ]
    tool_frames = ["left_wrist_yaw_link", "right_wrist_yaw_link"]

    def __init__(
        self,
        *,
        max_batch_size: int,
        device: str = "cuda:0",
        num_seeds: int = 32,
        position_threshold: float = 0.002,
        rotation_threshold: float = 0.02,
        use_cuda_graph: bool = True,
        self_collision_check: bool = True,
    ) -> None:
        if not CuroboIK.available():
            raise RuntimeError("Unitree G1 batched IK requires CUDA-enabled cuRobo")
        import torch
        from curobo.content import get_robot_configs_path
        from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
        from curobo._src.util.config_io import load_yaml
        from curobo._src.types.device_cfg import DeviceCfg

        self.torch = torch
        self.device = torch.device(device)
        self.max_batch_size = int(max_batch_size)
        self.position_threshold = float(position_threshold)
        self.rotation_threshold = float(rotation_threshold)
        self.self_collision_check = bool(self_collision_check)
        robot = load_yaml(str(get_robot_configs_path() / "unitree_g1_29dof_retarget.yml"))
        kinematics = robot["kinematics"]
        kinematics["tool_frames"] = list(self.tool_frames)
        cspace = kinematics["cspace"]
        defaults = dict(zip(cspace["joint_names"], cspace["default_joint_position"]))
        active = set(self.joint_names)
        kinematics["lock_joints"] = {
            name: value for name, value in defaults.items() if name not in active
        }
        cfg = InverseKinematicsCfg.create(
            robot=robot,
            num_seeds=int(num_seeds),
            max_batch_size=self.max_batch_size,
            use_cuda_graph=bool(use_cuda_graph),
            self_collision_check=bool(self_collision_check),
            position_tolerance=self.position_threshold,
            orientation_tolerance=self.rotation_threshold,
            success_requires_convergence=False,
            device_cfg=DeviceCfg(device=self.device),
        )
        self.solver = InverseKinematics(cfg)
        if self.solver.joint_names != self.joint_names:
            raise RuntimeError(f"Unexpected cuRobo G1 joint order: {self.solver.joint_names}")

        offsets = torch.eye(4, device=self.device, dtype=torch.float32).repeat(2, 1, 1)
        offsets[0, :3, :3] = torch.tensor(
            [[1, 0, 0], [0, 0, 1], [0, -1, 0]], device=self.device, dtype=torch.float32
        )
        offsets[1, :3, :3] = torch.tensor(
            [[1, 0, 0], [0, 0, -1], [0, 1, 0]], device=self.device, dtype=torch.float32
        )
        offsets[:, :3, 3] = torch.tensor([0.1, 0.0, 0.0], device=self.device)
        self._inverse_tool_offsets = torch.linalg.inv(offsets)

    def solve(self, left_ee, right_ee, current_configuration):
        """Solve SPARK L_ee/R_ee targets and retain the seed for failed rows."""
        import time
        from curobo.types import GoalToolPose, JointState, Pose

        torch = self.torch
        left = torch.as_tensor(left_ee, device=self.device, dtype=torch.float32)
        right = torch.as_tensor(right_ee, device=self.device, dtype=torch.float32)
        current = torch.as_tensor(current_configuration, device=self.device, dtype=torch.float32)
        if left.ndim == 2:
            left, right, current = left[None], right[None], current[None]
        current = current.contiguous()
        requested_batch_size = left.shape[0]
        if requested_batch_size > self.max_batch_size:
            raise ValueError(
                f"IK batch {requested_batch_size} exceeds maximum {self.max_batch_size}"
            )
        if left.shape != (requested_batch_size, 4, 4) or right.shape != left.shape:
            raise ValueError("left_ee and right_ee must have shape (N, 4, 4)")
        if current.shape != (requested_batch_size, 14):
            raise ValueError("current_configuration must have shape (N, 14)")

        # cuRobo specializes kernels/CUDA graphs by batch shape. Per-env RL
        # resets naturally produce changing mask sizes, so pad with copies of
        # the last request and always execute the configured batch. Results
        # from padding rows are discarded below.
        if requested_batch_size < self.max_batch_size:
            pad = self.max_batch_size - requested_batch_size
            left = torch.cat((left, left[-1:].expand(pad, -1, -1)), dim=0).contiguous()
            right = torch.cat((right, right[-1:].expand(pad, -1, -1)), dim=0).contiguous()
            current = torch.cat((current, current[-1:].expand(pad, -1)), dim=0).contiguous()
        batch_size = self.max_batch_size

        wrists = torch.stack((left, right), dim=1) @ self._inverse_tool_offsets[None]
        goals = GoalToolPose.from_poses(
            {
                self.tool_frames[0]: Pose.from_matrix(wrists[:, 0]),
                self.tool_frames[1]: Pose.from_matrix(wrists[:, 1]),
            },
            ordered_tool_frames=self.tool_frames,
            num_goalset=1,
        )
        state = JointState.from_position(current, joint_names=self.joint_names)
        started = time.perf_counter()
        result = self.solver.solve_pose(goals, current_state=state, return_seeds=1)
        torch.cuda.synchronize(self.device)
        elapsed = time.perf_counter() - started
        solution = result.js_solution.position[:, 0, :14]
        position_error = result.position_error.reshape(batch_size, -1).amax(dim=1)
        rotation_error = result.rotation_error.reshape(batch_size, -1).amax(dim=1)
        success = (
            torch.isfinite(solution).all(dim=1)
            & (position_error <= self.position_threshold)
            & (rotation_error <= self.rotation_threshold)
        )
        solution = torch.where(success[:, None], solution, current)
        solution = solution[:requested_batch_size]
        success = success[:requested_batch_size]
        position_error = position_error[:requested_batch_size]
        rotation_error = rotation_error[:requested_batch_size]
        return {
            "configuration": solution,
            "success": success,
            "position_error": position_error,
            "rotation_error": rotation_error,
            "solve_time": elapsed,
            "backend": "curobo_gpu_unitree_g1_dual_arm",
            "raw_result": result,
        }

    def forward(self, configuration):
        """Return SPARK L_ee and R_ee matrices for a GPU joint batch."""
        from curobo.types import JointState

        current = self.torch.as_tensor(
            configuration, device=self.device, dtype=self.torch.float32
        ).contiguous()
        if current.ndim == 1:
            current = current[None]
        state = JointState.from_position(current, joint_names=self.joint_names)
        kinematics = self.solver.compute_kinematics(state)
        left_wrist = kinematics.tool_poses.get_link_pose(
            self.tool_frames[0], make_contiguous=True
        ).get_matrix()
        right_wrist = kinematics.tool_poses.get_link_pose(
            self.tool_frames[1], make_contiguous=True
        ).get_matrix()
        offsets = self.torch.linalg.inv(self._inverse_tool_offsets)
        return (
            left_wrist @ offsets[0],
            right_wrist @ offsets[1],
        )
