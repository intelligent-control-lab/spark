"""Backend-independent, tensorized Unitree G1 Sport policy."""

from __future__ import annotations

from typing import Any

import numpy as np

from spark_policy.core.policy import BasePolicy

from .config import UnitreeG1SportPolicyConfig


class UnitreeG1SportPolicy(BasePolicy):
    """Run the legacy recurrent Sport network for one or many robots.

    The original implementation lived inside the MuJoCo agent and stored a
    fixed batch-size-one LSTM state.  This policy rebuilds the same network
    from its TorchScript weights, making the recurrent state explicitly
    batched and keeping simulation concerns in the agent.
    """

    def __init__(
        self,
        robot_cfg,
        robot_kinematics=None,
        *,
        config: UnitreeG1SportPolicyConfig | None = None,
        num_envs: int = 1,
        device: str = "cpu",
    ) -> None:
        super().__init__(robot_cfg, robot_kinematics)
        import torch

        self.torch = torch
        self.config = config or UnitreeG1SportPolicyConfig()
        if self.config.num_threads > 0 and str(device) == "cpu":
            torch.set_num_threads(int(self.config.num_threads))
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        source = torch.jit.load(self.config.model_path, map_location="cpu")

        self.memory = torch.nn.LSTM(47, 64).to(self.device)
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(64, 32),
            torch.nn.ELU(),
            torch.nn.Linear(32, 12),
        ).to(self.device)
        with torch.no_grad():
            self.memory.load_state_dict(source.memory.state_dict())
            self.actor.load_state_dict(source.actor.state_dict())
        self.memory.eval()
        self.actor.eval()

        self.default_lower = torch.tensor(
            self.config.default_lower_body_position,
            device=self.device,
            dtype=torch.float32,
        )
        self.command_scale = torch.tensor(
            self.config.command_scale, device=self.device, dtype=torch.float32
        )
        self.command_limit = torch.tensor(
            self.config.command_limit, device=self.device, dtype=torch.float32
        )
        self.hidden_state = torch.zeros(1, self.num_envs, 64, device=self.device)
        self.cell_state = torch.zeros_like(self.hidden_state)
        self.last_action = torch.zeros(self.num_envs, 12, device=self.device)
        self.phase_time = torch.zeros(self.num_envs, device=self.device)

    def reset(self, context=None, env_ids=None) -> None:
        if env_ids is None:
            env_ids = self.torch.arange(self.num_envs, device=self.device)
        else:
            env_ids = self.torch.as_tensor(env_ids, device=self.device, dtype=self.torch.long)
        self.hidden_state[:, env_ids] = 0.0
        self.cell_state[:, env_ids] = 0.0
        self.last_action[env_ids] = 0.0
        self.phase_time[env_ids] = 0.0

    def _batch(self, value, width: int, name: str):
        value = self.torch.as_tensor(value, device=self.device, dtype=self.torch.float32)
        if value.ndim == 1:
            value = value.unsqueeze(0)
        if value.shape != (self.num_envs, width):
            raise ValueError(
                f"{name} must have shape ({self.num_envs}, {width}), got {tuple(value.shape)}"
            )
        return value

    def _projected_gravity_xyzw(self, quaternion):
        qx, qy, qz, qw = quaternion.unbind(dim=-1)
        return self.torch.stack(
            (
                2.0 * (-qz * qx + qw * qy),
                -2.0 * (qz * qy + qw * qx),
                1.0 - 2.0 * (qw * qw + qz * qz),
            ),
            dim=-1,
        )

    def infer_tensor(
        self,
        *,
        body_joint_pos,
        body_joint_vel,
        root_quat_xyzw,
        root_angular_velocity,
        velocity_command,
        upper_body_target=None,
    ):
        """Return 29 body-joint position targets without CPU round trips."""
        torch = self.torch
        q = self._batch(body_joint_pos, 29, "body_joint_pos")
        dq = self._batch(body_joint_vel, 29, "body_joint_vel")
        quat = self._batch(root_quat_xyzw, 4, "root_quat_xyzw")
        omega = self._batch(root_angular_velocity, 3, "root_angular_velocity")
        command = self._batch(velocity_command, 3, "velocity_command")
        command = torch.maximum(torch.minimum(command, self.command_limit), -self.command_limit)

        phase = torch.remainder(self.phase_time, self.config.period) / self.config.period
        clock = torch.stack(
            (torch.sin(2.0 * torch.pi * phase), torch.cos(2.0 * torch.pi * phase)),
            dim=-1,
        )
        observation = torch.cat(
            (
                omega * self.config.angular_velocity_scale,
                self._projected_gravity_xyzw(quat),
                command * self.command_scale,
                (q[:, :12] - self.default_lower) * self.config.dof_position_scale,
                dq[:, :12] * self.config.dof_velocity_scale,
                self.last_action,
                clock,
            ),
            dim=-1,
        )
        with torch.inference_mode():
            memory_output, (next_hidden, next_cell) = self.memory(
                observation.unsqueeze(0), (self.hidden_state, self.cell_state)
            )
            action = self.actor(memory_output.squeeze(0))
            # Keep the long-lived buffers as ordinary tensors. Assigning the
            # inference tensors returned by LSTM here makes a later partial
            # environment reset illegal outside inference mode.
            self.hidden_state.copy_(next_hidden)
            self.cell_state.copy_(next_cell)
        self.last_action.copy_(action)
        self.phase_time.add_(self.config.control_dt)

        target = q.clone()
        target[:, :12] = action * self.config.action_scale + self.default_lower
        if upper_body_target is not None:
            target[:, 12:] = self._batch(upper_body_target, 17, "upper_body_target")
        return target, {
            "observation": observation,
            "sport_action": action,
            "sport_command": command,
        }

    def act(self, agent_feedback: dict, task_info: dict) -> tuple[np.ndarray, dict[str, Any]]:
        if self.num_envs != 1:
            raise RuntimeError(
                "act() is the scalar adapter; use infer_tensor() for batched execution"
            )
        root_pose = agent_feedback["root_pose_w"]
        root_velocity = agent_feedback["root_velocity_w"]
        root_angular_velocity = agent_feedback.get(
            "root_angular_velocity_b", root_velocity[..., 3:6]
        )
        command = task_info.get("sport_command", np.zeros(3, dtype=np.float32))
        target, details = self.infer_tensor(
            body_joint_pos=agent_feedback["body_joint_pos"],
            body_joint_vel=agent_feedback["body_joint_vel"],
            root_quat_xyzw=root_pose[..., 3:7],
            root_angular_velocity=root_angular_velocity,
            velocity_command=command,
            upper_body_target=task_info.get("upper_body_target"),
        )
        target_np = target[0].detach().cpu().numpy()
        info = {key: value[0].detach().cpu().numpy() for key, value in details.items()}
        info["target_actuated_pos"] = target_np.copy()
        return np.zeros(self.num_control, dtype=np.float32), info
