from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
import yaml

from spark_policy.core.policy import BasePolicy
from spark_robot import RobotConfig, RobotKinematics, UnitreeG1WholeBodyDynamic1Config


def _wrap_to_pi(angle: float) -> float:
    return (float(angle) + np.pi) % (2.0 * np.pi) - np.pi


def _repo_module_root() -> Path:
    return Path(__file__).resolve().parents[5]


def _default_wbt_root() -> Path:
    return Path(__file__).resolve().with_name("runtime")


def _resolve_existing_path(path: str, base_dir: Optional[Path] = None) -> Path:
    candidate = Path(path).expanduser()
    if candidate.is_absolute() and candidate.exists():
        return candidate

    if base_dir is not None:
        rel_candidate = (base_dir / candidate).resolve()
        if rel_candidate.exists():
            return rel_candidate

    path_str = str(path)
    module_root = _repo_module_root()
    if path_str.startswith("module/"):
        rel_candidate = (module_root.parent / path_str).resolve()
        if rel_candidate.exists():
            return rel_candidate

    for package_name in ("spark_policy", "spark_agent"):
        package_candidate = (module_root / package_name / path_str).resolve()
        if package_candidate.exists():
            return package_candidate

    rel_candidate = (Path.cwd() / path_str).resolve()
    if rel_candidate.exists():
        return rel_candidate

    raise FileNotFoundError(f"Could not resolve path: {path}")


def _as_vector(value, length: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape == ():
        return np.full(length, float(array), dtype=float)
    array = array.reshape(-1)
    if array.shape[0] != length:
        raise ValueError(f"{name} must have {length} entries, got {array.shape[0]}")
    return array.copy()


class WBTPolicyConfig:
    def __init__(self, file_path: str) -> None:
        self.file_path = _resolve_existing_path(file_path)
        with open(self.file_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        for key, value in config.items():
            setattr(self, key, value)

        self.simulation_dt = self.control_dt / self.control_decimation
        self.kps = np.array(config["kps"], dtype=np.float32)
        self.kds = np.array(config["kds"], dtype=np.float32)
        self.default_angles = np.array(config["default_angles"], dtype=np.float32)
        self.cmd_scale = (
            np.array(config["cmd_scale"], dtype=np.float32) if "cmd_scale" in config else None
        )
        self.max_cmd = (
            np.array(config["max_cmd"], dtype=np.float32) if "max_cmd" in config else None
        )
        self.cmd_debug = (
            np.array(config["cmd_debug"], dtype=np.float32) if "cmd_debug" in config else None
        )
        self.cmd_clip = (
            np.array(config["cmd_clip"], dtype=np.float32) if "cmd_clip" in config else None
        )
        if hasattr(self, "policy_path"):
            self.policy_path = str(
                _resolve_existing_path(self.policy_path, base_dir=self.file_path.parent)
            )

        self.gait_parameters = None
        if "gait_parameters" in config:
            self.gait_parameters = dict(config["gait_parameters"])


def _load_policy(policy_path: str):
    if policy_path.endswith(".onnx"):
        import onnxruntime as ort

        return "onnx", ort.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
    if policy_path.endswith(".pt"):
        import torch

        return "jit", torch.jit.load(policy_path)
    raise NotImplementedError(f"Unsupported policy file: {policy_path}")


class BipedalGaitPlanner:
    num_feet = 2

    def __init__(
        self,
        dt: float,
        frequencies: float = 1.5,
        phase_offset: float = 0.5,
        stance_ratio: float = 0.6,
    ) -> None:
        self.dt = float(dt)
        self.frequencies = float(frequencies)
        self.phase_offset = float(phase_offset)
        self.stance_ratio = float(stance_ratio)
        self.stance_middle_point = 0.3
        self.gait_index = self.stance_middle_point
        self.foot_indices = np.zeros(self.num_feet, dtype=np.float32)
        self.clock_inputs = np.zeros(self.num_feet, dtype=np.float32)

    def update_gait_phase(self, stop: bool = False) -> None:
        self.gait_index = (self.gait_index + self.dt * self.frequencies) % 1.0
        self.foot_indices[0] = (self.gait_index + self.phase_offset) % 1.0
        self.foot_indices[1] = self.gait_index

        if stop:
            self.gait_index = self.stance_middle_point
            self.foot_indices[:] = self.stance_middle_point

        for i in range(self.num_feet):
            idx = self.foot_indices[i]
            if idx < self.stance_ratio:
                self.foot_indices[i] = 0.5 * idx / self.stance_ratio
            else:
                self.foot_indices[i] = 0.5 + 0.5 * (idx - self.stance_ratio) / (
                    1 - self.stance_ratio
                )
            self.clock_inputs[i] = np.sin(2.0 * np.pi * self.foot_indices[i])


class _LowLevelPolicy:
    def __init__(self, cfg: WBTPolicyConfig) -> None:
        self.cfg = cfg
        self.policy_type, self.policy_session = _load_policy(cfg.policy_path)
        self._last_action = np.zeros(self.cfg.num_actions, dtype=np.float32)
        self.hidden_states = np.zeros([1, 1, 256], dtype=np.float32)

    def _run_policy(self, obs: np.ndarray) -> np.ndarray:
        if self.policy_type == "onnx":
            actions, self.hidden_states = self.policy_session.run(
                ["action", "output_hidden_states"],
                {
                    "obs": obs,
                    "input_hidden_states": self.hidden_states,
                },
            )
            return actions[0].squeeze()

        if self.policy_type == "jit":
            import torch

            return self.policy_session(torch.from_numpy(obs)).detach().numpy().squeeze()

        raise NotImplementedError(f"Unsupported policy type: {self.policy_type}")

    def action_to_target_dof_pos(self, action: np.ndarray) -> np.ndarray:
        return action * self.cfg.action_scale + self.cfg.default_angles[self.cfg.action_idx]


class SquatLowLevelPolicy(_LowLevelPolicy):
    def inference(self, cmd, gravity_orientation, omega, qj, dqj):
        obs = self.compute_observation(cmd, gravity_orientation, omega, qj, dqj)
        action = self._run_policy(obs)
        action = np.clip(action, -self.cfg.clip_actions, self.cfg.clip_actions)
        self._last_action[:] = action.copy()
        if action.shape[0] != self.cfg.num_actions:
            raise ValueError(
                f"Expected {self.cfg.num_actions} squat actions, got {action.shape[0]}"
            )
        return obs, action, self.action_to_target_dof_pos(action)

    def compute_observation(self, cmd, gravity_orientation, omega, qj, dqj):
        default_angles_obs = self.cfg.default_angles[self.cfg.dof_idx]
        obs = np.concatenate(
            [
                cmd * self.cfg.cmd_scale,
                gravity_orientation,
                omega * self.cfg.ang_vel_scale,
                (qj - default_angles_obs) * self.cfg.dof_pos_scale,
                dqj * self.cfg.dof_vel_scale,
                self._last_action,
            ]
        )
        if obs.shape[0] != self.cfg.num_obs:
            raise ValueError(f"Expected {self.cfg.num_obs} squat obs entries, got {obs.shape[0]}")
        return (
            np.clip(obs, -self.cfg.clip_observations, self.cfg.clip_observations)
            .astype(np.float32)
            .reshape(1, -1)
        )


class LocoLowLevelPolicy(_LowLevelPolicy):
    def __init__(self, cfg: WBTPolicyConfig) -> None:
        super().__init__(cfg)
        gait_parameters = cfg.gait_parameters or {}
        self.gait_planner = BipedalGaitPlanner(
            cfg.control_dt,
            gait_parameters.get("frequencies", 1.5),
            phase_offset=gait_parameters.get("phase_offset", 0.5),
            stance_ratio=gait_parameters.get("stance_ratio", 0.6),
        )

    def inference(self, cmd, gravity_orientation, omega, qj, dqj):
        obs = self.compute_observation(cmd, gravity_orientation, omega, qj, dqj)
        action = self._run_policy(obs)
        action = np.clip(action, -self.cfg.clip_actions, self.cfg.clip_actions)
        self._last_action[:] = action.copy()
        if action.shape[0] != self.cfg.num_actions:
            raise ValueError(f"Expected {self.cfg.num_actions} loco actions, got {action.shape[0]}")
        return obs, action, self.action_to_target_dof_pos(action)

    def compute_observation(self, cmd, gravity_orientation, omega, qj, dqj):
        default_angles_obs = self.cfg.default_angles[self.cfg.dof_idx]
        obs = np.concatenate(
            [
                cmd * self.cfg.cmd_scale,
                gravity_orientation,
                omega * self.cfg.ang_vel_scale,
                (qj - default_angles_obs) * self.cfg.dof_pos_scale,
                dqj * self.cfg.dof_vel_scale,
                self._last_action,
                self.gait_planner.clock_inputs,
            ]
        )
        if obs.shape[0] != self.cfg.num_obs:
            raise ValueError(f"Expected {self.cfg.num_obs} loco obs entries, got {obs.shape[0]}")
        return (
            np.clip(obs, -self.cfg.clip_observations, self.cfg.clip_observations)
            .astype(np.float32)
            .reshape(1, -1)
        )


class _Controller:
    def __init__(self, config: WBTPolicyConfig) -> None:
        self.config = config
        self.action = np.zeros(config.num_actions, dtype=np.float32)
        self.obs = np.zeros(config.num_obs, dtype=np.float32)
        self.kps = config.kps
        self.kds = config.kds
        self.transition_count = 0

    def set_transition_count(self) -> None:
        self.transition_count = int(self.config.transition_time)


class _LocoController(_Controller):
    def __init__(self, config: WBTPolicyConfig) -> None:
        super().__init__(config)
        self.low_level_policy = LocoLowLevelPolicy(self.config)
        self.loco_cmd = np.zeros(3, dtype=np.float32)
        self.stance_command = False
        self.last_policy_target_dof_pos = self.config.default_angles.copy()[:12]

    def run(self, cmd_raw, gravity_orientation, omega, qj_obs, dqj_obs, target_dof_pos):
        self.loco_cmd = (
            np.zeros(3, dtype=np.float32)
            if cmd_raw is None
            else np.asarray(cmd_raw, dtype=np.float32)
        )
        if self.stance_command:
            self.loco_cmd = np.zeros(3, dtype=np.float32)

        self.low_level_policy.gait_planner.update_gait_phase(self.stance_command)
        self.obs, self.action, target_dof_pos[self.config.action_idx] = (
            self.low_level_policy.inference(
                self.loco_cmd,
                gravity_orientation,
                omega,
                qj_obs[self.config.dof_idx],
                dqj_obs[self.config.dof_idx],
            )
        )
        return target_dof_pos


class _SquatController(_Controller):
    def __init__(self, config: WBTPolicyConfig) -> None:
        super().__init__(config)
        self.low_level_policy = SquatLowLevelPolicy(self.config)
        self.squat_cmd = np.array([0.75, 0.0], dtype=np.float32)

    def run(self, cmd_raw, gravity_orientation, omega, qj_obs, dqj_obs, target_dof_pos):
        if cmd_raw is not None:
            d_hp = np.asarray(cmd_raw, dtype=np.float32) * self.config.max_cmd / 250.0
            self.squat_cmd += d_hp
            self.squat_cmd[0] = np.clip(self.squat_cmd[0], 0.75 - self.config.max_cmd[0], 0.75)
            self.squat_cmd[1] = np.clip(self.squat_cmd[1], 0.0, self.config.max_cmd[1])

        self.obs, self.action, target_dof_pos[self.config.action_idx] = (
            self.low_level_policy.inference(
                self.squat_cmd,
                gravity_orientation,
                omega,
                qj_obs[self.config.dof_idx],
                dqj_obs[self.config.dof_idx],
            )
        )
        target_dof_pos[[5, 11]] = 0.0
        return target_dof_pos


class _TensorGruPolicy:
    """Torch reconstruction of the fixed-batch ONNX recurrent policy."""

    def __init__(self, model_path: str, num_envs: int, device: str) -> None:
        import onnx
        import torch
        from onnx import numpy_helper

        self.torch = torch
        self.device = torch.device(device)
        graph = onnx.load(model_path).graph
        weights = {value.name: numpy_helper.to_array(value).copy() for value in graph.initializer}
        input_width = int(weights["onnx::GRU_111"].shape[-1])
        hidden_width = int(weights["onnx::GRU_112"].shape[-1])

        self.memory = torch.nn.GRU(input_width, hidden_width).to(self.device)
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(hidden_width, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 12),
        ).to(self.device)

        gate_order = (1, 0, 2)  # ONNX (update, reset, new) -> Torch (reset, update, new)

        def reorder(array):
            chunks = np.split(array, 3, axis=0)
            return np.concatenate([chunks[index] for index in gate_order], axis=0)

        onnx_bias = weights["onnx::GRU_113"][0]
        with torch.no_grad():
            self.memory.weight_ih_l0.copy_(
                torch.from_numpy(reorder(weights["onnx::GRU_111"][0])).to(self.device)
            )
            self.memory.weight_hh_l0.copy_(
                torch.from_numpy(reorder(weights["onnx::GRU_112"][0])).to(self.device)
            )
            self.memory.bias_ih_l0.copy_(
                torch.from_numpy(reorder(onnx_bias[: 3 * hidden_width])).to(self.device)
            )
            self.memory.bias_hh_l0.copy_(
                torch.from_numpy(reorder(onnx_bias[3 * hidden_width :])).to(self.device)
            )
            for torch_index, onnx_index in ((0, 0), (2, 2), (4, 4)):
                self.actor[torch_index].weight.copy_(
                    torch.from_numpy(weights[f"actor.{onnx_index}.weight"]).to(self.device)
                )
                self.actor[torch_index].bias.copy_(
                    torch.from_numpy(weights[f"actor.{onnx_index}.bias"]).to(self.device)
                )
        self.memory.eval()
        self.actor.eval()
        self.hidden = torch.zeros(1, num_envs, hidden_width, device=self.device)

    def reset(self, env_ids) -> None:
        # ``__call__`` replaces this state inside inference mode, so the
        # returned tensor is itself an inference tensor. Partial episode
        # resets must mutate it under the same mode.
        with self.torch.inference_mode():
            self.hidden[:, env_ids] = 0.0

    def __call__(self, observation):
        with self.torch.inference_mode():
            output, self.hidden = self.memory(observation.unsqueeze(0), self.hidden)
            return self.actor(output.squeeze(0))


class UnitreeG1BatchedWBTPolicy(BasePolicy):
    """GPU-batched learned WBT stage with externally supplied arm targets.

    Parallel IK is intentionally kept outside this class. This allows the
    recurrent lower-body controller and simulator scaling to be verified
    before a cuRobo-backed arm target generator is connected.
    """

    def __init__(
        self,
        robot_cfg: RobotConfig,
        robot_kinematics: RobotKinematics | None = None,
        *,
        num_envs: int = 1,
        device: str = "cuda:0",
        loco_config_path: str | None = None,
        squat_config_path: str | None = None,
    ) -> None:
        super().__init__(robot_cfg, robot_kinematics)
        import torch

        self.torch = torch
        self.device = torch.device(device)
        self.num_envs = int(num_envs)
        config_dir = _default_wbt_root() / "configs"
        self.loco_config = WBTPolicyConfig(
            str(loco_config_path or config_dir / "unitree_g1_loco.yaml")
        )
        self.squat_config = WBTPolicyConfig(
            str(squat_config_path or config_dir / "unitree_g1_squat.yaml")
        )
        self.loco_network = _TensorGruPolicy(self.loco_config.policy_path, self.num_envs, device)
        self.squat_network = _TensorGruPolicy(self.squat_config.policy_path, self.num_envs, device)
        self.loco_last_action = torch.zeros(self.num_envs, 12, device=self.device)
        self.squat_last_action = torch.zeros_like(self.loco_last_action)
        self.gait_index = torch.full((self.num_envs,), 0.3, device=self.device)
        self.squat_command = torch.tensor(
            (0.75, 0.0), device=self.device, dtype=torch.float32
        ).repeat(self.num_envs, 1)
        self.loco_default = torch.tensor(
            self.loco_config.default_angles, device=self.device, dtype=torch.float32
        )
        self.squat_default = torch.tensor(
            self.squat_config.default_angles, device=self.device, dtype=torch.float32
        )
        self.loco_command_scale = torch.tensor(
            (3.0, 3.0, 5.0), device=self.device, dtype=torch.float32
        )

    def reset(self, context=None, env_ids=None) -> None:
        if env_ids is None:
            env_ids = self.torch.arange(self.num_envs, device=self.device)
        else:
            env_ids = self.torch.as_tensor(env_ids, device=self.device, dtype=self.torch.long)
        self.loco_network.reset(env_ids)
        self.squat_network.reset(env_ids)
        self.loco_last_action[env_ids] = 0.0
        self.squat_last_action[env_ids] = 0.0
        self.gait_index[env_ids] = 0.3
        self.squat_command[env_ids, 0] = 0.75
        self.squat_command[env_ids, 1] = 0.0

    def _batch(self, value, width: int, name: str):
        value = self.torch.as_tensor(value, device=self.device, dtype=self.torch.float32)
        if value.ndim == 1:
            value = value.unsqueeze(0)
        if value.shape != (self.num_envs, width):
            raise ValueError(
                f"{name} must have shape ({self.num_envs}, {width}), got {tuple(value.shape)}"
            )
        return value

    def _projected_gravity(self, quaternion):
        qx, qy, qz, qw = quaternion.unbind(dim=-1)
        return self.torch.stack(
            (
                2.0 * (-qz * qx + qw * qy),
                -2.0 * (qz * qy + qw * qx),
                1.0 - 2.0 * (qw * qw + qz * qz),
            ),
            dim=-1,
        )

    def _loco_clock(self, stance: bool):
        cfg = self.loco_config.gait_parameters or {}
        frequency = float(cfg.get("frequencies", 1.5))
        phase_offset = float(cfg.get("phase_offset", 0.5))
        stance_ratio = float(cfg.get("stance_ratio", 0.6))
        self.gait_index.add_(self.loco_config.control_dt * frequency).remainder_(1.0)
        if stance:
            self.gait_index.fill_(0.3)
        foot_phase = self.torch.stack(
            ((self.gait_index + phase_offset).remainder(1.0), self.gait_index), dim=-1
        )
        warped = self.torch.where(
            foot_phase < stance_ratio,
            0.5 * foot_phase / stance_ratio,
            0.5 + 0.5 * (foot_phase - stance_ratio) / (1.0 - stance_ratio),
        )
        return self.torch.sin(2.0 * self.torch.pi * warped)

    def infer_tensor(
        self,
        *,
        body_joint_pos,
        body_joint_vel,
        root_quat_xyzw,
        root_angular_velocity,
        command,
        upper_body_target=None,
        mode: str = "loco",
        stance: bool = False,
    ):
        torch = self.torch
        q = self._batch(body_joint_pos, 29, "body_joint_pos")
        dq = self._batch(body_joint_vel, 29, "body_joint_vel")
        quaternion = self._batch(root_quat_xyzw, 4, "root_quat_xyzw")
        omega = self._batch(root_angular_velocity, 3, "root_angular_velocity")
        gravity = self._projected_gravity(quaternion)

        if mode == "loco":
            cfg = self.loco_config
            # Match UnitreeG1WBTPolicy._run_loco exactly.  ``cmd_clip`` is a
            # legacy config field but the scalar ONNX controller does not
            # clamp the command with it; only the assembled observation is
            # clipped.  Clamping here removed most goal-direction authority
            # from the batched controller and let its open-loop gait drift.
            policy_command = self._batch(command, 3, "command") * self.loco_command_scale
            observation = torch.cat(
                (
                    policy_command * torch.tensor(cfg.cmd_scale, device=self.device),
                    gravity,
                    omega * cfg.ang_vel_scale,
                    (q[:, :12] - self.loco_default[:12]) * cfg.dof_pos_scale,
                    dq[:, :12] * cfg.dof_vel_scale,
                    self.loco_last_action,
                    self._loco_clock(stance),
                ),
                dim=-1,
            )
            observation.clamp_(-cfg.clip_observations, cfg.clip_observations)
            action = self.loco_network(observation).clamp(-cfg.clip_actions, cfg.clip_actions)
            self.loco_last_action.copy_(action)
            default = self.loco_default
        elif mode == "squat":
            cfg = self.squat_config
            if command is not None:
                self.squat_command = self._batch(command, 2, "command").clone()
            policy_command = self.squat_command
            observation = torch.cat(
                (
                    self.squat_command * torch.tensor(cfg.cmd_scale, device=self.device),
                    gravity,
                    omega * cfg.ang_vel_scale,
                    (q - self.squat_default) * cfg.dof_pos_scale,
                    dq * cfg.dof_vel_scale,
                    self.squat_last_action,
                ),
                dim=-1,
            )
            observation.clamp_(-cfg.clip_observations, cfg.clip_observations)
            action = self.squat_network(observation).clamp(-cfg.clip_actions, cfg.clip_actions)
            self.squat_last_action.copy_(action)
            default = self.squat_default
        else:
            raise ValueError("mode must be 'loco' or 'squat'")

        target = q.clone()
        target[:, :12] = action * cfg.action_scale + default[:12]
        if mode == "squat":
            target[:, (5, 11)] = 0.0
        if upper_body_target is not None:
            target[:, 12:] = self._batch(upper_body_target, 17, "upper_body_target")
        return target, {
            "observation": observation,
            "wbt_action": action,
            "wbt_command": policy_command,
            "motor_kps": torch.tensor(cfg.kps, device=self.device).expand(self.num_envs, -1),
            "motor_kds": torch.tensor(cfg.kds, device=self.device).expand(self.num_envs, -1),
        }

    def act(self, agent_feedback: dict, task_info: dict):
        if self.num_envs != 1:
            raise RuntimeError(
                "act() is the scalar adapter; use infer_tensor() for batched execution"
            )
        # Match the composed WBT policy: standing starts in the squat
        # controller and locomotion is selected only by an explicit request.
        mode = task_info.get("wbt_mode", "squat")
        default_command = np.zeros(3 if mode == "loco" else 2, dtype=np.float32)
        root_velocity = agent_feedback["root_velocity_w"]
        target, details = self.infer_tensor(
            body_joint_pos=agent_feedback["body_joint_pos"],
            body_joint_vel=agent_feedback["body_joint_vel"],
            root_quat_xyzw=agent_feedback["root_pose_w"][..., 3:7],
            root_angular_velocity=agent_feedback.get(
                "root_angular_velocity_b", root_velocity[..., 3:6]
            ),
            command=task_info.get("wbt_command", default_command),
            upper_body_target=task_info.get("upper_body_target"),
            mode=mode,
            stance=bool(task_info.get("stance", False)),
        )
        info = {key: value[0].detach().cpu().numpy() for key, value in details.items()}
        info["target_actuated_pos"] = target[0].detach().cpu().numpy()
        return np.zeros(self.num_control, dtype=np.float32), info


class UnitreeG1WBTPolicy(BasePolicy):
    """Unitree G1 whole-body teleop low-level policy.

    This policy owns the learned squat/loco policy state and returns full-body
    joint-position targets. It intentionally does not step MuJoCo or publish to
    hardware; agents consume the returned target fields.
    """

    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics, **kwargs) -> None:
        super().__init__(robot_cfg, robot_kinematics)

        wbt_root = _default_wbt_root()
        config_dir = wbt_root / "configs"
        self.loco_config = WBTPolicyConfig(
            str(kwargs.get("loco_config_path", config_dir / "unitree_g1_loco.yaml"))
        )
        self.squat_config = WBTPolicyConfig(
            str(kwargs.get("squat_config_path", config_dir / "unitree_g1_squat.yaml"))
        )
        self.loco_controller = _LocoController(self.loco_config)
        self.squat_controller = _SquatController(self.squat_config)

        self.body_robot_cfg = UnitreeG1WholeBodyDynamic1Config()
        self.default_qpos = np.array(
            [self.body_robot_cfg.DefaultDoFVal[dof] for dof in self.body_robot_cfg.DoFs],
            dtype=np.float64,
        )
        self.model_default_qpos = np.array(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs], dtype=np.float64
        )
        self.model_actuated_qpos_indices = self._model_actuated_qpos_indices()
        self.body_actuated_qpos_indices = self._body_actuated_qpos_indices()
        self.num_actuated = self.model_actuated_qpos_indices.shape[0]
        self.target_dof_pos_lower_body = self.default_qpos[7:19].astype(np.float32).copy()
        self.last_upper_body_target = self.default_qpos[19:36].astype(np.float64).copy()
        self.transfer_to_squat = True
        self.transfer_to_loco = False

        self.base_position_kp = np.asarray(
            kwargs.get("base_position_kp", [0.80, 0.80, 0.60]), dtype=float
        )
        self.base_orientation_kp = np.asarray(
            kwargs.get("base_orientation_kp", [0.50, 0.50, 0.80]), dtype=float
        )
        self.base_velocity_kd = np.asarray(
            kwargs.get("base_velocity_kd", [0.40, 0.40, 0.25]), dtype=float
        )
        self.base_arrival_velocity = np.asarray(
            kwargs.get("base_arrival_velocity", [0.05, 0.05, 0.10]), dtype=float
        )
        # The 1.5 Hz gait crosses zero velocity twice per cycle.  Require more
        # than one complete gait period before declaring the base stationary.
        self.base_settle_steps = max(1, int(kwargs.get("base_settle_steps", 50)))
        self.hold_loco_stance_after_motion = bool(
            kwargs.get("hold_loco_stance_after_motion", False)
        )
        self.base_xy_yaw_limit = np.asarray(
            kwargs.get("base_xy_yaw_limit", [0.14, 0.10, 0.45]), dtype=float
        )
        self.base_z_pitch_limit = np.asarray(
            kwargs.get("base_z_pitch_limit", [0.10, 0.03]), dtype=float
        )
        self.base_command_rate_limit = np.asarray(
            kwargs.get("base_command_rate_limit", [0.04, 0.04, 0.15, 0.02, 0.004]),
            dtype=float,
        )
        self.base_command_deadband = np.asarray(
            kwargs.get("base_command_deadband", [0.005, 0.005, 0.010, 0.004, 0.004]),
            dtype=float,
        )
        self.loco_command_scale = np.asarray(
            kwargs.get("loco_command_scale", [3.0, 3.0, 5.0]), dtype=float
        )
        self.squat_command_scale = np.asarray(
            kwargs.get("squat_command_scale", [10.0, 10.0]), dtype=float
        )
        self.zero_squat_command = bool(kwargs.get("zero_squat_command", False))
        self.loco_start_threshold = float(kwargs.get("loco_start_threshold", 0.035))
        self.safe_loco_start_threshold = float(kwargs.get("safe_loco_start_threshold", 0.01))
        self.stop_threshold = np.asarray(
            kwargs.get("stop_threshold", [0.025, 0.025, 0.050]), dtype=float
        )
        self.base_goal_xy_deadband = float(kwargs.get("base_goal_xy_deadband", 0.0))
        self.base_goal_resume_distance = float(
            kwargs.get("base_goal_resume_distance", self.base_goal_xy_deadband)
        )
        self.base_goal_yaw_deadband = float(kwargs.get("base_goal_yaw_deadband", 0.0))
        self.base_goal_yaw_resume_distance = float(
            kwargs.get("base_goal_yaw_resume_distance", self.base_goal_yaw_deadband)
        )
        self.use_policy_motor_gains = bool(kwargs.get("use_policy_motor_gains", True))
        self.motor_kp_scale = float(kwargs.get("motor_kp_scale", 1.0))
        self.motor_kd_scale = float(kwargs.get("motor_kd_scale", 1.0))
        self.upper_body_gravity_compensation = bool(
            kwargs.get("upper_body_gravity_compensation", False)
        )
        self.hold_upper_body_during_locomotion = bool(
            kwargs.get("hold_upper_body_during_locomotion", False)
        )
        self.upper_body_target_rate_limit = _as_vector(
            kwargs.get("upper_body_target_rate_limit", [0.006, 0.006, 0.006] + [0.014] * 14),
            17,
            "upper_body_target_rate_limit",
        )
        self.upper_body_target_filter_gain = float(kwargs.get("upper_body_target_filter_gain", 1.0))
        self.disable_upper_body_target_rate_limit = bool(
            kwargs.get("disable_upper_body_target_rate_limit", False)
        )
        self.wbt_arm_debug = bool(kwargs.get("wbt_arm_debug", False))
        self.apply_safe_control_targets = bool(kwargs.get("apply_safe_control_targets", True))
        self.safe_control_only_when_triggered = bool(
            kwargs.get("safe_control_only_when_triggered", True)
        )
        self.safe_target_dt = float(kwargs.get("safe_target_dt", self.squat_config.control_dt))
        self._last_base_command = None
        self._base_goal_xy_hold = False
        self._base_goal_yaw_hold = False
        self._last_base_goal_xy = None
        self._last_base_goal_yaw = None
        self._base_goal_motion_requested = False
        self._base_settle_count = 0
        self._base_arrival_count = 0
        self._last_filtered_upper_body_target = self.last_upper_body_target.copy()
        self.arm_robot_cfg = kwargs.get("arm_robot_cfg", self.robot_cfg)
        if self.arm_robot_cfg is None:
            self.arm_robot_cfg = self.robot_cfg
        self.arm_kinematics = kwargs.get("arm_kinematics", self.robot_kinematics)
        if self.arm_kinematics is None:
            self.arm_kinematics = self.robot_kinematics
        self.arm_kinematics_mode = self._infer_arm_kinematics_mode(self.arm_kinematics)
        self.lock_default_waist = self.arm_kinematics_mode == "dual_arm"
        self.default_waist_qpos = self.default_qpos[19:22].copy()

    def reset(self, context=None) -> None:
        del context
        for controller in (self.loco_controller, self.squat_controller):
            controller.transition_count = 0
            controller.action.fill(0.0)
            controller.obs.fill(0.0)
            controller.low_level_policy._last_action.fill(0.0)
            controller.low_level_policy.hidden_states.fill(0.0)
        self.loco_controller.low_level_policy.gait_planner.gait_index = 0.3
        self.loco_controller.low_level_policy.gait_planner.foot_indices.fill(0.0)
        self.loco_controller.low_level_policy.gait_planner.clock_inputs.fill(0.0)
        self.loco_controller.loco_cmd.fill(0.0)
        self.loco_controller.stance_command = False
        self.squat_controller.squat_cmd[:] = (0.75, 0.0)
        self.target_dof_pos_lower_body = self.default_qpos[7:19].astype(np.float32).copy()
        self.last_upper_body_target = self.default_qpos[19:36].astype(np.float64).copy()
        self._last_filtered_upper_body_target = self.last_upper_body_target.copy()
        self.transfer_to_squat = True
        self.transfer_to_loco = False
        self._last_base_command = None
        self._base_goal_xy_hold = False
        self._base_goal_yaw_hold = False
        self._last_base_goal_xy = None
        self._last_base_goal_yaw = None
        self._base_goal_motion_requested = False
        self._base_settle_count = 0
        self._base_arrival_count = 0

    @staticmethod
    def _infer_arm_kinematics_mode(kinematics: RobotKinematics) -> str:
        class_name = kinematics.__class__.__name__
        if class_name.endswith("DualArmKinematics"):
            return "dual_arm"
        if class_name.endswith("FixedBaseKinematics"):
            return "fixed_base"
        return "robot"

    def _model_actuated_qpos_indices(self) -> np.ndarray:
        indices = []
        for motor in self.robot_cfg.MujocoMotors:
            # MobileBase configs include virtual X/Y/yaw motors in their
            # public control contract.  The articulated WBT target contains
            # only physical joints; locomotion commands are handled
            # separately by the composed policy.
            if not hasattr(self.body_robot_cfg.DoFs, motor.name):
                continue
            if not hasattr(self.robot_cfg.MujocoDoFs, motor.name):
                raise KeyError(f"No MuJoCo DoF named {motor.name} for motor {motor}")
            indices.append(int(getattr(self.robot_cfg.MujocoDoFs, motor.name)))
        return np.asarray(indices, dtype=int)

    def _body_actuated_qpos_indices(self) -> np.ndarray:
        indices = []
        for motor in self.robot_cfg.MujocoMotors:
            if not hasattr(self.body_robot_cfg.DoFs, motor.name):
                continue
            indices.append(int(getattr(self.body_robot_cfg.DoFs, motor.name)))
        return np.asarray(indices, dtype=int)

    def _raw_qpos_qvel_from_feedback(self, agent_feedback: dict) -> Tuple[np.ndarray, np.ndarray]:
        has_body_feedback = "body_qpos_fbk" in agent_feedback
        qpos_key = "body_qpos_fbk" if has_body_feedback else "qpos_fbk"
        qvel_key = "body_qvel_fbk" if has_body_feedback else "qvel_fbk"
        qpos = np.asarray(
            agent_feedback.get(qpos_key, agent_feedback["dof_pos_fbk"]), dtype=float
        ).reshape(-1)
        qvel = np.asarray(
            agent_feedback.get(qvel_key, agent_feedback.get("dof_vel_fbk", np.zeros(0))),
            dtype=float,
        ).reshape(-1)
        if has_body_feedback:
            return qpos, qvel

        if qpos.shape[0] < self.model_default_qpos.shape[0]:
            full = self.model_default_qpos.copy()
            full[: qpos.shape[0]] = qpos
            qpos = full
        return qpos, qvel

    def _virtual_qpos(self, qpos: np.ndarray) -> np.ndarray:
        qpos = np.asarray(qpos, dtype=float).reshape(-1)
        if qpos.shape[0] == self.default_qpos.shape[0]:
            return qpos.copy()
        state = np.asarray(self.robot_cfg.compose_state_from_dof(qpos, None), dtype=float).reshape(
            -1
        )
        if state.shape[0] == self.default_qpos.shape[0]:
            return state
        full = self.default_qpos.copy()
        full[: min(full.shape[0], qpos.shape[0])] = qpos[: min(full.shape[0], qpos.shape[0])]
        return full

    def _virtual_qvel(self, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
        qpos = np.asarray(qpos, dtype=float).reshape(-1)
        qvel = np.asarray(qvel, dtype=float).reshape(-1)
        if qpos.shape[0] == self.default_qpos.shape[0] and qvel.shape[0] >= 35:
            return qvel[:35].copy()

        virtual_qvel = np.zeros(35, dtype=float)
        qpos_qvel_offset = 1 if qpos.shape[0] - qvel.shape[0] == 1 else 0
        for model_qpos_idx, body_qpos_idx in zip(
            self.model_actuated_qpos_indices, self.body_actuated_qpos_indices
        ):
            model_qvel_idx = int(model_qpos_idx) - qpos_qvel_offset
            body_qvel_idx = int(body_qpos_idx) - 1
            if 0 <= model_qvel_idx < qvel.shape[0] and 0 <= body_qvel_idx < virtual_qvel.shape[0]:
                virtual_qvel[body_qvel_idx] = qvel[model_qvel_idx]
        return virtual_qvel

    def _qpos_qvel_from_feedback(self, agent_feedback: dict) -> Tuple[np.ndarray, np.ndarray]:
        qpos_raw, qvel_raw = self._raw_qpos_qvel_from_feedback(agent_feedback)
        return self._virtual_qpos(qpos_raw), self._virtual_qvel(qpos_raw, qvel_raw)

    def _actuated_qpos_from_raw(self, qpos: np.ndarray) -> np.ndarray:
        qpos = np.asarray(qpos, dtype=float).reshape(-1)
        if qpos.shape[0] > int(np.max(self.model_actuated_qpos_indices)):
            return qpos[self.model_actuated_qpos_indices].copy()
        return self._actuated_qpos_from_virtual(self._virtual_qpos(qpos))

    def _actuated_qpos_from_virtual(self, qpos: np.ndarray) -> np.ndarray:
        return np.asarray(qpos, dtype=float).reshape(-1)[self.body_actuated_qpos_indices].copy()

    def _control_limit_vector(self) -> np.ndarray:
        return np.asarray(
            [self.robot_cfg.ControlLimit[control_id] for control_id in self.robot_cfg.Control],
            dtype=float,
        )

    def _add_gripper_info(self, task_info: dict, action_info: Dict[str, Any]) -> None:
        goal_teleop = task_info.get("goal_teleop", {})
        for key in (
            "left_gripper_goal",
            "right_gripper_goal",
            "gripper_goal",
            "left_grasp_target_name",
            "right_grasp_target_name",
            "grasp_target_name",
            "left_grasp_radius",
            "right_grasp_radius",
            "grasp_radius",
        ):
            if key in goal_teleop:
                action_info[key] = goal_teleop[key]
            elif key in task_info:
                action_info[key] = task_info[key]

    def _safe_control_reference(
        self, current_actuated_pos: np.ndarray, target_actuated_pos: np.ndarray
    ) -> np.ndarray:
        dt = max(self.safe_target_dt, 1.0e-6)
        u_ref = (target_actuated_pos - current_actuated_pos) / dt
        control_limit = self._control_limit_vector()
        if u_ref.shape == control_limit.shape:
            u_ref = np.clip(u_ref, -control_limit, control_limit)
        return u_ref

    def _gravity_orientation(self, quat_wxyz: np.ndarray) -> np.ndarray:
        qw, qx, qy, qz = quat_wxyz
        return np.array(
            [
                2.0 * (-qz * qx + qw * qy),
                -2.0 * (qz * qy + qw * qx),
                1.0 - 2.0 * (qw * qw + qz * qz),
            ],
            dtype=np.float32,
        )

    def _filter_base_command(self, command: np.ndarray) -> np.ndarray:
        command = np.asarray(command, dtype=float).reshape(5).copy()
        command[:3] = np.clip(command[:3], -self.base_xy_yaw_limit, self.base_xy_yaw_limit)
        command[3:] = np.clip(command[3:], -self.base_z_pitch_limit, self.base_z_pitch_limit)

        if self.base_command_deadband.shape == command.shape:
            command[np.abs(command) < self.base_command_deadband] = 0.0

        if self._last_base_command is None:
            self._last_base_command = np.zeros(5, dtype=float)
        if self.base_command_rate_limit.shape == command.shape:
            delta = np.clip(
                command - self._last_base_command,
                -self.base_command_rate_limit,
                self.base_command_rate_limit,
            )
            command = self._last_base_command + delta
        self._last_base_command = command.copy()
        return command

    def _filter_upper_body_target(self, target: np.ndarray) -> np.ndarray:
        target = self._clip_upper_body_target(target)
        if self.disable_upper_body_target_rate_limit:
            self._last_filtered_upper_body_target = target.copy()
            return target.copy()

        if self._last_filtered_upper_body_target is None:
            self._last_filtered_upper_body_target = self.last_upper_body_target.copy()

        delta = self.upper_body_target_filter_gain * (
            target - self._last_filtered_upper_body_target
        )
        delta = np.clip(
            delta, -self.upper_body_target_rate_limit, self.upper_body_target_rate_limit
        )
        filtered = self._clip_upper_body_target(self._last_filtered_upper_body_target + delta)
        self._last_filtered_upper_body_target = filtered.copy()
        return filtered

    def _apply_default_waist_lock(self, target: np.ndarray) -> np.ndarray:
        target = np.asarray(target, dtype=float).reshape(17).copy()
        if self.lock_default_waist:
            target[:3] = self.default_waist_qpos
        return target

    def _base_command_from_goal(self, agent_feedback: dict, task_info: dict) -> np.ndarray:
        for key in ("wbt_command", "base_command", "locomotion_command"):
            if key in task_info:
                # Keep task inputs immutable: arrival handling modifies the
                # local command before filtering.
                command = np.asarray(task_info[key], dtype=float).reshape(-1).copy()
                if command.shape[0] == 3:
                    command = np.concatenate([command, np.zeros(2)])
                if command.shape[0] != 5:
                    raise ValueError(f"{key} must have 3 or 5 entries.")
                if "wbt_height" in task_info:
                    target_height = float(
                        np.asarray(task_info["wbt_height"], dtype=float).reshape(-1)[0]
                    )
                    robot_base_frame = np.asarray(
                        agent_feedback["robot_base_frame"], dtype=float
                    ).reshape(4, 4)
                    command[3] = self.base_position_kp[2] * (target_height - robot_base_frame[2, 3])
                if task_info.get("wbt_goal_arrived", False):
                    command[:3] = 0.0
                    if self._last_base_command is not None:
                        self._last_base_command[:3] = 0.0
                return self._filter_base_command(command)

        goal_teleop = task_info.get("goal_teleop", {})
        base_goal = goal_teleop.get("base", None)
        if base_goal is None:
            return self._filter_base_command(np.zeros(5, dtype=float))

        base_goal = np.asarray(base_goal, dtype=float).reshape(-1, 4, 4)[0]
        robot_base_frame = np.asarray(agent_feedback["robot_base_frame"], dtype=float)

        yaw = np.arctan2(robot_base_frame[1, 0], robot_base_frame[0, 0])
        target_yaw = np.arctan2(base_goal[1, 0], base_goal[0, 0])
        base_goal_xy = base_goal[:2, 3].copy()
        if self._last_base_goal_xy is None:
            self._last_base_goal_xy = base_goal_xy.copy()
            self._last_base_goal_yaw = float(target_yaw)
            # A fixed goal supplied at reset is still a motion request.  The
            # previous implementation only armed locomotion after the goal
            # changed, so non-zero benchmark goals were silently treated as
            # an already-reached standing pose.
            initial_xy_error = float(np.linalg.norm(base_goal_xy - robot_base_frame[:2, 3]))
            initial_yaw_error = abs(_wrap_to_pi(target_yaw - yaw))
            self._base_goal_motion_requested = bool(
                initial_xy_error > self.base_goal_xy_deadband
                or initial_yaw_error > self.base_goal_yaw_deadband
            )
        else:
            goal_translation_changed = bool(
                np.linalg.norm(base_goal_xy - self._last_base_goal_xy) > 1.0e-6
            )
            goal_yaw_changed = bool(
                abs(_wrap_to_pi(target_yaw - self._last_base_goal_yaw)) > 1.0e-6
            )
            if goal_translation_changed or goal_yaw_changed:
                self._base_goal_motion_requested = True
                self._base_goal_xy_hold = False
                self._base_goal_yaw_hold = False
                self._base_arrival_count = 0
            self._last_base_goal_xy = base_goal_xy.copy()
            self._last_base_goal_yaw = float(target_yaw)
        pos_error_world = base_goal[:3, 3] - robot_base_frame[:3, 3]
        c, s = np.cos(yaw), np.sin(yaw)
        xy_body = np.array(
            [
                c * pos_error_world[0] + s * pos_error_world[1],
                -s * pos_error_world[0] + c * pos_error_world[1],
            ],
            dtype=float,
        )
        pitch = np.arctan2(
            -robot_base_frame[2, 0], np.hypot(robot_base_frame[2, 1], robot_base_frame[2, 2])
        )
        target_pitch = np.arctan2(-base_goal[2, 0], np.hypot(base_goal[2, 1], base_goal[2, 2]))
        yaw_error = _wrap_to_pi(target_yaw - yaw)

        body_qvel = np.asarray(
            agent_feedback.get("body_qvel_fbk", agent_feedback.get("qvel_fbk", np.zeros(6))),
            dtype=float,
        ).reshape(-1)
        linear_velocity_world = body_qvel[:3] if body_qvel.shape[0] >= 3 else np.zeros(3)
        planar_velocity_body = np.array(
            [
                c * linear_velocity_world[0] + s * linear_velocity_world[1],
                -s * linear_velocity_world[0] + c * linear_velocity_world[1],
            ],
            dtype=float,
        )
        yaw_velocity = float(body_qvel[5]) if body_qvel.shape[0] >= 6 else 0.0

        xy_error_norm = float(np.linalg.norm(xy_body))
        xy_arrival_candidate = False
        if self.base_goal_xy_deadband > 0.0:
            if not self._base_goal_motion_requested and self.transfer_to_squat:
                # An unchanged teleoperation goal is not a request to correct
                # the squat policy's small contact-dependent position wander.
                self._base_goal_xy_hold = True
            elif self._base_goal_xy_hold:
                # Arrival is event-latched.  A learned stance can drift beyond
                # a spatial hysteresis threshold after stopping; treating that
                # drift as a new command makes it alternate indefinitely
                # between walking and stance.  A changed task goal explicitly
                # clears this latch above.
                self._base_goal_xy_hold = True
            else:
                # Position alone is not arrival.  Latching while the learned
                # gait still has momentum removes the braking command and can
                # leave the locomotion RNN walking indefinitely at zero input.
                xy_arrival_candidate = (
                    xy_error_norm
                    <= max(
                        self.base_goal_xy_deadband,
                        self.base_goal_resume_distance,
                    )
                    and self.transfer_to_loco
                )
            if self._base_goal_xy_hold:
                xy_body[:] = 0.0
            elif xy_error_norm <= self.base_goal_xy_deadband:
                # Inside the spatial deadband, command only velocity damping.
                # This avoids chasing contact-scale pose noise while bringing
                # the base to a genuine stop before the stationary handoff.
                xy_body[:] = 0.0

        yaw_abs_error = abs(yaw_error)
        yaw_arrival_candidate = False
        if self.base_goal_yaw_deadband > 0.0:
            if not self._base_goal_motion_requested and self.transfer_to_squat:
                self._base_goal_yaw_hold = True
            elif self._base_goal_yaw_hold:
                self._base_goal_yaw_hold = True
            else:
                yaw_arrival_candidate = (
                    yaw_abs_error
                    <= max(
                        self.base_goal_yaw_deadband,
                        self.base_goal_yaw_resume_distance,
                    )
                    and self.transfer_to_loco
                )
            if self._base_goal_yaw_hold:
                yaw_error = 0.0
            elif yaw_abs_error <= self.base_goal_yaw_deadband:
                yaw_error = 0.0

        if (
            self._base_goal_motion_requested
            and not self._base_goal_xy_hold
            and not self._base_goal_yaw_hold
        ):
            if xy_arrival_candidate and yaw_arrival_candidate:
                self._base_arrival_count += 1
            else:
                self._base_arrival_count = 0
            # A learned gait crosses zero velocity within each step. Require
            # the pose to remain inside the configured hysteresis region for
            # more than one gait period; this measures net settling without
            # mistaking either an instantaneous velocity zero or foot-swing
            # velocity for base arrival.
            if self._base_arrival_count >= self.base_settle_steps:
                self._base_goal_xy_hold = True
                self._base_goal_yaw_hold = True
                self._base_arrival_count = 0

        if (
            self._base_goal_motion_requested
            and self._base_goal_xy_hold
            and self._base_goal_yaw_hold
        ):
            self._base_goal_motion_requested = False

        braking_enabled = self._base_goal_motion_requested or self.transfer_to_loco
        braking_planar_velocity = planar_velocity_body if braking_enabled else np.zeros(2)
        braking_yaw_velocity = yaw_velocity if braking_enabled else 0.0

        command = np.array(
            [
                self.base_position_kp[0] * xy_body[0]
                - self.base_velocity_kd[0] * braking_planar_velocity[0],
                self.base_position_kp[1] * xy_body[1]
                - self.base_velocity_kd[1] * braking_planar_velocity[1],
                self.base_orientation_kp[2] * yaw_error
                - self.base_velocity_kd[2] * braking_yaw_velocity,
                self.base_position_kp[2] * pos_error_world[2],
                self.base_orientation_kp[1] * _wrap_to_pi(target_pitch - pitch),
            ],
            dtype=float,
        )
        return self._filter_base_command(command)

    def _fallback_goal_frame(self, qpos: np.ndarray, frame_name: str) -> np.ndarray:
        frames = self.robot_kinematics.forward_kinematics(qpos)
        frame_id = getattr(self.robot_cfg.Frames, frame_name)
        return frames[frame_id].copy()

    def _solve_upper_body_ik(
        self, right_base: np.ndarray, left_base: np.ndarray, qpos: np.ndarray, qvel: np.ndarray
    ) -> np.ndarray:
        if self.arm_kinematics_mode == "dual_arm":
            current_arm_q = np.concatenate([qpos[22:29], qpos[29:36]])
            current_arm_dq = None
            if qvel.shape[0] >= 35:
                current_arm_dq = np.concatenate([qvel[21:28], qvel[28:35]])

            target, _ = self.arm_kinematics.inverse_kinematics(
                [right_base, left_base], current_arm_q, current_arm_dq
            )
            target = np.asarray(target, dtype=float).reshape(-1)
            if target.shape[0] != 14:
                raise ValueError(f"Dual-arm IK returned {target.shape[0]} entries; expected 14")

            upper_target = qpos[19:36].copy()
            upper_target[:3] = self.default_waist_qpos if self.lock_default_waist else qpos[19:22]
            upper_target[3:10] = target[:7]
            upper_target[10:17] = target[7:14]
            return self._apply_default_waist_lock(upper_target)

        if self.arm_kinematics_mode == "fixed_base":
            target, _ = self.arm_kinematics.inverse_kinematics(
                [right_base, left_base], qpos[19:36], None
            )
            target = np.asarray(target, dtype=float).reshape(-1)
            if target.shape[0] != 17:
                raise ValueError(
                    f"Fixed-base arm IK returned {target.shape[0]} entries; expected 17"
                )
            return self._apply_default_waist_lock(target)

        target, _ = self.arm_kinematics.inverse_kinematics([right_base, left_base], qpos, qvel)
        target = np.asarray(target, dtype=float).reshape(-1)
        if target.shape[0] < 17:
            raise ValueError(f"Arm IK returned {target.shape[0]} entries; expected at least 17")
        return self._apply_default_waist_lock(target[-17:])

    def _clip_upper_body_target(self, target: np.ndarray) -> np.ndarray:
        target = np.asarray(target, dtype=float).reshape(17).copy()
        real_motors = getattr(self.body_robot_cfg, "RealMotors", None)
        real_limits = getattr(self.body_robot_cfg, "RealMotorPosLimit", {})
        if real_motors is None:
            return target

        for local_idx, qpos_idx in enumerate(range(19, 36)):
            dof_name = self.body_robot_cfg.DoFs(qpos_idx).name
            if dof_name not in real_motors.__members__:
                continue
            motor = real_motors.__members__[dof_name]
            if motor in real_limits:
                target[local_idx] = np.clip(target[local_idx], *real_limits[motor])
        return self._apply_default_waist_lock(target)

    def _apply_upper_body_safety_target(self, target: np.ndarray, task_info: dict) -> np.ndarray:
        target = self._clip_upper_body_target(target)

        override = task_info.get("upper_body_target_override", None)
        if override is not None:
            override = self._clip_upper_body_target(override)
            mask = task_info.get("upper_body_target_override_mask", None)
            if mask is None:
                target = override
            else:
                mask = np.asarray(mask, dtype=bool).reshape(-1)
                if mask.shape[0] != 17:
                    raise ValueError(
                        f"upper_body_target_override_mask must have 17 entries, got {mask.shape[0]}"
                    )
                target = target.copy()
                target[mask] = override[mask]

        delta = task_info.get("upper_body_target_delta", None)
        if delta is None:
            return self._clip_upper_body_target(target)

        delta = np.asarray(delta, dtype=float).reshape(-1)
        if delta.shape[0] != 17:
            raise ValueError(f"upper_body_target_delta must have 17 entries, got {delta.shape[0]}")
        return self._clip_upper_body_target(np.asarray(target, dtype=float).reshape(17) + delta)

    def _teleop_upper_body_mode(self, task_info: dict) -> str:
        mode = str(task_info.get("teleop_upper_body_mode", "cartesian")).strip().lower()
        aliases = {
            "cart": "cartesian",
            "ee": "cartesian",
            "ik": "cartesian",
            "pos": "joint",
            "position": "joint",
            "joint_pos": "joint",
            "joint_position": "joint",
        }
        mode = aliases.get(mode, mode)
        return mode if mode in ("cartesian", "joint", "auto") else "cartesian"

    def _teleop_joint_upper_body_target(self, task_info: dict) -> Optional[np.ndarray]:
        target = task_info.get("teleop_joint_upper_body_target", None)
        if target is None:
            return None

        target = self._clip_upper_body_target(target)
        mask = task_info.get("teleop_joint_upper_body_target_mask", None)
        if mask is None:
            return target

        mask = np.asarray(mask, dtype=bool).reshape(-1)
        if mask.shape[0] != 17:
            raise ValueError(
                f"teleop_joint_upper_body_target_mask must have 17 entries, got {mask.shape[0]}"
            )
        merged = self.last_upper_body_target.copy()
        merged[mask] = target[mask]
        return self._clip_upper_body_target(merged)

    def _upper_body_target(
        self, qpos: np.ndarray, qvel: np.ndarray, agent_feedback: dict, task_info: dict
    ) -> Tuple[np.ndarray, bool]:
        if self.hold_upper_body_during_locomotion and self.transfer_to_loco:
            return self.last_upper_body_target.copy(), True
        teleop_mode = self._teleop_upper_body_mode(task_info)
        teleop_joint_target = self._teleop_joint_upper_body_target(task_info)
        if teleop_mode in ("joint", "auto") and teleop_joint_target is not None:
            target = self._apply_upper_body_safety_target(teleop_joint_target, task_info)
            self.last_upper_body_target = self._filter_upper_body_target(target)
            return self.last_upper_body_target.copy(), True
        if teleop_mode == "joint":
            target = self._apply_upper_body_safety_target(self.last_upper_body_target, task_info)
            self.last_upper_body_target = self._filter_upper_body_target(target)
            return self.last_upper_body_target.copy(), False

        if teleop_mode == "auto" and "target_dof_pos" in task_info:
            target = np.asarray(task_info["target_dof_pos"], dtype=float).reshape(-1)
            if target.shape[0] >= 36:
                target = self._apply_upper_body_safety_target(
                    self._virtual_qpos(target)[19:36], task_info
                )
                self.last_upper_body_target = self._filter_upper_body_target(target)
                return self.last_upper_body_target.copy(), True

        goal_teleop = task_info.get("goal_teleop", {})
        if not task_info.get("arm_goal_enable", True):
            target = self._apply_upper_body_safety_target(self.last_upper_body_target, task_info)
            self.last_upper_body_target = self._filter_upper_body_target(target)
            return self.last_upper_body_target.copy(), True

        robot_base_frame = np.asarray(agent_feedback["robot_base_frame"], dtype=float)
        inv_base = np.linalg.inv(robot_base_frame)

        right_goal = goal_teleop.get("right", None)
        left_goal = goal_teleop.get("left", None)
        right_base = (
            inv_base @ np.asarray(right_goal, dtype=float).reshape(-1, 4, 4)[0]
            if right_goal is not None
            else self._fallback_goal_frame(qpos, "R_ee")
        )
        left_base = (
            inv_base @ np.asarray(left_goal, dtype=float).reshape(-1, 4, 4)[0]
            if left_goal is not None
            else self._fallback_goal_frame(qpos, "L_ee")
        )

        try:
            target = self._apply_upper_body_safety_target(
                self._solve_upper_body_ik(right_base, left_base, qpos, qvel),
                task_info,
            )
            self.last_upper_body_target = self._filter_upper_body_target(target)
            return self.last_upper_body_target.copy(), True
        except Exception as exc:
            if self.wbt_arm_debug:
                print("UnitreeG1WBTPolicy inverse_kinematics error", exc)
            target = self._apply_upper_body_safety_target(self.last_upper_body_target, task_info)
            self.last_upper_body_target = self._filter_upper_body_target(target)
            return self.last_upper_body_target.copy(), False

    def upper_body_target(self, agent_feedback: dict, task_info: dict) -> Tuple[np.ndarray, bool]:
        """Compute the shared waist/arm target without advancing WBT locomotion.

        Locomotion policies such as Sport can reuse the exact same Cartesian
        teleop IK and target filtering while retaining their own lower-body
        recurrent policy. Keeping this operation in the policy layer avoids
        teaching a simulator agent about arm goals or policy composition.
        """
        qpos, qvel = self._qpos_qvel_from_feedback(agent_feedback)
        return self._upper_body_target(qpos, qvel, agent_feedback, task_info)

    def _locoable(self) -> bool:
        return (
            self.squat_controller.squat_cmd[0] > 0.72 and self.squat_controller.squat_cmd[1] < 0.05
        )

    def _stopable(self) -> bool:
        return bool(np.all(np.abs(self.loco_controller.loco_cmd) < self.stop_threshold))

    def _run_squat(self, command, gravity_orientation, omega, qj, dqj) -> None:
        target_dof_pos = self.target_dof_pos_lower_body.copy()
        cmd_raw = self.squat_controller.config.cmd_debug.copy()
        cmd_raw[0] = command[3] * self.squat_command_scale[0]
        cmd_raw[1] = command[4] * self.squat_command_scale[1]
        if self.zero_squat_command:
            cmd_raw[:] = 0.0
        self.target_dof_pos_lower_body = self.squat_controller.run(
            cmd_raw, gravity_orientation, omega, qj, dqj, target_dof_pos
        )
        self._transition_squat(gravity_orientation, omega, qj, dqj)

    def _run_loco(self, command, gravity_orientation, omega, qj, dqj) -> None:
        target_dof_pos = self.target_dof_pos_lower_body.copy()
        cmd_raw = self.loco_controller.config.cmd_debug.copy()
        cmd_raw[:] = command[:3] * self.loco_command_scale
        if self._transition_loco(cmd_raw, gravity_orientation, omega, qj, dqj):
            return
        self.target_dof_pos_lower_body = self.loco_controller.run(
            cmd_raw, gravity_orientation, omega, qj, dqj, target_dof_pos
        )

    def _transition_loco(self, cmd_raw, gravity_orientation, omega, qj, dqj) -> bool:
        if self.loco_controller.transition_count <= 0:
            return False

        # Crossfade two live balance policies.  The legacy WBT runtime blended
        # the last squat target toward a passive default pose for one second,
        # then started the locomotion RNN without a live balance bridge.  That
        # open-loop bridge is contact-solver dependent and tips the robot in
        # PhysX.  Updating both controllers from the observed state keeps the
        # transition closed-loop while preserving a continuous joint target.
        squat_target = self.squat_controller.run(
            None,
            gravity_orientation,
            omega,
            qj,
            dqj,
            self.target_dof_pos_lower_body.copy(),
        )
        loco_target = self.loco_controller.run(
            cmd_raw,
            gravity_orientation,
            omega,
            qj,
            dqj,
            self.target_dof_pos_lower_body.copy(),
        )
        alpha = self.loco_controller.transition_count / self.loco_controller.config.transition_time
        self.target_dof_pos_lower_body = alpha * squat_target + (1.0 - alpha) * loco_target
        self.loco_controller.transition_count -= 1
        return True

    def _transition_squat(self, gravity_orientation, omega, qj, dqj) -> bool:
        if self.squat_controller.transition_count <= 0:
            return False
        target_dof_pos = self.target_dof_pos_lower_body.copy()
        other_policy_target = self.loco_controller.run(
            None, gravity_orientation, omega, qj, dqj, target_dof_pos
        )
        alpha = (
            self.squat_controller.transition_count / self.squat_controller.config.transition_time
        )
        self.target_dof_pos_lower_body = (
            alpha * other_policy_target + (1.0 - alpha) * self.target_dof_pos_lower_body
        )
        self.squat_controller.transition_count -= 1
        return True

    def _update_lower_body_target(
        self,
        command: np.ndarray,
        qpos: np.ndarray,
        qvel: np.ndarray,
        safety_loco_override: bool = False,
        goal_arrived: bool = False,
    ) -> None:
        qj = qpos[7:36].astype(np.float32)
        dqj = qvel[6:35].astype(np.float32)
        gravity_orientation = self._gravity_orientation(qpos[3:7])
        omega = qvel[3:6].astype(np.float32)

        loco_start_threshold = (
            self.safe_loco_start_threshold if safety_loco_override else self.loco_start_threshold
        )
        wants_planar_loco = bool(
            not goal_arrived and np.any(np.abs(command[:3]) > loco_start_threshold)
        )
        base_motion_unsettled = bool(
            np.any(np.abs(qvel[:2]) > self.base_arrival_velocity[:2])
            or abs(qvel[5]) > self.base_arrival_velocity[2]
        )
        # Once walking, keep the balance/gait controller active until the base
        # has actually stopped.  Position-error deadbands alone can become
        # true while the robot still has substantial momentum.
        if self.transfer_to_loco and not wants_planar_loco:
            if base_motion_unsettled:
                self._base_settle_count = 0
            else:
                self._base_settle_count += 1
        else:
            self._base_settle_count = 0
        wants_loco = wants_planar_loco or (
            self.transfer_to_loco and self._base_settle_count < self.base_settle_steps
        )
        if self.transfer_to_squat:
            self._run_squat(command, gravity_orientation, omega, qj, dqj)
            if wants_loco and self._locoable():
                # Arm the live blend before changing mode.  Switching the
                # flags above this block bypasses ``set_transition_count``.
                self.transfer_to_squat = False
                self.transfer_to_loco = True
                self._base_settle_count = 0
                self.squat_controller.transition_count = 0
                self.loco_controller.stance_command = False
                self.loco_controller.set_transition_count()
            return

        if self.transfer_to_loco:
            goal_arrived = bool(
                goal_arrived or (self._base_goal_xy_hold and self._base_goal_yaw_hold)
            )
            if goal_arrived:
                # Arrival is residency-qualified, so keeping the zero-input
                # locomotion network alive only reintroduces gait motion. Hand
                # control to the stationary balance policy on this cycle.
                self._base_settle_count = self.base_settle_steps
                wants_loco = False
            elif wants_planar_loco:
                self.loco_controller.stance_command = False
            self._run_loco(command, gravity_orientation, omega, qj, dqj)
            if not wants_loco and self._stopable():
                self._base_settle_count = 0
                self.loco_controller.transition_count = 0
                self.loco_controller.stance_command = True
                if not self.hold_loco_stance_after_motion:
                    # Return directly to the stationary controller.  The
                    # locomotion-to-squat crossfade evaluates the locomotion
                    # network in its stance phase; that phase is unstable in
                    # PhysX and can tip the robot before the squat policy has
                    # meaningful authority.
                    self.transfer_to_squat = True
                    self.transfer_to_loco = False
                    self.squat_controller.transition_count = 0

    def act(self, agent_feedback: dict, task_info: dict):
        qpos_raw, qvel_raw = self._raw_qpos_qvel_from_feedback(agent_feedback)
        qpos, qvel = self._qpos_qvel_from_feedback(agent_feedback)
        command = self._base_command_from_goal(agent_feedback, task_info)
        safety_loco_override = bool(task_info.get("wbt_loco_safety_override", False))
        goal_arrived = bool(task_info.get("wbt_goal_arrived", False))
        self._update_lower_body_target(
            command,
            qpos,
            qvel,
            safety_loco_override=safety_loco_override,
            goal_arrived=goal_arrived,
        )
        upper_target, ik_success = self._upper_body_target(qpos, qvel, agent_feedback, task_info)

        target_qpos = qpos.copy()
        target_qpos[7:19] = self.target_dof_pos_lower_body
        target_qpos[19:36] = upper_target
        current_actuated_pos = self._actuated_qpos_from_raw(qpos_raw)
        target_actuated_pos = self._actuated_qpos_from_virtual(target_qpos)
        u_ref = self._safe_control_reference(current_actuated_pos, target_actuated_pos)

        motor_kps = self.squat_controller.kps.copy()
        motor_kds = self.squat_controller.kds.copy()
        if self.transfer_to_loco:
            motor_kps = self.loco_controller.kps.copy()
            motor_kds = self.loco_controller.kds.copy()
        motor_kps = motor_kps * self.motor_kp_scale
        motor_kds = motor_kds * self.motor_kd_scale

        info: Dict[str, Any] = {
            "ik_success": ik_success,
            "wbt_mode": "loco" if self.transfer_to_loco else "squat",
            "wbt_command": command.copy(),
            "target_dof_pos": target_qpos.copy(),
            "target_dof_pos_nominal": target_qpos.copy(),
            "target_actuated_pos": target_actuated_pos.copy(),
            "target_actuated_pos_nominal": target_actuated_pos.copy(),
            "target_actuated_pos_safe": target_actuated_pos.copy(),
            "target_dof_vel": np.zeros(self.num_actuated, dtype=float),
            "target_lower_body_pos": self.target_dof_pos_lower_body.copy(),
            "target_upper_body_pos": upper_target.copy(),
            "arm_kinematics_mode": self.arm_kinematics_mode,
            "upper_body_target_rate_limit_disabled": bool(
                self.disable_upper_body_target_rate_limit
            ),
            "upper_body_target_filter_gain": float(self.upper_body_target_filter_gain),
            "upper_body_gravity_compensation": bool(self.upper_body_gravity_compensation),
            "transfer_to_squat": bool(self.transfer_to_squat),
            "transfer_to_loco": bool(self.transfer_to_loco),
            "loco_cmd": self.loco_controller.loco_cmd.copy(),
            "wbt_loco_safety_override": safety_loco_override,
            "wbt_goal_arrived": goal_arrived,
            "wbt_height_control_active": bool(
                abs(command[3]) > max(self.base_command_deadband[3], 1.0e-5)
                or abs(command[4]) > max(self.base_command_deadband[4], 1.0e-5)
            ),
            "base_goal_xy_hold": bool(self._base_goal_xy_hold),
            "base_goal_yaw_hold": bool(self._base_goal_yaw_hold),
            "base_goal_motion_requested": bool(self._base_goal_motion_requested),
            "base_settle_count": int(self._base_settle_count),
            "base_arrival_count": int(self._base_arrival_count),
            "active_loco_start_threshold": (
                self.safe_loco_start_threshold
                if safety_loco_override
                else self.loco_start_threshold
            ),
            "squat_cmd": self.squat_controller.squat_cmd.copy(),
        }
        if self.use_policy_motor_gains:
            info["motor_kps"] = motor_kps
            info["motor_kds"] = motor_kds
        self._add_gripper_info(task_info, info)

        return u_ref, info

    def post_safe_control(
        self,
        agent_feedback: dict,
        task_info: dict,
        action_info: Dict[str, Any],
        u_ref: np.ndarray,
        u_safe: np.ndarray,
        safe_control_info: Dict[str, Any],
    ) -> Dict[str, Any]:
        if not self.apply_safe_control_targets:
            return action_info

        trigger_safe = bool(action_info.get("trigger_safe", False))
        if self.safe_control_only_when_triggered and not trigger_safe:
            return action_info

        qpos_raw, _ = self._raw_qpos_qvel_from_feedback(agent_feedback)
        current_actuated_pos = self._actuated_qpos_from_raw(qpos_raw)
        u_safe = np.asarray(u_safe, dtype=float).reshape(-1)
        if u_safe.shape[0] != current_actuated_pos.shape[0]:
            return action_info

        safe_actuated_pos = current_actuated_pos + u_safe * self.safe_target_dt
        target_qpos = np.asarray(action_info["target_dof_pos"], dtype=float).copy()
        target_qpos[self.body_actuated_qpos_indices] = safe_actuated_pos
        action_info["target_dof_pos"] = target_qpos
        action_info["target_actuated_pos"] = safe_actuated_pos.copy()
        action_info["target_actuated_pos_safe"] = safe_actuated_pos.copy()
        action_info["safe_control_applied_to_target"] = True
        return action_info

    def capture_state(self) -> Dict[str, Any]:
        return {
            "target_dof_pos_lower_body": self.target_dof_pos_lower_body.copy(),
            "last_upper_body_target": self.last_upper_body_target.copy(),
            "last_filtered_upper_body_target": self._last_filtered_upper_body_target.copy(),
            "transfer_to_squat": bool(self.transfer_to_squat),
            "transfer_to_loco": bool(self.transfer_to_loco),
            "last_base_command": None
            if self._last_base_command is None
            else self._last_base_command.copy(),
            "base_goal_xy_hold": bool(self._base_goal_xy_hold),
            "base_goal_yaw_hold": bool(self._base_goal_yaw_hold),
            "last_base_goal_xy": None
            if self._last_base_goal_xy is None
            else self._last_base_goal_xy.copy(),
            "last_base_goal_yaw": self._last_base_goal_yaw,
            "base_goal_motion_requested": bool(self._base_goal_motion_requested),
            "base_settle_count": int(self._base_settle_count),
            "base_arrival_count": int(self._base_arrival_count),
            "loco_transition_count": int(self.loco_controller.transition_count),
            "loco_stance_command": bool(self.loco_controller.stance_command),
            "loco_cmd": self.loco_controller.loco_cmd.copy(),
            "loco_last_policy_target_dof_pos": self.loco_controller.last_policy_target_dof_pos.copy(),
            "loco_hidden_states": self.loco_controller.low_level_policy.hidden_states.copy(),
            "loco_last_action": self.loco_controller.low_level_policy._last_action.copy(),
            "loco_gait_index": float(self.loco_controller.low_level_policy.gait_planner.gait_index),
            "squat_transition_count": int(self.squat_controller.transition_count),
            "squat_cmd": self.squat_controller.squat_cmd.copy(),
            "squat_hidden_states": self.squat_controller.low_level_policy.hidden_states.copy(),
            "squat_last_action": self.squat_controller.low_level_policy._last_action.copy(),
        }

    def restore_state(self, state: Dict[str, Any]) -> None:
        if state.get("target_dof_pos_lower_body", None) is not None:
            self.target_dof_pos_lower_body = state["target_dof_pos_lower_body"].copy()
        if state.get("last_upper_body_target", None) is not None:
            self.last_upper_body_target = state["last_upper_body_target"].copy()
        if state.get("last_filtered_upper_body_target", None) is not None:
            self._last_filtered_upper_body_target = state["last_filtered_upper_body_target"].copy()
        self.transfer_to_squat = bool(state.get("transfer_to_squat", self.transfer_to_squat))
        self.transfer_to_loco = bool(state.get("transfer_to_loco", self.transfer_to_loco))
        if state.get("last_base_command", None) is not None:
            self._last_base_command = state["last_base_command"].copy()
        self._base_goal_xy_hold = bool(state.get("base_goal_xy_hold", self._base_goal_xy_hold))
        self._base_goal_yaw_hold = bool(state.get("base_goal_yaw_hold", self._base_goal_yaw_hold))
        if state.get("last_base_goal_xy", None) is not None:
            self._last_base_goal_xy = state["last_base_goal_xy"].copy()
        if "last_base_goal_yaw" in state:
            self._last_base_goal_yaw = state["last_base_goal_yaw"]
        self._base_goal_motion_requested = bool(
            state.get("base_goal_motion_requested", self._base_goal_motion_requested)
        )
        self._base_settle_count = int(state.get("base_settle_count", self._base_settle_count))
        self._base_arrival_count = int(state.get("base_arrival_count", self._base_arrival_count))

        self.loco_controller.transition_count = int(
            state.get("loco_transition_count", self.loco_controller.transition_count)
        )
        self.loco_controller.stance_command = bool(
            state.get("loco_stance_command", self.loco_controller.stance_command)
        )
        if state.get("loco_cmd", None) is not None:
            self.loco_controller.loco_cmd = state["loco_cmd"].copy()
        if state.get("loco_last_policy_target_dof_pos", None) is not None:
            self.loco_controller.last_policy_target_dof_pos = state[
                "loco_last_policy_target_dof_pos"
            ].copy()
        if state.get("loco_hidden_states", None) is not None:
            self.loco_controller.low_level_policy.hidden_states = state["loco_hidden_states"].copy()
        if state.get("loco_last_action", None) is not None:
            self.loco_controller.low_level_policy._last_action[:] = state["loco_last_action"]
        if state.get("loco_gait_index", None) is not None:
            self.loco_controller.low_level_policy.gait_planner.gait_index = float(
                state["loco_gait_index"]
            )

        self.squat_controller.transition_count = int(
            state.get("squat_transition_count", self.squat_controller.transition_count)
        )
        if state.get("squat_cmd", None) is not None:
            self.squat_controller.squat_cmd = state["squat_cmd"].copy()
        if state.get("squat_hidden_states", None) is not None:
            self.squat_controller.low_level_policy.hidden_states = state[
                "squat_hidden_states"
            ].copy()
        if state.get("squat_last_action", None) is not None:
            self.squat_controller.low_level_policy._last_action[:] = state["squat_last_action"]
