"""GPU-native, planner-free batched adapter for the released SONIC policy.

The deployment adapter in :mod:`batched_policy` is the correct path for the
complete SONIC locomotion stack.  It intentionally owns a stateful motion
planner and communicates with the reference C++ runtime.  That design is a
poor fit for thousands of Isaac environments, however: tensors leave CUDA,
are serialized through JSON/ZMQ, and one deployment session is maintained per
environment.

This module implements the narrower controller contract needed by training:

* an upstream component supplies a short desired full-body motion window;
* the released SONIC encoder turns that window into a motion token;
* the released SONIC decoder combines the token with robot history and emits
  the same 29 joint actions as the deployment runtime.

No locomotion planner, transport, safety filter, or motion generator is hidden
inside this class.  Those components remain explicit experiment variables.
"""

from __future__ import annotations

from pathlib import Path
from time import perf_counter
from typing import Any
import warnings

import torch


# Hardware/SPARK order -> IsaacLab policy order and its inverse.  These match
# policy_parameters.hpp in the released SONIC deployment repository.
MUJOCO_TO_ISAACLAB = (
    0,
    6,
    12,
    1,
    7,
    13,
    2,
    8,
    14,
    3,
    9,
    15,
    22,
    4,
    10,
    16,
    23,
    5,
    11,
    17,
    24,
    18,
    25,
    19,
    26,
    20,
    27,
    21,
    28,
)
ISAACLAB_TO_MUJOCO = (
    0,
    3,
    6,
    9,
    13,
    17,
    1,
    4,
    7,
    10,
    14,
    18,
    2,
    5,
    8,
    11,
    15,
    19,
    21,
    23,
    25,
    27,
    12,
    16,
    20,
    22,
    24,
    26,
    28,
)

SONIC_DEFAULT_JOINT_POSITION = (
    -0.312,
    0.0,
    0.0,
    0.669,
    -0.363,
    0.0,
    -0.312,
    0.0,
    0.0,
    0.669,
    -0.363,
    0.0,
    0.0,
    0.0,
    0.0,
    0.2,
    0.2,
    0.0,
    0.6,
    0.0,
    0.0,
    0.0,
    0.2,
    -0.2,
    0.0,
    0.6,
    0.0,
    0.0,
    0.0,
)


def _sonic_action_scale() -> tuple[float, ...]:
    natural_frequency = 10.0 * 2.0 * torch.pi
    stiffness = {
        "5020": 0.003609725 * natural_frequency**2,
        "7520_14": 0.010177520 * natural_frequency**2,
        "7520_22": 0.025101925 * natural_frequency**2,
        "4010": 0.00425 * natural_frequency**2,
    }
    effort = {"5020": 25.0, "7520_14": 88.0, "7520_22": 139.0, "4010": 5.0}
    kinds = (
        "7520_22",
        "7520_22",
        "7520_14",
        "7520_22",
        "5020",
        "5020",
        "7520_22",
        "7520_22",
        "7520_14",
        "7520_22",
        "5020",
        "5020",
        "7520_14",
        "5020",
        "5020",
        "5020",
        "5020",
        "5020",
        "5020",
        "5020",
        "4010",
        "4010",
        "5020",
        "5020",
        "5020",
        "5020",
        "5020",
        "4010",
        "4010",
    )
    return tuple(0.25 * effort[kind] / stiffness[kind] for kind in kinds)


SONIC_ACTION_SCALE = _sonic_action_scale()


# The two released G1 encoders contain the same ten-frame reference signal,
# but it is packed differently.  In particular, the low-latency checkpoint
# removed one unused 17-D block before the root-orientation trajectory.  A
# hard-coded release offset therefore fed its orientation into the wrong
# columns and also built an input that was 515 values too wide.
SONIC_ENCODER_PROFILES = {
    "release": {
        "encoder_width": 1762,
        "reference_position_offset": 4,
        "reference_velocity_offset": 294,
        "reference_orientation_offset": 601,
        "reference_frame_dt": 0.10,
    },
    "low_latency": {
        "encoder_width": 1247,
        "reference_position_offset": 4,
        "reference_velocity_offset": 294,
        "reference_orientation_offset": 584,
        "reference_frame_dt": 0.02,
    },
}


def _load_onnx_as_torch(path: Path, device: torch.device) -> torch.nn.Module:
    try:
        import onnx
        from onnx2torch import convert
    except ImportError as exc:  # pragma: no cover - depends on deployment environment
        raise RuntimeError(
            "Planner-free SONIC requires the 'onnx' and 'onnx2torch' packages"
        ) from exc
    if not path.is_file():
        raise FileNotFoundError(f"SONIC ONNX model does not exist: {path}")
    # onnx2torch currently emits these compatibility warnings on every
    # inference call under PyTorch 2.8. They do not affect today's converted
    # model, but can otherwise grow a one-hour training log by several MiB.
    warnings.filterwarnings(
        "ignore",
        message=r"Using a non-tuple sequence for multidimensional indexing.*",
        category=UserWarning,
        module=r"onnx2torch\.node_converters\..*",
    )
    model = convert(onnx.load(str(path))).eval().to(device=device, dtype=torch.float32)
    model.requires_grad_(False)
    return model


def _model_output(value: Any) -> torch.Tensor:
    if isinstance(value, torch.Tensor):
        return value
    if isinstance(value, (tuple, list)) and len(value) == 1:
        return _model_output(value[0])
    if isinstance(value, dict) and len(value) == 1:
        return _model_output(next(iter(value.values())))
    raise TypeError(f"Unexpected SONIC model output type: {type(value).__name__}")


def _quat_conjugate_xyzw(quaternion: torch.Tensor) -> torch.Tensor:
    return torch.cat((-quaternion[..., :3], quaternion[..., 3:4]), dim=-1)


def _quat_multiply_xyzw(left: torch.Tensor, right: torch.Tensor) -> torch.Tensor:
    left_xyz, left_w = left[..., :3], left[..., 3:4]
    right_xyz, right_w = right[..., :3], right[..., 3:4]
    xyz = left_w * right_xyz + right_w * left_xyz + torch.linalg.cross(left_xyz, right_xyz, dim=-1)
    w = left_w * right_w - (left_xyz * right_xyz).sum(dim=-1, keepdim=True)
    return torch.cat((xyz, w), dim=-1)


def _normalize_quaternion(quaternion: torch.Tensor) -> torch.Tensor:
    return quaternion / torch.linalg.vector_norm(quaternion, dim=-1, keepdim=True).clamp_min(1.0e-8)


def _quat_rotate_xyzw(quaternion: torch.Tensor, vector: torch.Tensor) -> torch.Tensor:
    q_xyz = quaternion[..., :3]
    q_w = quaternion[..., 3:4]
    cross = torch.linalg.cross(q_xyz, vector, dim=-1)
    return vector + 2.0 * (q_w * cross + torch.linalg.cross(q_xyz, cross, dim=-1))


def _rotation_6d_xyzw(quaternion: torch.Tensor) -> torch.Tensor:
    """First two rotation-matrix columns, flattened row-major like SONIC."""
    x, y, z, w = quaternion.unbind(dim=-1)
    return torch.stack(
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
        ),
        dim=-1,
    )


class UnitreeG1NativeBatchedSonicPolicy:
    """Run the released SONIC encoder/decoder directly on batched CUDA tensors.

    ``reference_joint_position`` is an absolute desired joint trajectory in
    SPARK/hardware order.  It can be ``[B, 29]`` (a pose repeated across the
    future window) or ``[B, 10, 29]``.  The latter is the intended interface
    for an upstream motion generator.  Reference velocities and world-frame
    root orientations are optional; zero velocity and the measured root
    orientation are safe defaults for a hold/reference-pose benchmark.
    """

    num_reference_frames = 10
    history_length = 10
    token_width = 64
    encoder_width = 1762
    decoder_width = 994

    # Offsets follow observation_config.yaml.  Unused teleop/SMPL features are
    # deliberately zero, exactly as the released mode-filtered G1 encoder path.
    encoder_mode_offset = 0
    reference_position_offset = 4
    reference_velocity_offset = 294
    reference_orientation_offset = 601

    def __init__(
        self,
        *,
        batch_size: int,
        device: str | torch.device = "cuda:0",
        encoder_path: str | Path | None = None,
        decoder_path: str | Path | None = None,
        encoder_model: torch.nn.Module | None = None,
        decoder_model: torch.nn.Module | None = None,
        hold_first_target: bool = True,
        encoder_profile: str = "release",
    ) -> None:
        if int(batch_size) < 1:
            raise ValueError("batch_size must be positive")
        self.batch_size = int(batch_size)
        self.device = torch.device(device)
        if self.device.type != "cuda":
            raise ValueError("planner-free SONIC is intended for a CUDA training device")

        try:
            profile = SONIC_ENCODER_PROFILES[encoder_profile]
        except KeyError as exc:
            choices = ", ".join(sorted(SONIC_ENCODER_PROFILES))
            raise ValueError(
                f"unknown SONIC encoder profile {encoder_profile!r}; choose {choices}"
            ) from exc
        self.encoder_profile = encoder_profile
        self.encoder_width = int(profile["encoder_width"])
        self.reference_position_offset = int(profile["reference_position_offset"])
        self.reference_velocity_offset = int(profile["reference_velocity_offset"])
        self.reference_orientation_offset = int(profile["reference_orientation_offset"])
        self.reference_frame_dt = float(profile["reference_frame_dt"])

        load_start = perf_counter()
        if encoder_model is None:
            if encoder_path is None:
                raise ValueError("encoder_path is required when encoder_model is not supplied")
            encoder_model = _load_onnx_as_torch(Path(encoder_path), self.device)
        if decoder_model is None:
            if decoder_path is None:
                raise ValueError("decoder_path is required when decoder_model is not supplied")
            decoder_model = _load_onnx_as_torch(Path(decoder_path), self.device)
        self.encoder = encoder_model.eval().to(self.device)
        self.decoder = decoder_model.eval().to(self.device)
        self.encoder.requires_grad_(False)
        self.decoder.requires_grad_(False)
        self.model_load_seconds = perf_counter() - load_start

        self._mujoco_to_isaac = torch.tensor(
            MUJOCO_TO_ISAACLAB, device=self.device, dtype=torch.long
        )
        self._isaac_to_mujoco = torch.tensor(
            ISAACLAB_TO_MUJOCO, device=self.device, dtype=torch.long
        )
        self.default_joint_position = torch.tensor(
            SONIC_DEFAULT_JOINT_POSITION, device=self.device, dtype=torch.float32
        )
        self.default_joint_position_isaac = self.default_joint_position[self._mujoco_to_isaac]
        self.action_scale = torch.tensor(
            SONIC_ACTION_SCALE, device=self.device, dtype=torch.float32
        )

        self._joint_position_history = self._history(29)
        self._joint_velocity_history = self._history(29)
        self._action_history = self._history(29)
        self._angular_velocity_history = self._history(3)
        self._gravity_history = self._history(3)
        self._last_action = torch.zeros(
            self.batch_size, 29, device=self.device, dtype=torch.float32
        )
        self._hold_current = torch.full(
            (self.batch_size,),
            bool(hold_first_target),
            device=self.device,
            dtype=torch.bool,
        )
        self.hold_first_target = bool(hold_first_target)

    @property
    def num_envs(self) -> int:
        return self.batch_size

    def _history(self, width: int) -> torch.Tensor:
        return torch.zeros(
            self.batch_size,
            self.history_length,
            width,
            device=self.device,
            dtype=torch.float32,
        )

    def close(self) -> None:
        # Kept for interface parity with the deployment-backed adapter.
        return None

    @torch.inference_mode()
    def reset(self, context=None, env_ids=None) -> None:
        del context
        if env_ids is None:
            ids = torch.arange(self.batch_size, device=self.device)
        else:
            ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long).reshape(-1)
        for history in (
            self._joint_position_history,
            self._joint_velocity_history,
            self._action_history,
            self._angular_velocity_history,
            self._gravity_history,
        ):
            history[ids] = 0.0
        self._last_action[ids] = 0.0
        self._hold_current[ids] = self.hold_first_target

    def _append(self, history: torch.Tensor, value: torch.Tensor) -> torch.Tensor:
        return torch.cat((history[:, 1:], value[:, None]), dim=1)

    def _reference_window(self, value: torch.Tensor, width: int, name: str) -> torch.Tensor:
        value = torch.as_tensor(value, device=self.device, dtype=torch.float32)
        if value.shape == (self.batch_size, width):
            return value[:, None].expand(-1, self.num_reference_frames, -1)
        expected = (self.batch_size, self.num_reference_frames, width)
        if value.shape != expected:
            raise ValueError(
                f"{name} must have shape {(self.batch_size, width)} or "
                f"{expected}, got {tuple(value.shape)}"
            )
        return value

    def _encode_reference(
        self,
        reference_joint_position: torch.Tensor,
        reference_joint_velocity: torch.Tensor | None,
        reference_root_quat_xyzw: torch.Tensor | None,
        measured_root_quat_xyzw: torch.Tensor,
    ) -> torch.Tensor:
        position = self._reference_window(reference_joint_position, 29, "reference_joint_position")[
            ..., self._mujoco_to_isaac
        ]
        velocity = (
            torch.zeros_like(position)
            if reference_joint_velocity is None
            else self._reference_window(reference_joint_velocity, 29, "reference_joint_velocity")[
                ..., self._mujoco_to_isaac
            ]
        )
        if reference_root_quat_xyzw is None:
            reference_quat = measured_root_quat_xyzw[:, None].expand(
                -1, self.num_reference_frames, -1
            )
        else:
            reference_quat = self._reference_window(
                reference_root_quat_xyzw, 4, "reference_root_quat_xyzw"
            )
        reference_quat = _normalize_quaternion(reference_quat)
        measured = _normalize_quaternion(measured_root_quat_xyzw)
        relative = _quat_multiply_xyzw(_quat_conjugate_xyzw(measured)[:, None], reference_quat)
        orientation_6d = _rotation_6d_xyzw(relative)

        encoder_input = torch.zeros(
            self.batch_size,
            self.encoder_width,
            device=self.device,
            dtype=torch.float32,
        )
        # encoder_mode_4 is [0, 0, 0, 0] for the G1 motion-reference encoder.
        encoder_input[:, self.reference_position_offset : self.reference_position_offset + 290] = (
            position.reshape(self.batch_size, -1)
        )
        encoder_input[:, self.reference_velocity_offset : self.reference_velocity_offset + 290] = (
            velocity.reshape(self.batch_size, -1)
        )
        encoder_input[
            :,
            self.reference_orientation_offset : self.reference_orientation_offset + 60,
        ] = orientation_6d.reshape(self.batch_size, -1)
        return _model_output(self.encoder(encoder_input))

    @torch.inference_mode()
    def encode_reference_tensor(
        self,
        *,
        reference_joint_position: torch.Tensor,
        root_pose_w: torch.Tensor,
        reference_joint_velocity: torch.Tensor | None = None,
        reference_root_quat_xyzw: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """Encode a reference without advancing the controller history.

        This exposes SONIC's 64-D motion token to a batched upstream policy.
        Keeping encoding separate from decoding makes it possible to apply a
        small task-conditioned token residual while retaining an ordinary,
        controller-independent reference as the anchor.
        """

        root_pose_w = torch.as_tensor(root_pose_w, device=self.device, dtype=torch.float32)
        if root_pose_w.shape != (self.batch_size, 7):
            raise ValueError(
                f"root_pose_w must have shape {(self.batch_size, 7)}, "
                f"got {tuple(root_pose_w.shape)}"
            )
        return self._encode_reference(
            reference_joint_position,
            reference_joint_velocity,
            reference_root_quat_xyzw,
            _normalize_quaternion(root_pose_w[:, 3:7]),
        )

    def _update_feedback_history(
        self,
        *,
        body_joint_pos: torch.Tensor,
        body_joint_vel: torch.Tensor,
        root_pose_w: torch.Tensor,
        root_angular_velocity: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        body_joint_pos = torch.as_tensor(body_joint_pos, device=self.device, dtype=torch.float32)
        body_joint_vel = torch.as_tensor(body_joint_vel, device=self.device, dtype=torch.float32)
        root_pose_w = torch.as_tensor(root_pose_w, device=self.device, dtype=torch.float32)
        root_angular_velocity = torch.as_tensor(
            root_angular_velocity, device=self.device, dtype=torch.float32
        )
        expected_joint = (self.batch_size, 29)
        if body_joint_pos.shape != expected_joint or body_joint_vel.shape != expected_joint:
            raise ValueError(
                f"body joint tensors must both have shape {expected_joint}, got "
                f"{tuple(body_joint_pos.shape)} and {tuple(body_joint_vel.shape)}"
            )
        if root_pose_w.shape != (self.batch_size, 7):
            raise ValueError(
                f"root_pose_w must have shape {(self.batch_size, 7)}, "
                f"got {tuple(root_pose_w.shape)}"
            )
        if root_angular_velocity.shape != (self.batch_size, 3):
            raise ValueError(
                "root_angular_velocity must have shape "
                f"{(self.batch_size, 3)}, got {tuple(root_angular_velocity.shape)}"
            )

        root_quat = _normalize_quaternion(root_pose_w[:, 3:7])
        position_isaac = (
            body_joint_pos[:, self._mujoco_to_isaac] - self.default_joint_position_isaac
        )
        velocity_isaac = body_joint_vel[:, self._mujoco_to_isaac]
        gravity_world = torch.zeros(self.batch_size, 3, device=self.device, dtype=torch.float32)
        gravity_world[:, 2] = -1.0
        gravity_body = _quat_rotate_xyzw(_quat_conjugate_xyzw(root_quat), gravity_world)

        self._joint_position_history = self._append(self._joint_position_history, position_isaac)
        self._joint_velocity_history = self._append(self._joint_velocity_history, velocity_isaac)
        self._action_history = self._append(self._action_history, self._last_action)
        self._angular_velocity_history = self._append(
            self._angular_velocity_history, root_angular_velocity
        )
        self._gravity_history = self._append(self._gravity_history, gravity_body)
        return body_joint_pos, root_quat

    def _decode_token(
        self, token: torch.Tensor, body_joint_pos: torch.Tensor
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        token = torch.as_tensor(token, device=self.device, dtype=torch.float32)
        token_width = self.decoder_width - (self.history_length * (3 + 29 + 29 + 29 + 3))
        if token_width != self.token_width:
            raise RuntimeError(
                f"configured token width {self.token_width} does not match "
                f"decoder width {token_width}"
            )
        if token.shape != (self.batch_size, token_width):
            raise ValueError(
                f"SONIC token must have shape {(self.batch_size, token_width)}, "
                f"got {tuple(token.shape)}"
            )
        decoder_input = torch.cat(
            (
                token,
                self._angular_velocity_history.flatten(start_dim=1),
                self._joint_position_history.flatten(start_dim=1),
                self._joint_velocity_history.flatten(start_dim=1),
                self._action_history.flatten(start_dim=1),
                self._gravity_history.flatten(start_dim=1),
            ),
            dim=-1,
        )
        if decoder_input.shape != (self.batch_size, self.decoder_width):
            raise RuntimeError(
                f"SONIC decoder observation has shape {tuple(decoder_input.shape)}, "
                f"expected {(self.batch_size, self.decoder_width)}"
            )
        action_isaac = _model_output(self.decoder(decoder_input))
        valid = torch.isfinite(action_isaac).all(dim=-1)
        safe_action = torch.where(valid[:, None], action_isaac, self._last_action)
        self._last_action.copy_(safe_action)

        target = (
            self.default_joint_position + safe_action[:, self._isaac_to_mujoco] * self.action_scale
        )
        valid_target = valid & torch.isfinite(target).all(dim=-1)
        target = torch.where(valid_target[:, None], target, body_joint_pos)
        target = torch.where(self._hold_current[:, None], body_joint_pos, target)
        self._hold_current[:] = False
        return target, {
            "sonic_ok": valid_target,
            "sonic_token": token,
            "sonic_raw_action": safe_action,
        }

    @torch.inference_mode()
    def infer_tensor(
        self,
        *,
        body_joint_pos: torch.Tensor,
        body_joint_vel: torch.Tensor,
        root_pose_w: torch.Tensor,
        root_angular_velocity: torch.Tensor,
        reference_joint_position: torch.Tensor,
        reference_joint_velocity: torch.Tensor | None = None,
        reference_root_quat_xyzw: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        body_joint_pos, root_quat = self._update_feedback_history(
            body_joint_pos=body_joint_pos,
            body_joint_vel=body_joint_vel,
            root_pose_w=root_pose_w,
            root_angular_velocity=root_angular_velocity,
        )

        token = self._encode_reference(
            reference_joint_position,
            reference_joint_velocity,
            reference_root_quat_xyzw,
            root_quat,
        )
        return self._decode_token(token, body_joint_pos)

    @torch.inference_mode()
    def preview_tensor(
        self,
        *,
        body_joint_pos: torch.Tensor,
        body_joint_vel: torch.Tensor,
        root_pose_w: torch.Tensor,
        root_angular_velocity: torch.Tensor,
        reference_joint_position: torch.Tensor,
        reference_joint_velocity: torch.Tensor | None = None,
        reference_root_quat_xyzw: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        """Evaluate one reference without advancing recurrent controller state.

        The history append operation replaces its tensors, so retaining their
        references is sufficient.  Decoder action/hold buffers are mutated in
        place and are therefore cloned and restored.  This permits a batched
        upstream reference governor to score one proposal while committing
        exactly one SONIC state transition per physical control step.
        """

        histories = (
            self._angular_velocity_history,
            self._joint_position_history,
            self._joint_velocity_history,
            self._action_history,
            self._gravity_history,
        )
        last_action = self._last_action.clone()
        hold_current = self._hold_current.clone()
        try:
            return self.infer_tensor(
                body_joint_pos=body_joint_pos,
                body_joint_vel=body_joint_vel,
                root_pose_w=root_pose_w,
                root_angular_velocity=root_angular_velocity,
                reference_joint_position=reference_joint_position,
                reference_joint_velocity=reference_joint_velocity,
                reference_root_quat_xyzw=reference_root_quat_xyzw,
            )
        finally:
            (
                self._angular_velocity_history,
                self._joint_position_history,
                self._joint_velocity_history,
                self._action_history,
                self._gravity_history,
            ) = histories
            self._last_action.copy_(last_action)
            self._hold_current.copy_(hold_current)

    @torch.inference_mode()
    def infer_token_tensor(
        self,
        *,
        body_joint_pos: torch.Tensor,
        body_joint_vel: torch.Tensor,
        root_pose_w: torch.Tensor,
        root_angular_velocity: torch.Tensor,
        token: torch.Tensor,
    ) -> tuple[torch.Tensor, dict[str, torch.Tensor]]:
        """Decode an externally steered SONIC token at full batch size."""

        body_joint_pos, _ = self._update_feedback_history(
            body_joint_pos=body_joint_pos,
            body_joint_vel=body_joint_vel,
            root_pose_w=root_pose_w,
            root_angular_velocity=root_angular_velocity,
        )
        return self._decode_token(token, body_joint_pos)


__all__ = [
    "ISAACLAB_TO_MUJOCO",
    "MUJOCO_TO_ISAACLAB",
    "SONIC_ACTION_SCALE",
    "SONIC_DEFAULT_JOINT_POSITION",
    "SONIC_ENCODER_PROFILES",
    "UnitreeG1NativeBatchedSonicPolicy",
]
