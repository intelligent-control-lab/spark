from __future__ import annotations

import pytest
import torch

from spark_policy.control.whole_body.unitree_g1.sonic.native_batched_policy import (
    ISAACLAB_TO_MUJOCO,
    MUJOCO_TO_ISAACLAB,
    SONIC_DEFAULT_JOINT_POSITION,
    UnitreeG1NativeBatchedSonicPolicy,
)


class ZeroEncoder(torch.nn.Module):
    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        return torch.zeros(observation.shape[0], 64, device=observation.device)


class CountingEncoder(ZeroEncoder):
    def __init__(self) -> None:
        super().__init__()
        self.calls = 0

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        self.calls += 1
        return super().forward(observation)


class CapturingEncoder(ZeroEncoder):
    def __init__(self) -> None:
        super().__init__()
        self.observation: torch.Tensor | None = None

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        self.observation = observation.detach().clone()
        return super().forward(observation)


class ZeroDecoder(torch.nn.Module):
    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        return torch.zeros(observation.shape[0], 29, device=observation.device)


def test_sonic_joint_order_mappings_are_inverses() -> None:
    source = list(range(29))
    isaac = [source[index] for index in MUJOCO_TO_ISAACLAB]
    recovered = [isaac[index] for index in ISAACLAB_TO_MUJOCO]
    assert recovered == source


@pytest.mark.skipif(not torch.cuda.is_available(), reason="native SONIC is CUDA-only")
def test_native_sonic_reset_and_reference_shapes() -> None:
    batch = 2
    policy = UnitreeG1NativeBatchedSonicPolicy(
        batch_size=batch,
        device="cuda:0",
        encoder_model=ZeroEncoder(),
        decoder_model=ZeroDecoder(),
    )
    default = torch.tensor(SONIC_DEFAULT_JOINT_POSITION, device="cuda:0").repeat(batch, 1)
    measured = default + 0.1
    velocity = torch.zeros_like(measured)
    root = torch.zeros(batch, 7, device="cuda:0")
    root[:, 2] = 0.793
    root[:, 6] = 1.0
    angular = torch.zeros(batch, 3, device="cuda:0")
    reference = measured[:, None].repeat(1, 10, 1)

    first, first_info = policy.infer_tensor(
        body_joint_pos=measured,
        body_joint_vel=velocity,
        root_pose_w=root,
        root_angular_velocity=angular,
        reference_joint_position=reference,
        reference_joint_velocity=torch.zeros_like(reference),
    )
    torch.testing.assert_close(first, measured)
    assert first_info["sonic_ok"].all()

    second, _ = policy.infer_tensor(
        body_joint_pos=measured,
        body_joint_vel=velocity,
        root_pose_w=root,
        root_angular_velocity=angular,
        reference_joint_position=measured,
    )
    torch.testing.assert_close(second, default)

    policy.reset(env_ids=torch.tensor([0], device="cuda:0"))
    third, _ = policy.infer_tensor(
        body_joint_pos=measured,
        body_joint_vel=velocity,
        root_pose_w=root,
        root_angular_velocity=angular,
        reference_joint_position=measured,
    )
    torch.testing.assert_close(third[0], measured[0])
    torch.testing.assert_close(third[1], default[1])


@pytest.mark.skipif(not torch.cuda.is_available(), reason="native SONIC is CUDA-only")
def test_external_token_bypasses_encoder_and_updates_history_once() -> None:
    batch = 2
    encoder = CountingEncoder()
    policy = UnitreeG1NativeBatchedSonicPolicy(
        batch_size=batch,
        device="cuda:0",
        encoder_model=encoder,
        decoder_model=ZeroDecoder(),
    )
    measured = torch.tensor(SONIC_DEFAULT_JOINT_POSITION, device="cuda:0").repeat(batch, 1) + 0.1
    velocity = torch.zeros_like(measured)
    root = torch.zeros(batch, 7, device="cuda:0")
    root[:, 2] = 0.793
    root[:, 6] = 1.0
    angular = torch.zeros(batch, 3, device="cuda:0")
    token = torch.randn(batch, policy.token_width, device="cuda:0")

    history_before = policy._joint_position_history.clone()
    _, info = policy.infer_token_tensor(
        body_joint_pos=measured,
        body_joint_vel=velocity,
        root_pose_w=root,
        root_angular_velocity=angular,
        token=token,
    )
    assert encoder.calls == 0
    torch.testing.assert_close(info["sonic_token"], token)
    assert not torch.equal(policy._joint_position_history, history_before)

    history_before_encode = policy._joint_position_history.clone()
    reference = measured[:, None].repeat(1, 10, 1)
    encoded = policy.encode_reference_tensor(
        reference_joint_position=reference,
        reference_joint_velocity=torch.zeros_like(reference),
        root_pose_w=root,
    )
    assert encoder.calls == 1
    assert encoded.shape == token.shape
    torch.testing.assert_close(policy._joint_position_history, history_before_encode)


@pytest.mark.skipif(not torch.cuda.is_available(), reason="native SONIC is CUDA-only")
def test_preview_reference_does_not_advance_recurrent_state() -> None:
    policy = UnitreeG1NativeBatchedSonicPolicy(
        batch_size=2,
        device="cuda:0",
        encoder_model=ZeroEncoder(),
        decoder_model=ZeroDecoder(),
    )
    measured = torch.tensor(SONIC_DEFAULT_JOINT_POSITION, device="cuda:0").repeat(2, 1)
    velocity = torch.zeros_like(measured)
    root = torch.zeros(2, 7, device="cuda:0")
    root[:, 2] = 0.793
    root[:, 6] = 1.0
    angular = torch.zeros(2, 3, device="cuda:0")
    reference = measured[:, None].repeat(1, 10, 1)
    histories = tuple(
        value.clone()
        for value in (
            policy._angular_velocity_history,
            policy._joint_position_history,
            policy._joint_velocity_history,
            policy._action_history,
            policy._gravity_history,
            policy._last_action,
            policy._hold_current,
        )
    )
    policy.preview_tensor(
        body_joint_pos=measured,
        body_joint_vel=velocity,
        root_pose_w=root,
        root_angular_velocity=angular,
        reference_joint_position=reference,
    )
    for actual, expected in zip(
        (
            policy._angular_velocity_history,
            policy._joint_position_history,
            policy._joint_velocity_history,
            policy._action_history,
            policy._gravity_history,
            policy._last_action,
            policy._hold_current,
        ),
        histories,
        strict=True,
    ):
        torch.testing.assert_close(actual, expected)


@pytest.mark.skipif(not torch.cuda.is_available(), reason="native SONIC is CUDA-only")
def test_low_latency_encoder_profile_uses_released_layout() -> None:
    encoder = CapturingEncoder()
    policy = UnitreeG1NativeBatchedSonicPolicy(
        batch_size=1,
        device="cuda:0",
        encoder_model=encoder,
        decoder_model=ZeroDecoder(),
        encoder_profile="low_latency",
    )
    measured = torch.tensor(SONIC_DEFAULT_JOINT_POSITION, device="cuda:0").reshape(1, 29)
    reference = measured[:, None].repeat(1, 10, 1)
    velocity = torch.full_like(reference, 0.25)
    root = torch.zeros(1, 7, device="cuda:0")
    root[:, 2] = 0.793
    root[:, 6] = 1.0

    policy.encode_reference_tensor(
        reference_joint_position=reference,
        reference_joint_velocity=velocity,
        root_pose_w=root,
    )

    assert encoder.observation is not None
    assert encoder.observation.shape == (1, 1247)
    assert policy.reference_frame_dt == pytest.approx(0.02)
    expected_velocity = velocity[..., MUJOCO_TO_ISAACLAB].reshape(1, -1)
    torch.testing.assert_close(encoder.observation[:, 294:584], expected_velocity)
    # Identical root attitudes yield ten identity 6-D rotations.
    expected_orientation = torch.tensor([1.0, 0.0, 0.0, 1.0, 0.0, 0.0], device="cuda:0").repeat(
        1, 10
    )
    torch.testing.assert_close(encoder.observation[:, 584:644], expected_orientation)


def test_unknown_encoder_profile_is_rejected_before_model_loading() -> None:
    with pytest.raises(ValueError, match="unknown SONIC encoder profile"):
        UnitreeG1NativeBatchedSonicPolicy(
            batch_size=1,
            device="cuda:0",
            encoder_model=ZeroEncoder(),
            decoder_model=ZeroDecoder(),
            encoder_profile="not_a_profile",
        )
