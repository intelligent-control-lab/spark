"""Parallel Isaac adapter for independent scalar SONIC policy servers."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import numpy as np

from spark_policy.control.whole_body.unitree_g1.wbt.config import (
    WBT_MOTOR_KDS,
    WBT_MOTOR_KPS,
)

from .client import SonicClient
from .policy import SPARK_TO_SONIC_UPPER_BODY, SonicBaseTracker


class UnitreeG1BatchedSonicPolicy:
    """Run one isolated, stateful SONIC session per environment concurrently."""

    def __init__(
        self,
        endpoints,
        *,
        batch_size=None,
        device="cuda:0",
        timeout_ms=100,
        upper_body_target_rate_limit=None,
        locomotion_mode="walk",
        slowdown_distance=0.50,
        upper_body_target_filter_gain=0.05,
        sonic_default_height=0.793,
        sonic_height_min=0.2,
        sonic_height_max=1.0,
        sonic_height_deadband=0.01,
        sonic_height_filter_gain=0.35,
        sonic_height_rate_limit=0.006,
    ):
        import torch

        self.torch = torch
        self.device = str(device)
        self.endpoints = tuple(str(endpoint) for endpoint in endpoints)
        if not self.endpoints:
            raise ValueError("At least one SONIC endpoint is required")
        self._num_envs = len(self.endpoints) if batch_size is None else int(batch_size)
        self._shared_server = len(self.endpoints) == 1 and self._num_envs > 1
        if not self._shared_server and self._num_envs != len(self.endpoints):
            raise ValueError("batch_size must match endpoints unless using one shared server")
        self.clients = [SonicClient(endpoint, timeout_ms=timeout_ms) for endpoint in self.endpoints]
        self._pool = (
            None
            if self._shared_server
            else ThreadPoolExecutor(max_workers=len(self.clients), thread_name_prefix="sonic-env")
        )
        self._seq = np.zeros(self._num_envs, dtype=np.int64)
        self._last_target = None
        # A freshly constructed scalar SONIC policy is reset by the pipeline
        # before its first action.  The tensor runner owns no outer pipeline,
        # so make that initial episode boundary explicit for every server row.
        self._reset_pending = np.ones(self._num_envs, dtype=bool)
        self._hold_current = np.ones(self._num_envs, dtype=bool)
        self._upper_reset_pending = np.ones(self._num_envs, dtype=bool)
        self._last_upper_target = None
        self._last_planner_movement = [None] * self._num_envs
        self._last_planner_facing = [None] * self._num_envs
        default_rate_limit = [0.006, 0.006, 0.006] + [0.014] * 14
        rate_limit = (
            default_rate_limit
            if upper_body_target_rate_limit is None
            else upper_body_target_rate_limit
        )
        self.upper_body_target_rate_limit = np.asarray(rate_limit, dtype=np.float32).reshape(17)
        self.upper_body_target_filter_gain = float(upper_body_target_filter_gain)
        self.locomotion_mode = str(locomotion_mode).lower()
        if self.locomotion_mode not in ("slow", "walk", "run", "hybrid"):
            raise ValueError("locomotion_mode must be slow, walk, run, or hybrid")
        self.slowdown_distance = float(slowdown_distance)
        self.sonic_default_height = float(sonic_default_height)
        self.sonic_height_min = float(sonic_height_min)
        self.sonic_height_max = float(sonic_height_max)
        self.sonic_height_deadband = float(sonic_height_deadband)
        self.sonic_height_filter_gain = float(sonic_height_filter_gain)
        self.sonic_height_rate_limit = float(sonic_height_rate_limit)
        self._last_sonic_height = np.full(self._num_envs, self.sonic_default_height, dtype=float)
        self._base_trackers = [
            SonicBaseTracker(
                0.02,
                [0.20, 0.20, 0.50, 0.10, 0.10],
                [0.80, 0.80, 2.00, 0.40, 0.40],
                [0.50, 0.50, 1.20, 0.20, 0.20],
                [3.00, 3.00, 6.00, 1.00, 1.00],
            )
            for _ in range(self._num_envs)
        ]

    @property
    def num_envs(self):
        return self._num_envs

    def close(self):
        for client in self.clients:
            client.close()
        if self._pool is not None:
            self._pool.shutdown(wait=True, cancel_futures=True)

    def reset(self, context=None, env_ids=None):
        ids = (
            range(self.num_envs)
            if env_ids is None
            else (env_ids.detach().cpu().tolist() if hasattr(env_ids, "detach") else list(env_ids))
        )
        for env_id in ids:
            env_id = int(env_id)
            self._seq[env_id] = 0
            self._reset_pending[env_id] = True
            self._hold_current[env_id] = True
            self._upper_reset_pending[env_id] = True
            self._last_sonic_height[env_id] = self.sonic_default_height
        self.reset_base_tracking(ids)

    def reset_base_tracking(self, env_ids=None):
        """Restart only locomotion command shaping for selected rows.

        This is deliberately lighter than an episode reset: policy history and
        upper-body targets remain intact while a stalled near-goal gait gets a
        clean acceleration ramp and planner direction decision.
        """
        ids = (
            range(self.num_envs)
            if env_ids is None
            else (env_ids.detach().cpu().tolist() if hasattr(env_ids, "detach") else list(env_ids))
        )
        for env_id in ids:
            env_id = int(env_id)
            self._base_trackers[env_id].reset()
            self._last_planner_movement[env_id] = None
            self._last_planner_facing[env_id] = None

    def _planner(self, env_id=None, command=None, root_quat_xyzw=None, goal_distance=None):
        # Preserve the original class-level helper API used by diagnostics:
        # UnitreeG1BatchedSonicPolicy._planner(command, quaternion). Stateful
        # inference calls the bound three-argument form with an environment id.
        stateful = isinstance(self, UnitreeG1BatchedSonicPolicy)
        if not stateful:
            root_quat_xyzw = env_id
            command = self
            env_id = None
        command = np.asarray(command, dtype=float)
        qx, qy, qz, qw = np.asarray(root_quat_xyzw, dtype=float)
        yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        c, s = np.cos(yaw), np.sin(yaw)
        vx, vy = float(command[0]), float(command[1])
        # Match the scalar Sonic adapter: the third base command is a yaw
        # correction, represented to the planner as a desired facing vector.
        # The tensor goal controller uses a yaw gain of 0.8 by default.
        facing_yaw = yaw + float(command[2]) / 0.8
        facing = [float(np.cos(facing_yaw)), float(np.sin(facing_yaw)), 0.0]
        # Planar benchmark commands contain x/y/yaw only.  A fourth value is
        # reserved for an explicit absolute height target from a safety
        # composition; -1 delegates height to SONIC's nominal locomotion.
        height = float(command[3]) if command.size > 3 else -1.0
        world = np.array((c * vx - s * vy, s * vx + c * vy))
        raw_speed = float(np.linalg.norm(world))
        if raw_speed < 0.02:
            if stateful:
                self._last_planner_movement[env_id] = None
                self._last_planner_facing[env_id] = np.asarray(facing[:2], dtype=float)
            return {
                "mode": 0,
                "movement": [0.0, 0.0, 0.0],
                "facing": facing,
                "speed": -1.0,
                "height": height,
            }
        movement = world / max(raw_speed, 1.0e-6)
        speed = raw_speed * 1.5
        # Match the scalar PID adapter: request the WALK expert while far
        # from a pose goal, then switch to SLOW_WALK for accurate stopping.
        # Selecting a gait only from the conservative 0.12 m/s PID envelope
        # left every batched request in SLOW_WALK and produced almost no net
        # displacement in Isaac.
        far_from_goal = (
            (goal_distance is None or float(goal_distance) > self.slowdown_distance)
            if stateful
            else False
        )
        if stateful and self.locomotion_mode == "walk" and far_from_goal:
            mode = 2  # SONIC_LOCOMOTION_WALK
            speed = float(np.clip(speed, 0.8, 2.5))
        elif stateful and self.locomotion_mode == "run" and far_from_goal:
            mode = 3  # SONIC_LOCOMOTION_RUN
            speed = float(np.clip(speed, 2.5, 3.0))
        elif speed <= 0.8:
            mode = 1  # SONIC_LOCOMOTION_SLOW_WALK
            speed = float(np.clip(speed, 0.2, 0.8))
        elif speed <= 2.5:
            mode = 2  # SONIC_LOCOMOTION_WALK
            speed = float(np.clip(speed, 0.8, 2.5))
        else:
            mode = 3  # SONIC_LOCOMOTION_RUN
            speed = float(np.clip(speed, 2.5, 3.0))

        def hold_direction(candidate, previous, threshold):
            candidate = np.asarray(candidate, dtype=float).reshape(2)
            if previous is None:
                return candidate.copy()
            dot = float(np.clip(np.dot(candidate, previous), -1.0, 1.0))
            return previous.copy() if np.arccos(dot) < threshold else candidate.copy()

        movement = hold_direction(
            movement,
            self._last_planner_movement[env_id] if stateful else None,
            0.12,
        )
        # Facing is a pose-control target, not merely a gait-selection hint.
        # Reusing the 0.12 rad movement hysteresis can leave the robot parked
        # just outside the benchmark's 0.10 rad yaw tolerance indefinitely.
        # Keep only a small anti-jitter threshold so the desired heading keeps
        # advancing as an IDLE Sonic row turns in place.
        facing_xy = hold_direction(
            facing[:2],
            self._last_planner_facing[env_id] if stateful else None,
            0.02,
        )
        if stateful:
            self._last_planner_movement[env_id] = movement.copy()
            self._last_planner_facing[env_id] = facing_xy.copy()
        facing = [float(facing_xy[0]), float(facing_xy[1]), 0.0]
        speed = max(0.2, round(speed / 0.05) * 0.05)
        request = {
            "mode": mode,
            "movement": [float(movement[0]), float(movement[1]), 0.0],
            "facing": facing,
            "speed": speed,
            "height": height,
        }
        return request

    def _request(
        self,
        env_id,
        body_pos,
        body_vel,
        root_pose,
        root_ang_vel,
        command,
        upper_target,
        goal_distance=None,
    ):
        self._seq[env_id] += 1
        root_pose = np.asarray(root_pose, dtype=float)
        xyzw = root_pose[3:7]
        quat_wxyz = [float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2])]
        body_pos = np.asarray(body_pos, dtype=float)
        body_vel = np.asarray(body_vel, dtype=float)
        full_qpos = np.concatenate((root_pose[:3], quat_wxyz, body_pos))
        full_qvel = np.concatenate((np.zeros(3), np.asarray(root_ang_vel, dtype=float), body_vel))
        upper_target = np.asarray(upper_target, dtype=float).reshape(-1)
        if upper_target.shape[0] == 29:
            upper_target = upper_target[12:29]
        elif upper_target.shape[0] != 17:
            raise ValueError(
                f"SONIC upper-body target must contain 17 or 29 joints, got {upper_target.shape[0]}"
            )
        upper = upper_target[SPARK_TO_SONIC_UPPER_BODY]
        request = {
            "protocol": "spark_sonic_policy_v1",
            "seq": int(self._seq[env_id]),
            "dt": 0.02,
            "body_qpos": full_qpos.tolist(),
            "body_qvel": full_qvel.tolist(),
            "actuated_qpos": body_pos.tolist(),
            "actuated_qvel": body_vel.tolist(),
            "base_quat": quat_wxyz,
            "base_ang_vel": np.asarray(root_ang_vel, dtype=float).tolist(),
            "planner": self._planner(env_id, command, xyzw, goal_distance),
            "upper_body_position": upper.tolist(),
            "start": True,
        }
        if self._reset_pending[env_id]:
            request["reset"] = True
            self._reset_pending[env_id] = False
        return request

    def _filtered_height(self, env_id, target):
        """Match scalar SONIC height semantics for one tensor environment."""
        target = float(target)
        if target < 0.0 or abs(target - self.sonic_default_height) < self.sonic_height_deadband:
            self._last_sonic_height[env_id] = self.sonic_default_height
            return -1.0
        target = float(np.clip(target, self.sonic_height_min, self.sonic_height_max))
        delta = self.sonic_height_filter_gain * (target - self._last_sonic_height[env_id])
        if self.sonic_height_rate_limit > 0.0:
            delta = float(
                np.clip(
                    delta,
                    -self.sonic_height_rate_limit,
                    self.sonic_height_rate_limit,
                )
            )
        self._last_sonic_height[env_id] = float(
            np.clip(
                self._last_sonic_height[env_id] + delta,
                self.sonic_height_min,
                self.sonic_height_max,
            )
        )
        if (
            abs(self._last_sonic_height[env_id] - self.sonic_default_height)
            < self.sonic_height_deadband
        ):
            return -1.0
        return self._last_sonic_height[env_id]

    def infer_tensor(
        self,
        *,
        body_joint_pos,
        body_joint_vel,
        root_pose_w,
        root_angular_velocity,
        command,
        upper_body_target,
        base_goal_distance=None,
    ):
        arrays = [
            value.detach().cpu().numpy()
            for value in (
                body_joint_pos,
                body_joint_vel,
                root_pose_w,
                root_angular_velocity,
                command,
                upper_body_target,
            )
        ]
        if arrays[0].shape[0] != self.num_envs:
            raise ValueError(f"Expected {self.num_envs} SONIC rows, got {arrays[0].shape[0]}")
        measured_upper = arrays[0][:, 12:29]
        if self._last_upper_target is None:
            self._last_upper_target = measured_upper.copy()
        self._last_upper_target[self._upper_reset_pending] = measured_upper[
            self._upper_reset_pending
        ]
        self._upper_reset_pending[:] = False
        upper_delta = np.clip(
            self.upper_body_target_filter_gain * (arrays[5] - self._last_upper_target),
            -self.upper_body_target_rate_limit,
            self.upper_body_target_rate_limit,
        )
        filtered_upper = self._last_upper_target + upper_delta
        self._last_upper_target = filtered_upper.copy()
        arrays[5] = filtered_upper
        # Match scalar teleop's jerk-limited SONIC base tracker.  Without this
        # transition, the benchmark jumps from IDLE directly into a gait on
        # the first post-warmup control tick.
        tracked_command = arrays[4].copy()
        for env_id, tracker in enumerate(self._base_trackers):
            command5 = np.zeros(5, dtype=np.float32)
            width = min(5, tracked_command.shape[1])
            command5[:width] = tracked_command[env_id, :width]
            explicit_height = command5[3] if width > 3 else None
            # Height is an absolute SONIC planner target, not a velocity-like
            # base command. Filter it around the nominal posture separately.
            command5[3] = 0.0
            tracked_command[env_id, :width] = tracker.update(command5)[:width]
            if explicit_height is not None:
                tracked_command[env_id, 3] = self._filtered_height(env_id, explicit_height)
        arrays[4] = tracked_command
        if base_goal_distance is None:
            goal_distances = [None] * self.num_envs
        else:
            goal_distances = base_goal_distance.detach().cpu().numpy().reshape(-1)
            if goal_distances.shape[0] != self.num_envs:
                raise ValueError(
                    f"Expected {self.num_envs} goal distances, got {goal_distances.shape[0]}"
                )
        requests = [
            self._request(i, *(array[i] for array in arrays), goal_distance=goal_distances[i])
            for i in range(self.num_envs)
        ]
        if self._shared_server:
            envelope = self.clients[0].request(
                {"protocol": "spark_sonic_policy_batch_v1", "requests": requests}
            )
            responses = [] if envelope is None else envelope.get("responses", [])
            if len(responses) != self.num_envs:
                responses = [None] * self.num_envs
        else:
            responses = [
                future.result()
                for future in (
                    self._pool.submit(client.request, request)
                    for client, request in zip(self.clients, requests)
                )
            ]
        current = arrays[0]
        if self._last_target is None:
            self._last_target = current.copy()
        target = self._last_target.copy()
        target[self._hold_current] = current[self._hold_current]
        self._hold_current[:] = False
        ok = np.zeros(self.num_envs, dtype=bool)
        for i, response in enumerate(responses):
            if response and response.get("ok", True) and "target_actuated_pos" in response:
                candidate = np.asarray(response["target_actuated_pos"], dtype=np.float32)
                if candidate.shape == (29,):
                    target[i] = candidate
                    ok[i] = True

        # Match the scalar adapter: Cartesian arm targets condition Sonic's
        # whole-body output; they are not hard-overwritten after inference.
        # This preserves the learned coordination among arms, waist, and gait.
        self._last_target = target.copy()
        info = {"sonic_ok": self.torch.as_tensor(ok, device=self.device)}
        # The scalar Isaac benchmark explicitly selects this gain profile for
        # Sonic. Use it for every row instead of tensor-only scaled server gains.
        info.update(
            motor_kps=self.torch.as_tensor(
                np.broadcast_to(WBT_MOTOR_KPS, target.shape).copy(),
                device=self.device,
                dtype=body_joint_pos.dtype,
            ),
            motor_kds=self.torch.as_tensor(
                np.broadcast_to(WBT_MOTOR_KDS, target.shape).copy(),
                device=self.device,
                dtype=body_joint_pos.dtype,
            ),
        )
        return self.torch.as_tensor(target, device=self.device, dtype=body_joint_pos.dtype), info
