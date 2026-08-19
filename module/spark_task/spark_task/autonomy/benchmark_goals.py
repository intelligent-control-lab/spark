"""Shared scalar and vector benchmark goal semantics."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib

import numpy as np


DEFAULT_LEFT_ARM_GOAL_RANGE = ((0.1, 0.4), (0.1, 0.4), (0.0, 0.3))
DEFAULT_RIGHT_ARM_GOAL_RANGE = ((0.1, 0.4), (-0.4, -0.1), (0.0, 0.3))


BENCHMARK_ENVIRONMENT_SEED_STRIDE = 100_003


@dataclass(frozen=True)
class BenchmarkScenario:
    """Backend-neutral reset sample in one simulator-local world frame.

    Arm targets are expressed in the robot root frame. The base target and
    obstacle centers are expressed relative to the simulator environment
    origin. Keeping this data as NumPy values makes a benchmark seed
    independent of Torch device and simulator backend.
    """

    seed: int
    right_arm_goal: np.ndarray | None
    left_arm_goal: np.ndarray | None
    base_goal: np.ndarray | None
    obstacle_positions: np.ndarray


def benchmark_scenario_seed(
    base_seed: int,
    *,
    environment_index: int = 0,
    episode_index: int = 0,
) -> int:
    """Return the stable seed assigned to one environment episode.

    ``+1`` preserves the public scalar task convention: a configured seed of
    zero produces seed one on the first reset. The large environment stride
    prevents another environment's episode stream from overlapping it.
    """

    environment_index = int(environment_index)
    episode_index = int(episode_index)
    if environment_index < 0 or episode_index < 0:
        raise ValueError("environment and episode indices cannot be negative")
    return (
        int(base_seed) + 1 + BENCHMARK_ENVIRONMENT_SEED_STRIDE * environment_index + episode_index
    )


def benchmark_scenario_fingerprint(scenario: BenchmarkScenario) -> str:
    """Return a short stable identifier for cross-backend reset diagnostics."""

    digest = hashlib.sha256()
    digest.update(np.asarray([scenario.seed], dtype=np.int64).tobytes())
    for value in (
        scenario.right_arm_goal,
        scenario.left_arm_goal,
        scenario.base_goal,
        scenario.obstacle_positions,
    ):
        if value is None:
            digest.update(b"none")
        else:
            digest.update(np.asarray(value, dtype=np.float64).tobytes())
    return digest.hexdigest()[:12]


def rotation_angle_error(orientation, goal_orientation):
    """Return the shortest SO(3) angular distance in radians."""

    orientation = np.asarray(orientation, dtype=float)
    goal_orientation = np.asarray(goal_orientation, dtype=float)
    relative = np.matmul(np.swapaxes(goal_orientation, -1, -2), orientation)
    cosine = np.clip((np.trace(relative, axis1=-2, axis2=-1) - 1.0) * 0.5, -1.0, 1.0)
    return np.arccos(cosine)


def sample_bounded_position(random_state, bounds) -> np.ndarray:
    """Sample XYZ from the same axis-aligned representation used by tasks."""
    bounds = np.asarray(bounds, dtype=float).reshape(3, 2)
    return np.asarray([random_state.uniform(low, high) for low, high in bounds], dtype=float)


def arm_goal_pair_is_separated(right_goal, left_goal, minimum_distance: float) -> bool:
    """Return whether two end-effector targets satisfy their center keep-out."""

    minimum_distance = float(minimum_distance)
    if minimum_distance < 0.0:
        raise ValueError("arm goal pair minimum distance cannot be negative")
    right_goal = np.asarray(right_goal, dtype=float).reshape(3)
    left_goal = np.asarray(left_goal, dtype=float).reshape(3)
    return bool(np.linalg.norm(right_goal - left_goal) >= minimum_distance)


def sample_benchmark_scenario(
    *,
    seed: int,
    root_position=(0.0, 0.0, 0.793),
    root_yaw: float = 0.0,
    arm_goal_enabled: bool = True,
    dual_arm: bool = True,
    right_arm_goal_range=DEFAULT_RIGHT_ARM_GOAL_RANGE,
    left_arm_goal_range=DEFAULT_LEFT_ARM_GOAL_RANGE,
    right_arm_start=None,
    left_arm_start=None,
    arm_goal_minimum_distance: float = 0.0,
    arm_goal_pair_keepout: float = 0.0,
    base_goal_enabled: bool = True,
    base_goal_range=((0.0, 0.0), (0.0, 0.0), (0.793, 0.793)),
    base_goal_rot_range=(0.0, 0.0),
    base_goal_relative_to_current: bool = False,
    base_goal_workspace_range=None,
    base_goal_minimum_distance: float = 0.0,
    num_obstacles: int = 0,
    obstacle_range=((-2.0, 2.0), (-2.0, 2.0), (0.8, 1.1)),
    obstacle_radius: float = 0.05,
    obstacle_keepout: float = 0.05,
    obstacle_robot_keepaway: float = 0.0,
    obstacle_goal_keepaway: float = 0.0,
    arm_goal_radius: float = 0.05,
    base_goal_radius: float = 0.1,
    robot_collision_centers=None,
    robot_collision_radii=None,
    max_attempts: int = 100_000,
) -> BenchmarkScenario:
    """Sample one complete static benchmark scene with NumPy only.

    Sampling goals before obstacles lets both backends enforce exactly the
    same goal, robot, and obstacle clearances. All values are simulator-local;
    callers add an Isaac clone origin only when writing world-space tensors.
    """

    random_state = np.random.RandomState(int(seed))
    root_position = np.asarray(root_position, dtype=float).reshape(3)
    cosine = float(np.cos(root_yaw))
    sine = float(np.sin(root_yaw))

    def root_to_world(point):
        point = np.asarray(point, dtype=float).reshape(3)
        return root_position + np.array(
            [
                cosine * point[0] - sine * point[1],
                sine * point[0] + cosine * point[1],
                point[2],
            ]
        )

    right_goal = None
    left_goal = None
    if arm_goal_enabled:
        for _ in range(max_attempts):
            candidate_right = sample_bounded_position(random_state, right_arm_goal_range)
            candidate_left = (
                sample_bounded_position(random_state, left_arm_goal_range) if dual_arm else None
            )
            right_world = root_to_world(candidate_right)
            valid = right_arm_start is None or (
                np.linalg.norm(right_world - np.asarray(right_arm_start, dtype=float).reshape(3))
                >= float(arm_goal_minimum_distance)
            )
            if dual_arm:
                left_world = root_to_world(candidate_left)
                valid &= left_arm_start is None or (
                    np.linalg.norm(left_world - np.asarray(left_arm_start, dtype=float).reshape(3))
                    >= float(arm_goal_minimum_distance)
                )
                valid &= arm_goal_pair_is_separated(right_world, left_world, arm_goal_pair_keepout)
            if valid:
                right_goal = candidate_right
                left_goal = candidate_left
                break
        else:
            raise ValueError("Could not sample separated arm benchmark goals")

    base_goal = None
    if base_goal_enabled:
        goal_bounds = np.asarray(base_goal_range, dtype=float).reshape(3, 2)
        workspace = (
            None
            if base_goal_workspace_range is None
            else np.asarray(base_goal_workspace_range, dtype=float).reshape(2, 2)
        )
        for _ in range(max_attempts):
            candidate = sample_bounded_position(random_state, goal_bounds)
            if base_goal_relative_to_current:
                candidate[:2] += root_position[:2]
            inside_workspace = workspace is None or bool(
                np.all(candidate[:2] >= workspace[:, 0])
                and np.all(candidate[:2] <= workspace[:, 1])
            )
            if inside_workspace and (
                np.linalg.norm(candidate[:2] - root_position[:2])
                >= float(base_goal_minimum_distance)
            ):
                yaw = random_state.uniform(*np.asarray(base_goal_rot_range, dtype=float))
                base_goal = np.concatenate((candidate, [yaw]))
                break
        else:
            raise ValueError("Could not sample a valid base benchmark goal")

    collision_centers = np.empty((0, 3), dtype=float)
    collision_radii = np.empty(0, dtype=float)
    if robot_collision_centers is not None:
        local_centers = np.asarray(robot_collision_centers, dtype=float).reshape(-1, 3)
        collision_centers = np.asarray([root_to_world(center) for center in local_centers])
        collision_radii = np.asarray(robot_collision_radii, dtype=float).reshape(-1)
        if collision_centers.shape[0] != collision_radii.shape[0]:
            raise ValueError("robot collision centers and radii must have equal length")

    protected_goal_centers = []
    protected_goal_radii = []
    if right_goal is not None:
        protected_goal_centers.append(root_to_world(right_goal))
        protected_goal_radii.append(float(arm_goal_radius))
    if left_goal is not None:
        protected_goal_centers.append(root_to_world(left_goal))
        protected_goal_radii.append(float(arm_goal_radius))
    if base_goal is not None:
        protected_goal_centers.append(base_goal[:3])
        protected_goal_radii.append(float(base_goal_radius))
    protected_goal_centers = np.asarray(protected_goal_centers, dtype=float).reshape(-1, 3)
    protected_goal_radii = np.asarray(protected_goal_radii, dtype=float)

    obstacles = []
    obstacle_bounds = np.asarray(obstacle_range, dtype=float).reshape(3, 2)
    for _ in range(max_attempts):
        if len(obstacles) >= int(num_obstacles):
            break
        candidate = sample_bounded_position(random_state, obstacle_bounds)
        if collision_centers.size and np.any(
            np.linalg.norm(collision_centers - candidate, axis=1)
            < collision_radii + float(obstacle_radius) + float(obstacle_robot_keepaway)
        ):
            continue
        if obstacles and np.any(
            np.linalg.norm(np.asarray(obstacles) - candidate, axis=1)
            < 2.0 * float(obstacle_radius) + float(obstacle_keepout)
        ):
            continue
        if protected_goal_centers.size and np.any(
            np.linalg.norm(protected_goal_centers - candidate, axis=1)
            < protected_goal_radii + float(obstacle_radius) + float(obstacle_goal_keepaway)
        ):
            continue
        obstacles.append(candidate)
    if len(obstacles) != int(num_obstacles):
        raise ValueError("Could not sample a valid benchmark obstacle layout")

    return BenchmarkScenario(
        seed=int(seed),
        right_arm_goal=None if right_goal is None else np.asarray(right_goal),
        left_arm_goal=None if left_goal is None else np.asarray(left_goal),
        base_goal=None if base_goal is None else np.asarray(base_goal),
        obstacle_positions=np.asarray(obstacles, dtype=float).reshape(-1, 3),
    )


def evaluate_goal_completion(
    *,
    left_position,
    right_position,
    left_goal,
    right_goal,
    base_position=None,
    base_goal=None,
    arm_tolerance: float = 0.05,
    arm_orientation_tolerance: float = 0.1,
    base_tolerance: float = 0.15,
    arm_enabled: bool = True,
    base_enabled: bool = True,
    dual_arm: bool = True,
    arm_position_only: bool = True,
    left_orientation=None,
    right_orientation=None,
    left_goal_orientation=None,
    right_goal_orientation=None,
):
    """Evaluate scalar or batched benchmark completion using Cartesian errors."""
    right_error = np.linalg.norm(np.asarray(right_position) - np.asarray(right_goal), axis=-1)
    if arm_position_only:
        right_orientation_error = np.zeros_like(right_error)
    else:
        if right_orientation is None or right_goal_orientation is None:
            raise ValueError("right arm orientations are required for pose completion")
        right_orientation_error = rotation_angle_error(right_orientation, right_goal_orientation)
    if dual_arm:
        left_error = np.linalg.norm(np.asarray(left_position) - np.asarray(left_goal), axis=-1)
        if arm_position_only:
            left_orientation_error = np.zeros_like(left_error)
        else:
            if left_orientation is None or left_goal_orientation is None:
                raise ValueError("left arm orientations are required for dual-arm pose completion")
            left_orientation_error = rotation_angle_error(left_orientation, left_goal_orientation)
        arm_reached = (
            (left_error < arm_tolerance)
            & (right_error < arm_tolerance)
            & (left_orientation_error < arm_orientation_tolerance)
            & (right_orientation_error < arm_orientation_tolerance)
        )
    else:
        # Preserve a shape-compatible metric while making completion depend
        # only on the end effector that exists in a single-arm embodiment.
        left_error = np.zeros_like(right_error)
        left_orientation_error = np.zeros_like(right_orientation_error)
        arm_reached = (right_error < arm_tolerance) & (
            right_orientation_error < arm_orientation_tolerance
        )
    if base_position is None or base_goal is None:
        base_error = np.zeros_like(left_error)
    else:
        base_error = np.linalg.norm(
            np.asarray(base_position)[..., :2] - np.asarray(base_goal)[..., :2],
            axis=-1,
        )
    base_reached = base_error < base_tolerance
    if arm_enabled and base_enabled:
        reached = arm_reached & base_reached
    elif arm_enabled:
        reached = arm_reached
    elif base_enabled:
        reached = base_reached
    else:
        reached = np.ones_like(arm_reached, dtype=bool)
    return {
        "reached": reached,
        "arm_reached": arm_reached,
        "base_reached": base_reached,
        "left_error": left_error,
        "right_error": right_error,
        "left_orientation_error": left_orientation_error,
        "right_orientation_error": right_orientation_error,
        "base_error": base_error,
    }
