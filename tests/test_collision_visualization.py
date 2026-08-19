import numpy as np

from spark_agent.simulation.isaac.isaac_agent import (
    ISAAC_COLLISION_VOLUME_OPACITY_FLOOR,
    ISAAC_COLLISION_VOLUME_OPACITY_SCALE,
    ISAAC_GOAL_OPACITY,
    _isaac_sphere_rgba,
    _quantize_debug_rgba,
)
from spark_pipeline.visualization import (
    _has_active_collision,
    closest_collision_volume_distances,
    collision_volume_distance_color,
)
from spark_utils import VizColor, collision_volume_distance_color as shared_distance_color


def test_physical_overlap_is_red_even_when_pair_is_not_a_safety_constraint():
    distances = np.array([[0.10], [-0.01]])
    safety_mask = np.array([[True], [False]])

    assert _has_active_collision(distances, safety_mask, obstacle_id=0)


def test_positive_clearance_is_not_a_collision():
    distances = np.array([[0.10], [0.01]])

    assert not _has_active_collision(distances, None, obstacle_id=0)


def test_physics_contact_tolerance_is_rendered_as_collision():
    distances = np.array([[5.0e-4]])

    assert _has_active_collision(distances, None, obstacle_id=0)


def test_collision_volume_distance_color_is_light_far_blue_near_and_red_at_contact():
    assert collision_volume_distance_color is shared_distance_color
    far = collision_volume_distance_color(np.inf, minimum_distance=0.05)
    approaching = collision_volume_distance_color(0.15, minimum_distance=0.05)
    near = collision_volume_distance_color(0.05, minimum_distance=0.05)
    contact = collision_volume_distance_color(0.0, minimum_distance=0.05)

    assert 0.0 < far[3] < 0.1
    assert min(far[:3]) >= 0.9
    assert far[3] < approaching[3] < near[3]
    np.testing.assert_allclose(far, VizColor.collision_volume_far)
    np.testing.assert_allclose(near, VizColor.collision_volume_near)
    np.testing.assert_allclose(contact, VizColor.collision)


def test_isaac_opacity_scale_keeps_far_volumes_faint_but_nonzero():
    far = collision_volume_distance_color(
        np.inf,
        minimum_distance=0.05,
        opacity_scale=1.0,
    )
    near = collision_volume_distance_color(
        0.05,
        minimum_distance=0.05,
        opacity_scale=1.0,
    )

    assert 0.0 < far[3] < 0.03
    assert far[3] < near[3] <= VizColor.collision_volume_near[3]
    np.testing.assert_allclose(far[:3], VizColor.collision_volume_far[:3])


def test_isaac_alpha_palette_preserves_faint_nonzero_collision_overlay():
    assert ISAAC_COLLISION_VOLUME_OPACITY_SCALE == 1.0
    assert ISAAC_COLLISION_VOLUME_OPACITY_FLOOR == 0.05
    far = collision_volume_distance_color(
        np.inf,
        minimum_distance=0.05,
        opacity_scale=ISAAC_COLLISION_VOLUME_OPACITY_SCALE,
        opacity_floor=ISAAC_COLLISION_VOLUME_OPACITY_FLOOR,
    )
    key, quantized = _quantize_debug_rgba(far)

    assert key[3] == 13
    assert quantized[3] == 13 / 255


def test_isaac_goal_spheres_are_more_opaque_without_changing_other_spheres():
    goal_rgb, goal_alpha = _isaac_sphere_rgba(VizColor.goal)
    obstacle_rgb, obstacle_alpha = _isaac_sphere_rgba(VizColor.obstacle_task)

    np.testing.assert_allclose(goal_rgb, VizColor.goal[:3])
    assert goal_alpha == ISAAC_GOAL_OPACITY == 0.65
    np.testing.assert_allclose(obstacle_rgb, VizColor.obstacle_task[:3])
    assert obstacle_alpha == VizColor.obstacle_task[3]


def test_task_and_debug_obstacles_share_the_orange_palette():
    collision = np.asarray(VizColor.collision_volume_far)
    task_obstacle = np.asarray(VizColor.obstacle_task)
    debug_obstacle = np.asarray(VizColor.obstacle_debug)

    np.testing.assert_allclose(task_obstacle, debug_obstacle)
    assert task_obstacle[0] > task_obstacle[1] > task_obstacle[2]
    assert task_obstacle[3] > collision[3]


def test_closest_collision_volume_distance_is_computed_per_robot_volume():
    distances = np.array([[0.4, 0.2], [np.inf, -0.01], [np.nan, np.nan]])
    closest = closest_collision_volume_distances(distances, 3)

    np.testing.assert_allclose(closest[:2], [0.2, -0.01])
    assert np.isinf(closest[2])
