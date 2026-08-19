import numpy as np

from spark_task.utils.task_objects import TaskObject3D


def test_brownian_object_reflects_instead_of_sticking_to_boundary_plane():
    obstacle = TaskObject3D(
        velocity=1.0,
        bound=[(-1.0, 1.0)] * 3,
        direction=np.array([0.0, 1.0, 0.0]),
        keep_direction_step=100,
        smooth_weight=1.0,
        dt=0.1,
        _seed=0,
    )
    obstacle.frame[1, 3] = 0.99
    obstacle.last_direction = np.array([0.0, 0.1, 0.0])
    obstacle.step_counter = 1

    obstacle.move("Brownian")

    assert obstacle.frame[1, 3] < 1.0
    assert obstacle.last_direction[1] < 0.0


def test_velocity_object_reflects_at_bounds():
    obstacle = TaskObject3D(
        velocity=1.0,
        bound=[(-1.0, 1.0)] * 3,
        direction=np.array([0.0, 1.0, 0.0]),
        dt=0.1,
    )
    obstacle.frame[1, 3] = 0.99

    obstacle.move("Velocity")

    assert np.isclose(obstacle.frame[1, 3], 0.91)
    assert obstacle.last_direction[1] < 0.0


def test_zero_velocity_brownian_goal_does_not_inherit_direction_as_displacement():
    goal = TaskObject3D(
        velocity=0.0,
        bound=[(-1.0, 1.0)] * 3,
        direction=np.array([1.0, 0.0, 0.0]),
        keep_direction_step=500,
        smooth_weight=0.8,
        dt=0.02,
    )
    initial = goal.frame.copy()

    for _ in range(20):
        goal.move("Brownian")

    assert np.allclose(goal.frame, initial)
