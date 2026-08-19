from types import SimpleNamespace

import numpy as np

from spark_policy.safety.filtering.value_based import ValueBasedSafeAlgorithm
from spark_policy.safety.monitoring.collision.base import BasicCollisionSafetyIndex


class _ValueAlgorithmForTest(ValueBasedSafeAlgorithm):
    def safe_control(self, x, u_ref, agent_feedback, task_info, action_info):
        raise NotImplementedError


def _algorithm_with_mask(mask):
    algorithm = object.__new__(_ValueAlgorithmForTest)
    algorithm.safety_index = SimpleNamespace(phi_mask=np.asarray(mask, dtype=bool))
    return algorithm


def test_value_filter_treats_an_empty_constraint_set_as_safe():
    algorithm = _algorithm_with_mask([])

    assert not algorithm.has_active_constraints()
    assert not algorithm.has_active_violation(np.array([]))


def test_value_filter_ignores_inactive_positive_constraints():
    algorithm = _algorithm_with_mask([False, True])

    assert algorithm.has_active_constraints()
    assert not algorithm.has_active_violation(np.array([1.0, -0.1]))
    assert algorithm.has_active_violation(np.array([-1.0, 0.1]))


def test_pairwise_collision_info_preserves_rank_for_an_empty_volume_set():
    distance, velocity, normal, curvature = BasicCollisionSafetyIndex.compute_pairwise_info(
        None, [object()], []
    )

    assert distance.shape == (1, 0, 3)
    assert velocity.shape == (1, 0, 3)
    assert normal.shape == (1, 0, 3)
    assert curvature.shape == (1, 0, 3, 3)
