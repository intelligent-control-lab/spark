from abc import abstractmethod
import numpy as np

from spark_policy.safety.filtering.base import BaseSafeAlgorithm
from spark_policy.safety.monitoring.base import BaseSafetyIndex


class ValueBasedSafeAlgorithm(BaseSafeAlgorithm):
    def __init__(self, safety_index: BaseSafetyIndex, **kwargs):

        super().__init__(**kwargs)

        self.safety_index = safety_index

        self.control_max = np.array(
            [self.robot_cfg.ControlLimit[i] for i in self.robot_cfg.Control]
        )
        self.control_min = np.array(
            [-self.robot_cfg.ControlLimit[i] for i in self.robot_cfg.Control]
        )

    def active_constraint_mask(self) -> np.ndarray:
        """Return the safety-index mask as a flat boolean array."""
        return np.asarray(self.safety_index.phi_mask, dtype=bool).reshape(-1)

    def has_active_constraints(self) -> bool:
        """Whether the safety index currently exposes any active constraint."""
        return bool(np.any(self.active_constraint_mask()))

    def has_active_violation(self, phi: np.ndarray) -> bool:
        """Whether any active constraint has a positive safety-index value.

        Collision indices legitimately contain no active constraints when a scene
        has no obstacles and self-collision monitoring is disabled.  Treat that
        scene as safe instead of reducing over an empty array.
        """
        phi = np.asarray(phi, dtype=float).reshape(-1)
        active_mask = self.active_constraint_mask()
        if phi.shape != active_mask.shape:
            raise ValueError(
                "Safety-index values and active mask must have matching shapes; "
                f"got {phi.shape} and {active_mask.shape}."
            )
        return bool(np.any(active_mask) and np.any(phi[active_mask] > 0.0))

    # @abstractmethod
    # def kappa(self, x):
    #     '''
    #         K-class function for value based safe algorithms

    #         control constraint: phi_dot <= -k(phi)

    #         x: R^n
    #         k: R^n -> R^n
    #     '''
    #     pass

    @abstractmethod
    def safe_control(
        self,
        x: np.ndarray,
        u_ref: np.ndarray,
        agent_feedback: dict,
        task_info: dict,
        action_info: dict,
    ):

        pass
