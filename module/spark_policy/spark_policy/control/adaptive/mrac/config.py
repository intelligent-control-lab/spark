from dataclasses import dataclass

import numpy as np

from spark_policy.core import PolicyConfig


@dataclass
class MRACPolicyConfig(PolicyConfig):
    """Runtime configuration for model-reference adaptive control.

    The plant dynamics are deliberately absent: those come from the selected
    robot configuration.  These matrices describe the controller's reference
    model and its online parameter estimates.
    """

    reference_model: np.ndarray | None = None
    initial_state_matrix: np.ndarray | None = None
    initial_control_matrix: np.ndarray | None = None
    initial_information: np.ndarray | None = None
    record_history: bool = True
