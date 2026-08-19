"""Compatibility entry point for explicit benchmark task selection.

New code should use :mod:`benchmark_test_cases` directly.  This module keeps
the historical function name, but it intentionally applies task settings only;
robot and agent selection belongs to each benchmark runner.
"""

from __future__ import annotations

from .benchmark_test_cases import apply_benchmark_test_case


# Historical robot-prefixed names are retained here only as migration backup.
# New cases use robot-independent goal roles and v0/v1/v2 density levels.
LEGACY_TASK_CASE_LIST = (
    "UnitreeG1FixedBase_D1_AG_SO_v0",
    "UnitreeG1FixedBase_D1_AG_SO_v1",
    "UnitreeG1FixedBase_D1_AG_DO_v0",
    "UnitreeG1FixedBase_D1_AG_DO_v1",
    "UnitreeG1FixedBase_D2_AG_SO_v0",
    "UnitreeG1FixedBase_D2_AG_SO_v1",
    "UnitreeG1FixedBase_D2_AG_DO_v0",
    "UnitreeG1FixedBase_D2_AG_DO_v1",
    "UnitreeG1MobileBase_D1_WG_SO_v0",
    "UnitreeG1MobileBase_D1_WG_SO_v1",
    "UnitreeG1MobileBase_D1_WG_DO_v0",
    "UnitreeG1MobileBase_D1_WG_DO_v1",
    "UnitreeG1MobileBase_D2_WG_SO_v0",
    "UnitreeG1MobileBase_D2_WG_SO_v1",
    "UnitreeG1MobileBase_D2_WG_DO_v0",
    "UnitreeG1MobileBase_D2_WG_DO_v1",
    "UnitreeG1WholeBody_D1_WG_SO_v0",
    "UnitreeG1WholeBody_D1_WG_SO_v1",
    "UnitreeG1WholeBody_D1_WG_DO_v0",
    "UnitreeG1WholeBody_D1_WG_DO_v1",
    "UnitreeG1WholeBody_D2_WG_SO_v0",
    "UnitreeG1WholeBody_D2_WG_SO_v1",
    "UnitreeG1WholeBody_D2_WG_DO_v0",
    "UnitreeG1WholeBody_D2_WG_DO_v1",
    "UnitreeG1SportMode_D1_WG_SO_v1",
)


def generate_benchmark_test_case(cfg, test_case="arm_goal_static_v0"):
    """Apply a named environment/task composition to ``cfg.env.task``.

    The function never mutates ``cfg.robot`` or ``cfg.env.agent``.
    """
    return apply_benchmark_test_case(cfg, test_case)
