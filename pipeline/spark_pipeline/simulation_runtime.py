"""Shared lifecycle for single-environment MuJoCo and Isaac pipelines."""

from __future__ import annotations

from collections.abc import Callable
import math
from typing import Any


ISAAC_INTERACTIVE_VIEWER_HZ = 10.0
ISAAC_INTERACTIVE_RGBD_HZ = 5.0


def configure_simulation_presentation(cfg, *, backend: str) -> int:
    """Choose presentation cadence independently of control cadence.

    MuJoCo's lightweight viewer can normally synchronize every control step.
    An Isaac RTX frame is substantially more expensive, especially with an
    RGB-D render product, so presentation is bounded while physics and policy
    updates retain the robot's declared control period.
    """

    backend = str(backend).lower()
    if getattr(cfg, "render_every", None) is not None:
        return max(1, int(cfg.render_every))
    if backend != "isaac":
        cfg.render_every = 1
        return cfg.render_every

    control_period = float(getattr(cfg.env.agent, "dt", 0.002)) * int(
        getattr(cfg.env.agent, "control_decimation", 1)
    )
    requested_hz = float(getattr(cfg.env.agent, "viewer_hz", ISAAC_INTERACTIVE_VIEWER_HZ))
    if bool(getattr(cfg.env.agent, "enable_camera", False)):
        requested_hz = min(requested_hz, ISAAC_INTERACTIVE_RGBD_HZ)
    if requested_hz <= 0.0:
        cfg.render_every = 1
    else:
        cfg.render_every = max(1, int(math.ceil(1.0 / (requested_hz * control_period))))
    return cfg.render_every


def run_simulation_pipeline(
    cfg,
    *,
    backend: str,
    pipeline_class: Callable[[Any], Any],
    save_path: str | None = None,
):
    """Construct, run, and close a simulator-backed pipeline.

    Isaac requires ``SimulationApp`` to exist before its agent is imported and
    constructed.  Keeping that ordering here prevents every robot example from
    implementing a subtly different Kit lifecycle.  The returned pipeline is
    closed but remains available to callers for metrics inspection.
    """

    backend = str(backend).lower()
    if backend not in {"mujoco", "isaac"}:
        raise ValueError("backend must be 'mujoco' or 'isaac'")

    simulation_app = None
    pipeline = None
    try:
        configure_simulation_presentation(cfg, backend=backend)
        if backend == "isaac":
            from isaacsim import SimulationApp

            enable_viewer = bool(cfg.env.agent.enable_viewer)
            enable_camera = bool(getattr(cfg.env.agent, "enable_camera", False))
            simulation_app = SimulationApp(
                {
                    "headless": not enable_viewer,
                    "hide_ui": not enable_viewer,
                    "multi_gpu": False,
                    # RTX camera render products still need Kit render updates
                    # in headless capture-only workflows.
                    "disable_viewport_updates": not (enable_viewer or enable_camera),
                }
            )
            # The same in-process viewport owns both the authoritative physics
            # state and the image shown to the user.
            cfg.env.agent.render = enable_viewer or enable_camera
            # BaseGoalPipeline prepares and flushes one complete frame after
            # each control step.  Rendering inside the Isaac physics adapter as
            # well would submit two RTX frames per control cycle.
            cfg.env.agent.render_on_step = False

        pipeline = pipeline_class(cfg)
        if simulation_app is not None:
            pipeline.env.agent.attach_simulation_app(simulation_app)
        pipeline.run(save_path=save_path)
        return pipeline
    finally:
        if pipeline is not None:
            pipeline.env.agent.close_viewer()
            if backend == "isaac" and hasattr(pipeline.env.agent, "close"):
                pipeline.env.agent.close()
        if simulation_app is not None:
            simulation_app.close(wait_for_replicator=False)
