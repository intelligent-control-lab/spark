"""Shared presentation options for robot teleoperation launchers."""

from __future__ import annotations

import argparse


def configure_teleop_camera(agent_config, kwargs) -> None:
    """Apply the common camera CLI contract to an agent configuration."""

    enabled = bool(kwargs.get("enable_camera", False))
    display = kwargs.get("camera_display")
    agent_config.enable_camera = enabled
    agent_config.camera_width = int(kwargs.get("camera_width") or 640)
    agent_config.camera_height = int(kwargs.get("camera_height") or 480)
    agent_config.camera_rate_hz = float(kwargs.get("camera_rate_hz") or 30.0)
    # Capturing without displaying remains available through
    # ``--no-camera-display``. Interactive launchers should show RGB and depth
    # as soon as users request a camera, matching the original MuJoCo UX.
    agent_config.camera_display = enabled if display is None else bool(display)


def configure_teleop_viewer(agent_config, kwargs) -> None:
    """Apply backend-neutral interactive-viewer options."""

    agent_config.viewer_show_simulation_info = bool(
        kwargs.get("viewer_show_simulation_info", False)
    )


def add_teleop_camera_arguments(parser: argparse.ArgumentParser) -> None:
    """Add the common backend-neutral RGB-D camera arguments."""

    parser.add_argument(
        "--enable-camera",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Capture an RGB-D view; display RGB and depth windows by default.",
    )
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-rate-hz", type=float, default=30.0)
    parser.add_argument(
        "--camera-display",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Show separate RGB and depth windows (defaults to --enable-camera).",
    )


def add_teleop_viewer_arguments(parser: argparse.ArgumentParser) -> None:
    """Add common backend-neutral interactive-viewer arguments."""

    parser.add_argument(
        "--show-simulation-info",
        dest="viewer_show_simulation_info",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Show the dynamics information panel in the MuJoCo or Isaac viewer.",
    )
