"""Shared viewer configuration for simulator visualization backends."""

from __future__ import annotations

import numpy as np


DEFAULT_VIEWER_CONFIG = {
    "camera_lookat": (0.5, 0.0, 1.3),
    "camera_distance": 2.0,
    "camera_azimuth": 180.0,
    "camera_elevation": -25.0,
    "camera_vertical_fov": 45.0,
    "ground_style": "grid",
    "ground_color": (0.82, 0.82, 0.82),
    "grid_color": (0.42, 0.42, 0.42),
    "grid_spacing": 0.5,
    "grid_extent": 5.0,
    "grid_line_width": 0.008,
}


def normalize_viewer_config(config=None):
    """Return a validated copy of the backend-neutral viewer settings."""
    result = dict(DEFAULT_VIEWER_CONFIG)
    result.update(dict(config or {}))
    result["camera_lookat"] = tuple(
        float(value) for value in np.asarray(result["camera_lookat"]).reshape(3)
    )
    for name in (
        "camera_distance",
        "camera_azimuth",
        "camera_elevation",
        "camera_vertical_fov",
        "grid_spacing",
        "grid_extent",
        "grid_line_width",
    ):
        result[name] = float(result[name])
    for name in ("ground_color", "grid_color"):
        result[name] = tuple(float(value) for value in np.asarray(result[name]).reshape(3))
    if result["camera_distance"] <= 0.0:
        raise ValueError("camera_distance must be positive")
    if result["grid_spacing"] <= 0.0 or result["grid_extent"] <= 0.0:
        raise ValueError("grid_spacing and grid_extent must be positive")
    if result["ground_style"] not in {"grid", "plain"}:
        raise ValueError("ground_style must be 'grid' or 'plain'")
    return result


def camera_eye_target(config=None):
    """Convert MuJoCo's look-at/spherical camera convention to eye/target."""
    config = normalize_viewer_config(config)
    target = np.asarray(config["camera_lookat"], dtype=float)
    azimuth = np.deg2rad(config["camera_azimuth"])
    elevation = np.deg2rad(config["camera_elevation"])
    target_to_camera = -config["camera_distance"] * np.array(
        [
            np.cos(elevation) * np.cos(azimuth),
            np.cos(elevation) * np.sin(azimuth),
            np.sin(elevation),
        ],
        dtype=float,
    )
    return target + target_to_camera, target


def normalize_sensor_camera_config(camera_config=None, viewer_config=None):
    """Return one backend-neutral mapping of RGB-D sensor definitions.

    A robot may supply a link-mounted camera through ``body_name`` and local
    ``pos``/``xyaxes`` fields.  When it does not, SPARK creates an overview
    camera from the same spherical pose used by the interactive viewer.  This
    keeps camera capture available to every robot without placing
    robot-specific link names in the simulator backends.
    """

    viewer = normalize_viewer_config(viewer_config)
    if camera_config is None:
        camera_config = {
            "overview": {
                "type": "free",
                "frame_id": "spark_overview_camera_optical_frame",
                "lookat": viewer["camera_lookat"],
                "distance": viewer["camera_distance"],
                "azimuth": viewer["camera_azimuth"],
                "elevation": viewer["camera_elevation"],
                "fovy": viewer["camera_vertical_fov"],
                "publish_rgb": True,
                "publish_depth": True,
            }
        }

    if isinstance(camera_config, dict):
        if "name" in camera_config:
            items = [(str(camera_config["name"]), camera_config)]
        else:
            items = list(camera_config.items())
    else:
        items = [
            (str(item.get("name", f"camera_{index}")), item)
            for index, item in enumerate(camera_config)
        ]

    normalized = {}
    for index, (name, raw_config) in enumerate(items):
        config = dict(raw_config)
        name = str(config.pop("name", name or f"camera_{index}"))
        config.setdefault("type", "fixed" if config.get("body_name") else "free")
        config.setdefault("frame_id", f"{name}_optical_frame")
        config.setdefault("publish_rgb", True)
        config.setdefault("publish_depth", True)
        config.setdefault("fovy", 58.0)
        if config["type"] == "free":
            config.setdefault("lookat", viewer["camera_lookat"])
            config.setdefault("distance", viewer["camera_distance"])
            config.setdefault("azimuth", viewer["camera_azimuth"])
            config.setdefault("elevation", viewer["camera_elevation"])
        else:
            config.setdefault("mujoco_camera_name", f"spark_{name}_camera")
        normalized[name] = config
    return normalized


def sensor_camera_eye_target(config):
    """Convert one free sensor-camera definition to an eye/target pair."""

    sensor_view = normalize_viewer_config(
        {
            "camera_lookat": config.get("lookat", DEFAULT_VIEWER_CONFIG["camera_lookat"]),
            "camera_distance": config.get("distance", DEFAULT_VIEWER_CONFIG["camera_distance"]),
            "camera_azimuth": config.get("azimuth", DEFAULT_VIEWER_CONFIG["camera_azimuth"]),
            "camera_elevation": config.get("elevation", DEFAULT_VIEWER_CONFIG["camera_elevation"]),
            "camera_vertical_fov": config.get("fovy", DEFAULT_VIEWER_CONFIG["camera_vertical_fov"]),
        }
    )
    return camera_eye_target(sensor_view)
