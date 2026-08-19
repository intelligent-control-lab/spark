"""Shared Isaac rendering-quality presets.

The presets describe presentation quality only. They never change physics,
control frequency, sensor resolution, or benchmark task behavior.
"""

from __future__ import annotations


ISAAC_RENDER_QUALITY_CHOICES = ("performance", "quality", "cinematic")

_PRESETS = {
    "performance": {
        "renderer": "RaytracedLighting",
        "anti_aliasing": 2,  # FXAA
        "width": 1440,
        "height": 900,
        "samples_per_pixel_per_frame": 16,
        "denoiser": True,
        "max_bounces": 2,
    },
    "quality": {
        "renderer": "RealTimePathTracing",
        "anti_aliasing": 3,  # DLSS
        "width": 1920,
        "height": 1080,
        "samples_per_pixel_per_frame": 32,
        "denoiser": True,
        "max_bounces": 4,
    },
    "cinematic": {
        "renderer": "PathTracing",
        "anti_aliasing": 4,  # RTXAA where applicable
        "width": 2560,
        "height": 1440,
        "samples_per_pixel_per_frame": 128,
        "denoiser": True,
        "max_bounces": 8,
    },
}


def isaac_render_preset(name: str = "performance", **overrides) -> dict:
    """Return a validated preset with optional non-``None`` overrides."""
    name = str(name).lower()
    if name not in _PRESETS:
        raise ValueError(
            f"Unknown Isaac render quality {name!r}; expected one of {ISAAC_RENDER_QUALITY_CHOICES}"
        )
    result = dict(_PRESETS[name])
    result.update({key: value for key, value in overrides.items() if value is not None})
    result["quality"] = name
    return result


def apply_isaac_render_preset(name: str) -> dict:
    """Apply a preset to an already-running Kit application.

    This is used by Isaac Lab's ``AppLauncher``, whose application is created
    before SPARK constructs the simulation agent.
    """
    preset = isaac_render_preset(name)
    import carb

    settings = carb.settings.get_settings()
    settings.set("/rtx/rendermode", preset["renderer"])
    settings.set("/app/renderer/resolution/width", int(preset["width"]))
    settings.set("/app/renderer/resolution/height", int(preset["height"]))
    settings.set("/rtx/pathtracing/spp", int(preset["samples_per_pixel_per_frame"]))
    settings.set("/rtx/pathtracing/optixDenoiser/enabled", bool(preset["denoiser"]))
    settings.set("/rtx/pathtracing/maxBounces", int(preset["max_bounces"]))
    if preset["anti_aliasing"] == 3:
        settings.set("/rtx/post/dlss/execMode", 2)  # Quality
    return preset
