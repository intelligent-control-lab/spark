"""Isaac viewport information window shared by scalar and tensor agents."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


ISAAC_SIMULATION_INFO_WINDOW_TITLE = "SPARK Simulation Information"
ISAAC_SIMULATION_INFO_FONT_SIZE = 20
ISAAC_SIMULATION_INFO_ROW_HEIGHT = 25
ISAAC_SIMULATION_INFO_LABEL_WIDTH = 165
ISAAC_SIMULATION_INFO_ROW_SPACING = 1
ISAAC_SIMULATION_INFO_COLUMN_SPACING = 6
ISAAC_SIMULATION_INFO_MARGIN = 6
ISAAC_SIMULATION_INFO_WINDOW_WIDTH = 700
ISAAC_SIMULATION_INFO_WINDOW_HEIGHT = 280
# Approximate title-bar and frame chrome that is not part of the table layout.
ISAAC_SIMULATION_INFO_WINDOW_CHROME_HEIGHT = 34


def isaac_simulation_info(agent) -> dict[str, str]:
    """Build dynamics metadata without importing the optional Isaac UI."""

    robot_cfg = agent.robot_cfg
    dynamics_model = getattr(agent, "dynamics_model", None)
    variant = getattr(
        dynamics_model,
        "variant",
        getattr(robot_cfg, "dynamics_variant", type(robot_cfg).__name__),
    )
    order = int(
        getattr(
            dynamics_model,
            "order",
            getattr(robot_cfg, "dynamics_order", 1),
        )
    )
    state_dim = int(
        getattr(
            dynamics_model,
            "state_dim",
            getattr(robot_cfg, "num_state", len(robot_cfg.DoFs)),
        )
    )
    control_dim = int(
        getattr(
            dynamics_model,
            "control_dim",
            len(robot_cfg.Control),
        )
    )
    backend = str(getattr(agent, "dynamics_backend", "simulator")).lower()
    if backend == "simulator":
        propagation = "Isaac PhysX"
        physical_self_contact = (
            "enabled" if bool(getattr(agent, "allow_self_collision", False)) else "disabled"
        )
    else:
        executor = getattr(agent, "dynamics_executor", None)
        integrator = getattr(executor, "integrator", None) or getattr(
            agent, "model_integrator", None
        )
        propagation = f"SPARK model ({integrator or 'Euler'})"
        physical_self_contact = "inactive (model propagation)"

    dt = float(getattr(agent, "dt", 0.0))
    decimation = int(getattr(agent, "control_decimation", 1))
    control_period = float(getattr(agent, "control_period", dt * decimation))

    return {
        "Robot config": type(robot_cfg).__name__,
        "Dynamics model": str(variant),
        "Dynamics order": str(order),
        "Dimensions": f"state={state_dim}, control={control_dim}",
        "Propagation": propagation,
        "PhysX self-contact": physical_self_contact,
        "Environments": str(int(getattr(agent, "num_envs", 1))),
        "Device": str(getattr(agent, "device", "unknown")),
        "Timing": (f"period={control_period:g} s, time={float(getattr(agent, 'time', 0.0)):.3f} s"),
    }


class IsaacSimulationInfoWindow:
    """Lazy ``omni.ui`` window with a caption and two-column metadata table."""

    def __init__(self, *, ui_module=None) -> None:
        self._ui_module = ui_module
        self._window = None
        self._value_labels: dict[str, Any] = {}
        self._unavailable = False

    def _ui(self):
        if self._ui_module is None:
            import omni.ui as ui

            self._ui_module = ui
        return self._ui_module

    def _build(self, information: Mapping[str, str]) -> None:
        ui = self._ui()
        row_count = len(information)
        content_height = (
            2 * ISAAC_SIMULATION_INFO_MARGIN
            + row_count * ISAAC_SIMULATION_INFO_ROW_HEIGHT
            + max(0, row_count - 1) * ISAAC_SIMULATION_INFO_ROW_SPACING
            + ISAAC_SIMULATION_INFO_WINDOW_CHROME_HEIGHT
        )
        self._window = ui.Window(
            ISAAC_SIMULATION_INFO_WINDOW_TITLE,
            width=ISAAC_SIMULATION_INFO_WINDOW_WIDTH,
            height=max(ISAAC_SIMULATION_INFO_WINDOW_HEIGHT, content_height),
            visible=True,
        )
        self._window.position_x = 12
        self._window.position_y = 48
        with self._window.frame:
            with ui.VStack(
                spacing=ISAAC_SIMULATION_INFO_ROW_SPACING,
                style={"margin": ISAAC_SIMULATION_INFO_MARGIN},
            ):
                for label, value in information.items():
                    with ui.HStack(
                        height=ISAAC_SIMULATION_INFO_ROW_HEIGHT,
                        spacing=ISAAC_SIMULATION_INFO_COLUMN_SPACING,
                    ):
                        ui.Label(
                            str(label),
                            width=ISAAC_SIMULATION_INFO_LABEL_WIDTH,
                            style={"font_size": ISAAC_SIMULATION_INFO_FONT_SIZE},
                        )
                        self._value_labels[label] = ui.Label(
                            str(value),
                            style={"font_size": ISAAC_SIMULATION_INFO_FONT_SIZE},
                        )

    def update(self, information: Mapping[str, Any]) -> bool:
        """Create or refresh the window; return false when UI is unavailable."""

        if self._unavailable:
            return False
        normalized = {str(label): str(value) for label, value in information.items()}
        try:
            if self._window is None or tuple(self._value_labels) != tuple(normalized):
                self.close()
                self._build(normalized)
            else:
                for label, value in normalized.items():
                    self._value_labels[label].text = value
        except (AttributeError, ImportError, RuntimeError):
            # Minimal/headless Kit experiences may not provide a workspace UI.
            # Simulation and rendering must remain usable in that case.
            self.close()
            self._unavailable = True
            return False
        return True

    def close(self) -> None:
        if self._window is not None:
            try:
                self._window.visible = False
            except (AttributeError, RuntimeError):
                pass
        self._window = None
        self._value_labels.clear()


class IsaacViewerInfoMixin:
    """Agent methods for the common Isaac dynamics information window."""

    def _initialize_viewer_simulation_info(self, **kwargs) -> None:
        self.viewer_show_simulation_info = bool(kwargs.get("viewer_show_simulation_info", False))
        self._viewer_simulation_info: dict[str, str] = {}
        self._simulation_info_window = None

    def get_simulation_info(self) -> dict[str, str]:
        information = isaac_simulation_info(self)
        information.update(self._viewer_simulation_info)
        return information

    def set_viewer_simulation_info(
        self,
        information: Mapping[str, Any] | None = None,
        *,
        replace: bool = False,
    ) -> None:
        if information is None:
            information = {}
        if not isinstance(information, Mapping):
            raise TypeError("information must be a mapping of viewer labels to values")
        if replace:
            self._viewer_simulation_info.clear()
        self._viewer_simulation_info.update(
            {str(label): str(value) for label, value in information.items()}
        )

    def _update_simulation_info_overlay(self) -> bool:
        if not self.viewer_show_simulation_info or not self.render_enabled:
            return False
        if self._simulation_info_window is None:
            self._simulation_info_window = IsaacSimulationInfoWindow()
        return self._simulation_info_window.update(self.get_simulation_info())

    def _close_simulation_info_overlay(self) -> None:
        if self._simulation_info_window is not None:
            self._simulation_info_window.close()
        self._simulation_info_window = None
