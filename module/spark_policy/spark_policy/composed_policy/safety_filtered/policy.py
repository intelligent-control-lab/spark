"""Self-contained nominal-policy plus safety-filter composition."""

from __future__ import annotations

import numpy as np
from spark_utils import initialize_class

from spark_policy.core import Policy


class SafetyFilteredPolicy(Policy):
    """Ground and execute a nominal policy, safety monitor, and safety filter."""

    def __init__(
        self,
        nominal_controller=None,
        monitor=None,
        safety_filter=None,
        *,
        policy=None,
        safe_controller=None,
        robot_cfg=None,
        robot_kinematics=None,
        clip_action_to_control_limits=True,
        **kwargs,
    ):
        del kwargs
        nominal_controller = nominal_controller if nominal_controller is not None else policy
        if safety_filter is None and callable(getattr(monitor, "safe_control", None)):
            safety_filter = monitor
            monitor = getattr(safety_filter, "safety_index", None)
        if safe_controller is not None:
            get = (
                safe_controller.get
                if isinstance(safe_controller, dict)
                else lambda name, default=None: getattr(safe_controller, name, default)
            )
            monitor = monitor if monitor is not None else get("safety_index")
            safety_filter = safety_filter if safety_filter is not None else get("safe_algo")

        self.nominal_policy = self._resolve(
            nominal_controller,
            robot_cfg=robot_cfg,
            robot_kinematics=robot_kinematics,
        )
        self.safety_monitor = (
            None if monitor is None else self._resolve(monitor, robot_kinematics=robot_kinematics)
        )
        self.safety_filter = self._resolve(
            safety_filter,
            safety_index=self.safety_monitor,
            safety_monitor=self.safety_monitor,
            robot_cfg=robot_cfg,
            robot_kinematics=robot_kinematics,
        )
        if self.safety_monitor is not None and not hasattr(self.safety_filter, "safety_index"):
            self.safety_filter.safety_index = self.safety_monitor

        self.robot_cfg = robot_cfg or getattr(self.nominal_policy, "robot_cfg", None)
        self.clip_action_to_control_limits = bool(clip_action_to_control_limits)

    @staticmethod
    def _resolve(component, **kwargs):
        if component is None:
            raise ValueError("Every safety-filtered component must be configured")
        if any(
            callable(getattr(component, method, None)) for method in ("act", "safe_control", "phi")
        ):
            return component
        return initialize_class(component, **kwargs)

    @property
    def components(self):
        return {
            "nominal": self.nominal_policy,
            "monitor": self.safety_monitor,
            "filter": self.safety_filter,
        }

    def observability_context(self) -> dict:
        """Expose optional diagnostics without leaking composition internals."""
        return {
            "constraint_monitor": self.safety_monitor,
            "environment_constraint_mask": getattr(self.safety_monitor, "env_collision_mask", None),
            "self_constraint_mask": getattr(self.safety_monitor, "self_collision_mask", None),
            "constraint_gain": getattr(self.safety_monitor, "k", None),
        }

    def reset(self, context=None) -> None:
        seen = set()
        for component in self.components.values():
            if id(component) in seen:
                continue
            seen.add(id(component))
            reset = getattr(component, "reset", None)
            if callable(reset):
                reset(context)

    def act(self, agent_feedback: dict, task_info: dict):
        nominal, nominal_info = self.nominal_policy.act(agent_feedback, task_info)
        safe, safety_info = self.safety_filter.safe_control(
            x=agent_feedback["state"],
            u_ref=nominal,
            agent_feedback=agent_feedback,
            task_info=task_info,
            action_info=nominal_info,
        )
        safe = np.asarray(safe, dtype=float)
        if not np.all(np.isfinite(safe)):
            safe = np.zeros_like(safe)
            safety_info = dict(safety_info)
            safety_info["invalid_safe_control"] = True

        if self.clip_action_to_control_limits and self.robot_cfg is not None:
            for control_id in self.robot_cfg.Control:
                safe[control_id] = np.clip(
                    safe[control_id],
                    -self.robot_cfg.ControlLimit[control_id],
                    self.robot_cfg.ControlLimit[control_id],
                )

        info = {**nominal_info, **safety_info, "u_ref": nominal, "u_safe": safe}
        self._add_base_control_info(info, nominal, safe)
        post_safe_control = getattr(self.nominal_policy, "post_safe_control", None)
        if callable(post_safe_control):
            info = post_safe_control(
                agent_feedback=agent_feedback,
                task_info=task_info,
                action_info=info,
                u_ref=nominal,
                u_safe=safe,
                safe_control_info=safety_info,
            )
            info["u_ref"] = nominal
            info["u_safe"] = safe
            self._add_base_control_info(info, nominal, safe)
        return safe, info

    def _add_base_control_info(self, info, nominal, safe) -> None:
        if self.robot_cfg is None:
            return
        controls = self.robot_cfg.Control.__members__
        indices = [
            int(controls[name])
            for name in ("vLinearX", "vLinearY", "vRotYaw", "vLinearZ")
            if name in controls
        ]
        if not indices:
            return
        nominal_values = np.asarray(nominal, dtype=float).reshape(-1)[indices]
        safe_values = np.asarray(safe, dtype=float).reshape(-1)[indices]
        info["safe_base_control_indices"] = np.asarray(indices, dtype=int)
        info["safe_base_u_ref"] = nominal_values.copy()
        info["safe_base_u_safe"] = safe_values.copy()
        info["safe_base_u_delta"] = (safe_values - nominal_values).copy()
        info["safe_base_motion_cmd"] = safe_values.copy()
        info["safe_base_motion_cmd_norm"] = float(np.linalg.norm(safe_values))
        if "trigger_safe" in info:
            info["safe_base_trigger_safe"] = int(bool(info["trigger_safe"]))
