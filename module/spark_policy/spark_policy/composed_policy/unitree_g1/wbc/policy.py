"""Goal-tracking plus whole-body-control composition."""

from spark_policy.core import Policy
from spark_utils import initialize_class


class UnitreeG1WBCComposedPolicy(Policy):
    def __init__(
        self,
        goal_tracking_policy,
        wbc_controller=None,
        wbc_policy=None,
        robot_cfg=None,
        robot_kinematics=None,
        **kwargs,
    ):
        controller = wbc_controller if wbc_controller is not None else wbc_policy
        self.goal_tracking_policy = self._resolve(goal_tracking_policy, robot_cfg, robot_kinematics)
        self.wbc_controller = self._resolve(controller, robot_cfg, robot_kinematics)

    @staticmethod
    def _resolve(component, robot_cfg, robot_kinematics):
        if component is None or callable(getattr(component, "act", None)):
            return component
        return initialize_class(
            component,
            robot_cfg=robot_cfg,
            robot_kinematics=robot_kinematics,
        )

    @property
    def components(self):
        return {"goal_tracking": self.goal_tracking_policy, "wbc": self.wbc_controller}

    def reset(self, context=None) -> None:
        for component in self.components.values():
            reset = getattr(component, "reset", None)
            if callable(reset):
                reset(context)

    def act(self, agent_feedback: dict, task_info: dict):
        reference, tracking_info = self.goal_tracking_policy.act(agent_feedback, task_info)
        wbc_task = {**task_info, **tracking_info, "ref_ddq": reference}
        action, wbc_info = self.wbc_controller.act(agent_feedback, wbc_task)
        info = {**tracking_info, **wbc_info, "u_ref": action, "u_safe": action}
        goal = task_info.get("goal_teleop", {})
        for key in (
            "left_gripper_goal",
            "right_gripper_goal",
            "gripper_goal",
            "left_grasp_target_name",
            "right_grasp_target_name",
            "grasp_target_name",
            "left_grasp_radius",
            "right_grasp_radius",
            "grasp_radius",
        ):
            if key in goal:
                info[key] = goal[key]
        return action, info
