from spark_utils import initialize_class
from abc import ABC, abstractmethod
from .base_pipeline_config import BasePipelineConfig
from spark_robot import RobotConfig, RobotKinematics
from spark_env import SparkEnvironment
from spark_policy import Policy
from typing import Generic, TypeVar

PipelineConfigT = TypeVar("PipelineConfigT", bound="BasePipelineConfig")


class BasePipeline(ABC, Generic[PipelineConfigT]):
    def __init__(self, cfg: PipelineConfigT):

        self.cfg = cfg
        self.max_num_steps = self.cfg.max_num_steps

        self.robot_cfg: RobotConfig = initialize_class(self.cfg.robot.cfg)
        self.cfg.robot.kinematics.class_name = self.robot_cfg.kinematics_class_name
        self.robot_kinematics: RobotKinematics = initialize_class(
            self.cfg.robot.kinematics, robot_cfg=self.robot_cfg
        )
        self.env: SparkEnvironment = SparkEnvironment(
            self.cfg.env,
            robot_cfg=self.robot_cfg,
            robot_kinematics=self.robot_kinematics,
        )
        if not getattr(self.cfg.policy, "class_name", None):
            raise ValueError("cfg.policy.class_name must select an executable Policy")
        self.policy: Policy = initialize_class(
            self.cfg.policy,
            robot_cfg=self.robot_cfg,
            robot_kinematics=self.robot_kinematics,
        )

        self.pipeline_step = 0

    @abstractmethod
    def post_physics_step(self):
        pass

    def run(self):

        # reset environment
        agent_feedback, task_info = self.env.reset()

        # initial action
        action, action_info = self.policy.act(agent_feedback, task_info)

        while self.pipeline_step < self.max_num_steps or self.max_num_steps < 0:
            # environment step
            # s_next = env(s, a)
            agent_feedback, task_info = self.env.step(action, action_info)

            # next action
            # a_next = policy(s_next)
            action, action_info = self.policy.act(agent_feedback, task_info)
            # post physics step (e.g., rendering, status publishing)
            self.post_physics_step(agent_feedback, task_info, action_info)

            self.pipeline_step += 1

            if task_info["done"]:
                break

        # end of pipeline
        print("Pipeline done")
