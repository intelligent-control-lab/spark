"""Configuration for IK followed by joint PID control."""

from dataclasses import dataclass, field

from spark_policy.core import PolicyConfig
from spark_policy.control.pid.joint import JointPIDConfig


@dataclass
class IKJointPIDConfig(PolicyConfig):
    joint: JointPIDConfig = field(default_factory=JointPIDConfig)
