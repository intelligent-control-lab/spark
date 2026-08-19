from enum import Enum
import inspect

from spark_env import SparkEnvConfig


class BasePipelineConfig:
    """Base configuration that materializes nested configuration classes."""

    def __init__(self) -> None:
        super().__init__()
        self.init_member_classes(self)

    # taken from git@github.com:leggedrobotics/legged_gym.git
    @staticmethod
    def init_member_classes(obj):
        """Recursively instantiate nested configuration classes."""

        # Iterate over all attributes names
        for key in dir(obj):
            # Disregard builtin attributes
            if key == "__class__":
                continue

            # Get the corresponding attribute object
            var = getattr(obj, key)

            # Check if the attribute is a class but not an Enum
            if inspect.isclass(var) and not issubclass(var, Enum):
                # Instantiate the class
                i_var = var()
                # Set the attribute to the instance instead of the type
                setattr(obj, key, i_var)
                # Recursively init members of the attribute
                BasePipelineConfig.init_member_classes(i_var)

    max_num_steps = -1
    continuous_goal_transition = False

    class robot:
        class cfg:
            class_name = None

        class kinematics:
            class_name = None

    class env(SparkEnvConfig):
        pass

    class policy:
        class_name = None
        clip_action_to_control_limits = True
