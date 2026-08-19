import inspect
from enum import Enum


class SparkEnvConfig:
    class task:
        class_name = None

    class agent:
        class_name = None

    def __init__(self) -> None:
        self.init_member_classes(self)

    @staticmethod
    def init_member_classes(obj):
        for key in dir(obj):
            if key == "__class__":
                continue
            value = getattr(obj, key)
            if inspect.isclass(value) and not issubclass(value, Enum):
                instance = value()
                setattr(obj, key, instance)
                SparkEnvConfig.init_member_classes(instance)
