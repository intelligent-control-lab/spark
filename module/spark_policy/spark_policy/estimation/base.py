from abc import ABC, abstractmethod

from spark_policy.core import ParameterEstimate, StateEstimate


class StateEstimator(ABC):
    def reset(self, context=None) -> None:
        pass

    @abstractmethod
    def estimate(self, observation, context=None) -> StateEstimate: ...


class ParameterEstimator(ABC):
    def reset(self, context=None) -> None:
        pass

    @abstractmethod
    def estimate(self, observation, context=None) -> ParameterEstimate: ...
