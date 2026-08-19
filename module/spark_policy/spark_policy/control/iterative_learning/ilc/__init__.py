from .config import ILCPolicyConfig
from .policy import FrequencyILCPolicy, TimeDomainILCPolicy

FrequencyDomainILC = FrequencyILCPolicy
TimeDomainILC = TimeDomainILCPolicy

__all__ = [
    "FrequencyILCPolicy",
    "TimeDomainILCPolicy",
    "ILCPolicyConfig",
    "FrequencyDomainILC",
    "TimeDomainILC",
]
