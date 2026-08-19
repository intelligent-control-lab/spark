# Composed Policy

`composed_policy` makes policy dependencies explicit. `SafetyFilteredPolicy`
combines a nominal policy with a configured safety controller. Unitree-specific
compositions provide WBC composition, WBT with safety, and SONIC with safety.
The composition owns conversions between controller-specific commands and the
robot action contract.
