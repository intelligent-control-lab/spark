class PolicyError(RuntimeError):
    """Base error for policy construction and execution."""


class CompositionError(PolicyError):
    """Raised when policy components cannot be composed consistently."""
