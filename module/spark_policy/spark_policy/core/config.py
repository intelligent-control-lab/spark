"""Shared configuration contract for public policy packages."""

from dataclasses import dataclass


@dataclass
class PolicyConfig:
    """Base configuration accepted by policy implementations.

    Concrete policies extend this dataclass with role-specific parameters.  The
    common base deliberately remains small so components are not forced to
    accept configuration that they do not use.
    """

    enabled: bool = True
