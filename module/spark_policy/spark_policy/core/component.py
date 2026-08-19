"""Structural interface for reusable policy components."""

from __future__ import annotations

from typing import Protocol, TypeVar

from .context import ExecutionContext, ResetContext
from .metadata import ComponentSpec
from .result import ComponentResult

InputT = TypeVar("InputT", contravariant=True)
OutputT = TypeVar("OutputT", covariant=True)


class Component(Protocol[InputT, OutputT]):
    @property
    def spec(self) -> ComponentSpec: ...

    def reset(self, context: ResetContext | None = None) -> None: ...

    def step(
        self,
        inputs: InputT,
        context: ExecutionContext | None = None,
    ) -> ComponentResult[OutputT]: ...
