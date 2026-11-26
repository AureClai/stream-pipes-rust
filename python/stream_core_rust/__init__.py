"""Python bindings for pipe-stream (`stream-pipes-rust`)."""

from typing import Protocol

from ._native import (
    Command,
    ControlView,
    Simulation,
    __version__,
    check,
    run,
    run_benchmarks,
    verify,
)


class Controller(Protocol):
    """Protocol for closed-loop signal / ramp-metering controllers."""

    def start(self, view: ControlView) -> list[Command]: ...

    def wake(self, view: ControlView) -> list[Command]: ...


__all__ = [
    "Command",
    "ControlView",
    "Controller",
    "Simulation",
    "__version__",
    "check",
    "run",
    "run_benchmarks",
    "verify",
]
