from typing import Any, Callable, Optional, Protocol, Sequence

__version__: str

class Command:
    @staticmethod
    def switch(group: str, at: float, green: bool) -> Command: ...
    @staticmethod
    def wake_at(at: float) -> Command: ...
    @staticmethod
    def set_vehicles_per_green(group: str, n: Optional[int] = None) -> Command: ...

class ControlView:
    def time(self) -> float: ...
    def group(self, id: str) -> Optional[int]: ...
    def is_green(self, id: str) -> Optional[bool]: ...
    def vehicles(self, link_id: int) -> int: ...
    def queue(self, link_id: int) -> int: ...
    def entry_queue(self, link_id: int) -> int: ...
    def density(self, link_id: int) -> float: ...
    def occupancy(self, link_id: int) -> float: ...
    def entered(self, link_id: int) -> int: ...
    def exited(self, link_id: int) -> int: ...

class Controller(Protocol):
    def start(self, view: ControlView) -> list[Command]: ...
    def wake(self, view: ControlView) -> list[Command]: ...

class Simulation:
    @staticmethod
    def from_dir(
        path: str,
        patches: Optional[Sequence[str | dict[str, Any]]] = None,
        assignment: bool = True,
        merge: Optional[str] = None,
        upstream_capacity: bool = False,
    ) -> Simulation: ...
    @staticmethod
    def from_inputs(
        network: dict[str, Any],
        demand: list[dict[str, Any]],
        config: dict[str, Any],
        signals: Optional[dict[str, Any]] = None,
        patches: Optional[Sequence[str | dict[str, Any]]] = None,
        assignment: bool = True,
        merge: Optional[str] = None,
        upstream_capacity: bool = False,
    ) -> Simulation: ...
    def add_controller(self, controller: Controller) -> None: ...
    def run(self) -> int: ...
    def collect(
        self,
        events_processed: int = 0,
        bin_size: float = 60.0,
        keep_vehicles: bool = False,
    ) -> dict[str, Any]: ...
    def verify(self, bin_size: float = 300.0) -> dict[str, Any]: ...
    def compare_observations(self, observed: list[dict[str, Any]]) -> dict[str, Any]: ...
    def signal_log(self) -> list[dict[str, Any]]: ...
    @property
    def current_time(self) -> float: ...
    @property
    def vehicles_count(self) -> int: ...
    @property
    def links_count(self) -> int: ...
    @property
    def nodes_count(self) -> int: ...
    @property
    def warnings(self) -> list[str]: ...

def check(
    inputs: Optional[str | dict[str, Any]] = None,
    *,
    dir: Optional[str] = None,
    network: Optional[dict[str, Any]] = None,
    demand: Optional[list[dict[str, Any]]] = None,
    config: Optional[dict[str, Any]] = None,
    signals: Optional[dict[str, Any]] = None,
    patches: Optional[Sequence[str | dict[str, Any]]] = None,
) -> dict[str, Any]: ...

def run(
    inputs: Optional[str | dict[str, Any]] = None,
    *,
    dir: Optional[str] = None,
    network: Optional[dict[str, Any]] = None,
    demand: Optional[list[dict[str, Any]]] = None,
    config: Optional[dict[str, Any]] = None,
    signals: Optional[dict[str, Any]] = None,
    patches: Optional[Sequence[str | dict[str, Any]]] = None,
    assignment: bool = True,
    merge: Optional[str] = None,
    upstream_capacity: bool = False,
    bin_size: float = 60.0,
    keep_vehicles: bool = False,
    controllers: Optional[Sequence[Controller]] = None,
    progress: Optional[Callable[[dict[str, Any]], Optional[bool]]] = None,
) -> dict[str, Any]: ...

def verify(
    inputs: Optional[str | dict[str, Any]] = None,
    *,
    dir: Optional[str] = None,
    network: Optional[dict[str, Any]] = None,
    demand: Optional[list[dict[str, Any]]] = None,
    config: Optional[dict[str, Any]] = None,
    signals: Optional[dict[str, Any]] = None,
    patches: Optional[Sequence[str | dict[str, Any]]] = None,
    assignment: bool = True,
    merge: Optional[str] = None,
    upstream_capacity: bool = False,
    bin_size: float = 300.0,
    benchmarks: bool = False,
    observed: Optional[str | list[dict[str, Any]]] = None,
    controllers: Optional[Sequence[Controller]] = None,
) -> dict[str, Any]: ...

def run_benchmarks() -> list[dict[str, Any]]: ...
