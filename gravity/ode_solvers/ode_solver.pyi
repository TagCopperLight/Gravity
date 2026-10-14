from ..system_state import SystemState


class OdeSolver:
    def __init__(self) -> None:
        ...

    def start(self, system_state_initial: SystemState, dt: float) -> None:
        ...
    
    def step(self, system_state: SystemState) -> bool:
        ...

    def solve(self, system_state: SystemState) -> None:
        ...
    
    def end(self) -> None:
        ...