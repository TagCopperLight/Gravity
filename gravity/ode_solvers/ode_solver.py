from ..system_state import SystemState


class OdeSolver:
    def __init__(self) -> None:
        self.dt = 0

    def start(self, system_state_initial: SystemState, dt: float) -> None:
        raise NotImplementedError
    
    def step(self, system_state: SystemState) -> bool:
        raise NotImplementedError

    def solve(self, system_state: SystemState) -> None:
        raise NotImplementedError
    
    def end(self) -> None:
        raise NotImplementedError