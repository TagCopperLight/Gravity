from ..system_state import SystemState
from .ode_solver import OdeSolver


class NSVOdeSolver(OdeSolver):
    def __init__(self) -> None:
        super().__init__()
    
    def start(self, system_state_initial: SystemState, dt: float) -> None:
        self.dt = dt

    def step(self, system_state: SystemState) -> bool:
        return False
    
    def solve(self, system_state: SystemState) -> None:
        for i in range(len(system_state.bodies)):
            system_state.velocities[i] += system_state.accelerations[i] * self.dt
            system_state.angular_velocities[i] += system_state.angular_accelerations[i] * self.dt
            
            system_state.positions[i] += system_state.velocities[i] * self.dt
            system_state.angles[i] += system_state.angular_velocities[i] * self.dt

    def end(self) -> None:
        pass