from ..system_state import SystemState
from .ode_solver import OdeSolver


class Rk4OdeSolver(OdeSolver):
    def __init__(self) -> None:
        super().__init__()
    
    def start(self, system_state_initial: SystemState, dt: float) -> None:
        self.dt = dt
        self.initial_state = system_state_initial.copy()
        self.accumulator = system_state_initial.copy()

        self.stage = 1
        self.next_stage = 1

    def step(self, system_state: SystemState) -> bool:
        if self.stage == 2 or self.stage == 3:
            for i in range(len(system_state.bodies)):
                system_state.angles[i] = self.initial_state.angles[i] + self.dt * system_state.angular_velocities[i] / 2
                system_state.angular_velocities[i] = self.initial_state.angular_velocities[i] + self.dt * system_state.angular_accelerations[i] / 2
                system_state.positions[i] = self.initial_state.positions[i] + self.dt * system_state.velocities[i] / 2
                system_state.velocities[i] = self.initial_state.velocities[i] + self.dt * system_state.accelerations[i] / 2

        elif self.stage == 4:
            for i in range(len(system_state.bodies)):
                system_state.angles[i] = self.initial_state.angles[i] + self.dt * system_state.angular_velocities[i]
                system_state.angular_velocities[i] = self.initial_state.angular_velocities[i] + self.dt * system_state.angular_accelerations[i]
                system_state.positions[i] = self.initial_state.positions[i] + self.dt * system_state.velocities[i]
                system_state.velocities[i] = self.initial_state.velocities[i] + self.dt * system_state.accelerations[i]

        self.next_stage += 1
        
        return self.next_stage <= 4
    
    def solve(self, system_state: SystemState) -> None:
        stage_weight = {1:1, 2:2, 3:2, 4:1}[self.stage]

        for i in range(len(system_state.bodies)):
            self.accumulator.angular_velocities[i] += (self.dt / 6) * stage_weight * system_state.angular_accelerations[i]
            self.accumulator.angles[i] += (self.dt / 6) * stage_weight * system_state.angular_velocities[i]
            self.accumulator.velocities[i] += (self.dt / 6) * stage_weight * system_state.accelerations[i]
            self.accumulator.positions[i] += (self.dt / 6) * stage_weight * system_state.velocities[i]
        
        if self.stage == 4:
            system_state.angular_velocities = self.accumulator.angular_velocities
            system_state.angles = self.accumulator.angles
            system_state.velocities = self.accumulator.velocities
            system_state.positions = self.accumulator.positions
        
        self.stage = self.next_stage

    def end(self) -> None:
        self.stage = -1