from ..system_state import SystemState
from .force_generator import ForceGenerator


class GravitationForceGenerator(ForceGenerator):
    def __init__(self) -> None:
        self.gravitational_constant = 6.67408e-11

    def apply(self, system_state: SystemState) -> None:
        for i in range(len(system_state.bodies)):
            for j in range(i + 1, len(system_state.bodies)):
                distance = system_state.positions[i] - system_state.positions[j]
                distance_squared = distance.length_squared()

                if distance_squared == 0:
                    continue

                force = distance.normalize() * (self.gravitational_constant * system_state.masses[i] * system_state.masses[j] / distance_squared)

                system_state.forces[j] += force
                system_state.forces[i] -= force