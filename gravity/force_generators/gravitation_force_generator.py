from ..system_state import SystemState
from .force_generator import ForceGenerator
from ..rigid_body_systems.rigid_body import RigidBody


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

    def energy(self, bodies: list[RigidBody]) -> float:
        energy = 0.0
        for i in range(len(bodies)):
            for j in range(i + 1, len(bodies)):
                distance = (bodies[i].position - bodies[j].position).length()

                if distance == 0:
                    continue

                energy -= self.gravitational_constant * bodies[i].mass * bodies[j].mass / distance

        return energy