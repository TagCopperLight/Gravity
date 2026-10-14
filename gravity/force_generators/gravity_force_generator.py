from ..system_state import SystemState
from .force_generator import ForceGenerator
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


class GravityForceGenerator(ForceGenerator):
    def __init__(self) -> None:
        self.gravity = Vector2(0, -9.81)

    def apply(self, system_state: SystemState) -> None:
        for i in range(len(system_state.bodies)):
            system_state.forces[i] += system_state.masses[i] * self.gravity

    def energy(self, bodies: list[RigidBody]) -> float:
        return -sum(body.mass * self.gravity.dot(body.position) for body in bodies)