from ..system_state import SystemState
from .force_generator import ForceGenerator
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


class StaticForceGenerator(ForceGenerator):
    def __init__(self, body: RigidBody, position: Vector2 | None = None, force: Vector2 | None = None) -> None:
        self.body = body

        self.position = position if position is not None else Vector2()
        self.force = force if force is not None else Vector2()

    def apply(self, system_state: SystemState) -> None:
        system_state.apply_force(self.body, self.force, self.position)
