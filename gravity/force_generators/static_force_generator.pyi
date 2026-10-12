from ..system_state import SystemState
from .force_generator import ForceGenerator
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


class StaticForceGenerator(ForceGenerator):
    def __init__(self, body: RigidBody, position: Vector2 | None = ..., force: Vector2 | None = ...) -> None:
        ...

    def apply(self, system_state: SystemState) -> None:
        ...