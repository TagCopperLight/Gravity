from ..system_state import SystemState
from .force_generator import ForceGenerator
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


class SpringForceGenerator(ForceGenerator):
    def __init__(self, rest_length: float, spring_constant: float, damping: float, body1: RigidBody, body2: RigidBody, local_pos1: Vector2 | None = ..., local_pos2: Vector2 | None = ...) -> None:
        ...

    def apply(self, system_state: SystemState) -> None:
        ...

