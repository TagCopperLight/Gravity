from ..system_state import SystemState
from .constraint import Constraint
from .constraint_row import ConstraintRow
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


AXES: tuple[Vector2, Vector2]


class FixedPositionConstraint(Constraint):
    def __init__(self, body: RigidBody, world_position: Vector2, local_pos: Vector2 | None = ...) -> None:
        self.body: RigidBody
        self.world_position: Vector2
        self.local_position: Vector2

    def rows(self, system_state: SystemState) -> list[ConstraintRow]:
        ...
