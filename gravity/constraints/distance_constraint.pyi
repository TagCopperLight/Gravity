from ..system_state import SystemState
from .constraint import Constraint
from .constraint_row import ConstraintRow
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


class DistanceConstraint(Constraint):
    def __init__(self, length: float, body1: RigidBody, body2: RigidBody, local_pos1: Vector2 | None = ..., local_pos2: Vector2 | None = ...) -> None:
        self.length: float

        self.body_1: RigidBody
        self.body_2: RigidBody

        self.local_position_1: Vector2
        self.local_position_2: Vector2

    def rows(self, system_state: SystemState) -> list[ConstraintRow]:
        ...
