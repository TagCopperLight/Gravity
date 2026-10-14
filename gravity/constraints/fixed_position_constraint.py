from ..system_state import SystemState
from .constraint import Constraint
from .constraint_row import ConstraintRow, JacobianEntry
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


AXES = (Vector2(1, 0), Vector2(0, 1))


class FixedPositionConstraint(Constraint):
    """Pins a point, fixed on a body, to a point in the world: a frictionless pivot.

    C = p - target, one row per axis.
    """

    def __init__(self, body: RigidBody, world_position: Vector2, local_pos: Vector2 | None = None) -> None:
        self.body: RigidBody = body
        self.world_position: Vector2 = world_position
        self.local_position: Vector2 = local_pos if local_pos is not None else Vector2()

    def rows(self, system_state: SystemState) -> list[ConstraintRow]:
        index = system_state.body_indices[self.body]

        r = self.local_position.rotate(system_state.angles[index])
        error = system_state.positions[index] + r - self.world_position
        # The point acceleration carries a centripetal -w^2 r term not covered by J
        centripetal = -r * system_state.angular_velocities[index] ** 2

        return [ConstraintRow(axis.dot(error), axis.dot(centripetal), [JacobianEntry(index, axis, r.cross(axis))]) for axis in AXES]
