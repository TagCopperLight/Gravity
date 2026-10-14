from ..system_state import SystemState
from .constraint import Constraint
from .constraint_row import ConstraintRow, JacobianEntry
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


class DistanceConstraint(Constraint):
    """Keeps two points, fixed on two bodies, at a constant distance: a rigid massless rod.

    C = |p2 - p1| - length
    """

    def __init__(self, length: float, body1: RigidBody, body2: RigidBody, local_pos1: Vector2 | None = None, local_pos2: Vector2 | None = None) -> None:
        self.length: float = length

        self.body_1: RigidBody = body1
        self.body_2: RigidBody = body2

        self.local_position_1: Vector2 = local_pos1 if local_pos1 is not None else Vector2()
        self.local_position_2: Vector2 = local_pos2 if local_pos2 is not None else Vector2()

    def rows(self, system_state: SystemState) -> list[ConstraintRow]:
        index_1 = system_state.body_indices[self.body_1]
        index_2 = system_state.body_indices[self.body_2]

        r_1 = self.local_position_1.rotate(system_state.angles[index_1])
        r_2 = self.local_position_2.rotate(system_state.angles[index_2])

        delta = (system_state.positions[index_2] + r_2) - (system_state.positions[index_1] + r_1)
        distance = delta.length()

        if distance == 0:
            return []

        direction = delta / distance
        delta_velocity = system_state.velocity_at_point(self.body_2, self.local_position_2) - system_state.velocity_at_point(self.body_1, self.local_position_1)

        # d2C/dt2 = n . (a_p2 - a_p1) + dn/dt . (v_p2 - v_p1), where the point accelerations
        # carry a centripetal -w^2 r term not covered by J and dn/dt is the velocity normal to n over the distance
        centripetal = r_1 * system_state.angular_velocities[index_1] ** 2 - r_2 * system_state.angular_velocities[index_2] ** 2
        normal_velocity_squared = delta_velocity.length_squared() - delta_velocity.dot(direction) ** 2
        bias = direction.dot(centripetal) + normal_velocity_squared / distance

        return [ConstraintRow(distance - self.length, bias, [
            JacobianEntry(index_1, -direction, -r_1.cross(direction)),
            JacobianEntry(index_2, direction, r_2.cross(direction)),
        ])]
