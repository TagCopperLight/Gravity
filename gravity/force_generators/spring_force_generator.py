from ..system_state import SystemState
from .force_generator import ForceGenerator
from ..rigid_body_systems.rigid_body import RigidBody

from ..math.vector import Vector2


class SpringForceGenerator(ForceGenerator):
    def __init__(self, rest_length: float, spring_constant: float, damping: float, body1: RigidBody, body2: RigidBody, local_pos1: Vector2 | None = None, local_pos2: Vector2 | None = None) -> None:
        self.rest_length: float = rest_length
        self.spring_constant: float = spring_constant
        self.damping: float = damping

        self.body_1: RigidBody = body1
        self.body_2: RigidBody = body2

        self.local_position_1: Vector2 = local_pos1 if local_pos1 is not None else Vector2()
        self.local_position_2: Vector2 = local_pos2 if local_pos2 is not None else Vector2()

    def apply(self, system_state: SystemState) -> None:
        delta = system_state.local_to_world(self.body_2, self.local_position_2) - system_state.local_to_world(self.body_1, self.local_position_1)
        delta_velocity = system_state.velocity_at_point(self.body_2, self.local_position_2) - system_state.velocity_at_point(self.body_1, self.local_position_1)

        length = delta.length()

        if length == 0:
            return

        direction = delta / length

        magnitude = (length - self.rest_length) * self.spring_constant
        damping_force = delta_velocity.dot(direction) * self.damping

        force = (magnitude + damping_force) * direction

        system_state.apply_force(self.body_1, force, self.local_position_1)
        system_state.apply_force(self.body_2, -force, self.local_position_2)

    def energy(self, bodies: list[RigidBody]) -> float:
        length = (self.body_2.local_to_world(self.local_position_2) - self.body_1.local_to_world(self.local_position_1)).length()
        return 0.5 * self.spring_constant * (length - self.rest_length) ** 2
