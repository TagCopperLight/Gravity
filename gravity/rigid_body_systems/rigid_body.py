from ..math.vector import Vector2


class RigidBody:
    def __init__(self, position: Vector2, velocity: Vector2, mass: float, angle: float, angular_velocity: float, inertia: float) -> None:
        self.position = position
        self.velocity = velocity

        self.mass = mass

        self.angle= angle
        self.angular_velocity = angular_velocity

        self.inertia = inertia
    
    def local_to_world(self, point: Vector2) -> Vector2:
        return point.rotate(self.angle) + self.position

    def world_to_local(self, point: Vector2) -> Vector2:
        return (point - self.position).rotate(-self.angle)

    def inverse_mass(self) -> float:
        return 0 if self.mass == 0 else 1 / self.mass

    def inverse_inertia(self) -> float:
        return 0 if self.inertia == 0 else 1 / self.inertia

    def energy(self) -> float:
        return 0.5 * self.mass * self.velocity.length_squared() + 0.5 * self.inertia * self.angular_velocity ** 2