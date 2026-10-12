from ..math.vector import Vector2


class RigidBody:
    def __init__(self, position: Vector2, velocity: Vector2, mass: float, angle: float, angular_velocity: float, inertia: float) -> None:
        self.position: Vector2
        self.velocity: Vector2

        self.mass: float

        self.angle: float
        self.angular_velocity: float

        self.inertia: float
    
    def local_to_world(self, point: Vector2) -> Vector2:
        ...

    def world_to_local(self, point: Vector2) -> Vector2:
        ...

    def inverse_mass(self) -> float:
        ...

    def inverse_inertia(self) -> float:
        ...

    def energy(self) -> float:
        ...