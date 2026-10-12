from .rigid_body_systems.rigid_body import RigidBody

from .math.vector import Vector2


class SystemState:
    def __init__(self) -> None:
        self.bodies: list[RigidBody]
        self.body_indices: dict[RigidBody, int]
        
        self.positions: list[Vector2]
        self.velocities: list[Vector2]
        self.accelerations: list[Vector2]
        self.forces: list[Vector2]
        self.masses: list[float]
        self.inverse_masses: list[float]

        self.angles: list[float]
        self.angular_velocities: list[float]
        self.angular_accelerations: list[float]
        self.torques: list[float]
        self.inertias: list[float]
        self.inverse_inertias: list[float]
    
    def local_to_world(self, body: RigidBody, local_point: Vector2) -> Vector2:
        ...
    
    def velocity_at_point(self, body: RigidBody, local_point: Vector2) -> Vector2:
        ...

    def apply_force(self, body: RigidBody, force: Vector2, local_point: Vector2) -> None:
        ...
    
    def copy(self) -> 'SystemState':
        ...