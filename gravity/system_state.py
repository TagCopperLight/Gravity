from .rigid_body_systems.rigid_body import RigidBody

from .math.vector import Vector2


class SystemState:
    def __init__(self) -> None:
        self.bodies: list[RigidBody] = []

        self.positions: list[Vector2] = []
        self.velocities: list[Vector2] = []
        self.accelerations: list[Vector2] = []
        self.forces: list[Vector2] = []
        self.masses: list[float] = []

        self.angles: list[float] = []
        self.angular_velocities: list[float] = []
        self.angular_accelerations: list[float] = []
        self.torques: list[float] = []
        self.inertias: list[float] = []

    def local_to_world(self, body: RigidBody, local_point: Vector2) -> Vector2:
        index = self.bodies.index(body)
        return local_point.rotate(self.angles[index]) + self.positions[index]
    
    def velocity_at_point(self, body: RigidBody, local_point: Vector2) -> Vector2:
        index = self.bodies.index(body)
        r = self.local_to_world(body, local_point) - self.positions[index]
        return self.velocities[index] + self.angular_velocities[index] * Vector2(-r.y, r.x)

    def apply_force(self, body: RigidBody, force: Vector2, local_point: Vector2) -> None:
        index = self.bodies.index(body)
        self.forces[index] += force
        self.torques[index] += (self.local_to_world(body, local_point) - self.positions[index]).cross(force)

    def copy(self) -> 'SystemState':
        copy = self.__class__()
        copy.bodies = self.bodies.copy()
        copy.positions = self.positions.copy()
        copy.velocities = self.velocities.copy()
        copy.accelerations = self.accelerations.copy()
        copy.forces = self.forces.copy()
        copy.masses = self.masses.copy()
        copy.angles = self.angles.copy()
        copy.angular_velocities = self.angular_velocities.copy()
        copy.angular_accelerations = self.angular_accelerations.copy()
        copy.torques = self.torques.copy()
        copy.inertias = self.inertias.copy()
        return copy