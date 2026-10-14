from .rigid_body_systems.rigid_body import RigidBody

from .math.vector import Vector2


class SystemState:
    def __init__(self) -> None:
        self.bodies: list[RigidBody] = []
        self.body_indices: dict[RigidBody, int] = {}

        self.positions: list[Vector2] = []
        self.velocities: list[Vector2] = []
        self.accelerations: list[Vector2] = []
        self.forces: list[Vector2] = []
        self.masses: list[float] = []
        self.inverse_masses: list[float] = []

        self.angles: list[float] = []
        self.angular_velocities: list[float] = []
        self.angular_accelerations: list[float] = []
        self.torques: list[float] = []
        self.inertias: list[float] = []
        self.inverse_inertias: list[float] = []

    def local_to_world(self, body: RigidBody, local_point: Vector2) -> Vector2:
        index = self.body_indices[body]
        return local_point.rotate(self.angles[index]) + self.positions[index]
    
    def velocity_at_point(self, body: RigidBody, local_point: Vector2) -> Vector2:
        index = self.body_indices[body]
        r = local_point.rotate(self.angles[index])
        return self.velocities[index] + self.angular_velocities[index] * Vector2(-r.y, r.x)

    def apply_force(self, body: RigidBody, force: Vector2, local_point: Vector2) -> None:
        index = self.body_indices[body]
        self.forces[index] += force
        self.torques[index] += local_point.rotate(self.angles[index]).cross(force)

    def copy(self) -> 'SystemState':
        copy = self.__class__()
        copy.bodies = self.bodies.copy()
        copy.body_indices = self.body_indices.copy()
        copy.positions = self.positions.copy()
        copy.velocities = self.velocities.copy()
        copy.accelerations = self.accelerations.copy()
        copy.forces = self.forces.copy()
        copy.masses = self.masses.copy()
        copy.inverse_masses = self.inverse_masses.copy()
        copy.angles = self.angles.copy()
        copy.angular_velocities = self.angular_velocities.copy()
        copy.angular_accelerations = self.angular_accelerations.copy()
        copy.torques = self.torques.copy()
        copy.inertias = self.inertias.copy()
        copy.inverse_inertias = self.inverse_inertias.copy()
        return copy