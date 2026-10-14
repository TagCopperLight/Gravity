from ..system_state import SystemState
from .rigid_body import RigidBody
from ..force_generators.force_generator import ForceGenerator
from ..constraints.constraint import Constraint

from ..math.vector import Vector2


class RigidBodySystem:
    def __init__(self, bodies: list[RigidBody], force_generators: list[ForceGenerator], constraints: list[Constraint] | None = None) -> None:
        self.bodies = bodies
        self.force_generators = force_generators
        self.constraints = constraints if constraints is not None else []

        self.system_state = SystemState()

    def process(self, dt: float, steps: int) -> None:
        raise NotImplementedError

    def kinetic_energy(self) -> float:
        return sum(body.energy() for body in self.bodies)

    def potential_energy(self) -> float:
        return sum(generator.energy(self.bodies) for generator in self.force_generators)

    def energy(self) -> float:
        return self.kinetic_energy() + self.potential_energy()

    def populate_system_state(self) -> None:
        self.system_state.bodies = self.bodies
        self.system_state.body_indices = {body: i for i, body in enumerate(self.bodies)}

        self.system_state.accelerations.clear()
        self.system_state.velocities.clear()
        self.system_state.positions.clear()
        self.system_state.masses.clear()
        self.system_state.inverse_masses.clear()
        self.system_state.angular_accelerations.clear()
        self.system_state.angular_velocities.clear()
        self.system_state.angles.clear()
        self.system_state.inertias.clear()
        self.system_state.inverse_inertias.clear()

        for i in range(len(self.bodies)):
            self.system_state.accelerations.append(Vector2())
            self.system_state.velocities.append(self.bodies[i].velocity)
            self.system_state.positions.append(self.bodies[i].position)

            self.system_state.masses.append(self.bodies[i].mass)
            self.system_state.inverse_masses.append(self.bodies[i].inverse_mass())
            
            self.system_state.angular_accelerations.append(0)
            self.system_state.angular_velocities.append(self.bodies[i].angular_velocity)
            self.system_state.angles.append(self.bodies[i].angle)

            self.system_state.inertias.append(self.bodies[i].inertia)
            self.system_state.inverse_inertias.append(self.bodies[i].inverse_inertia())
        
    def process_forces(self) -> None:
        self.system_state.forces = [Vector2() for _ in range(len(self.system_state.bodies))]
        self.system_state.torques = [0 for _ in range(len(self.system_state.bodies))]
        
        for generator in self.force_generators:
            generator.apply(self.system_state)