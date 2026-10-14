from ..system_state import SystemState
from ..rigid_body_systems.rigid_body import RigidBody
from ..force_generators.force_generator import ForceGenerator
from ..constraints.constraint import Constraint


class RigidBodySystem:
    def __init__(self, bodies: list[RigidBody], force_generators: list[ForceGenerator], constraints: list[Constraint] | None = ...) -> None:
        self.bodies: list[RigidBody]
        self.force_generators: list[ForceGenerator]
        self.constraints: list[Constraint]

        self.system_state: SystemState

    def process(self, dt: float, steps: int) -> None:
        ...

    def kinetic_energy(self) -> float:
        ...

    def potential_energy(self) -> float:
        ...

    def energy(self) -> float:
        ...

    def populate_system_state(self) -> None:
        ...
        
    def process_forces(self) -> None:
        ...