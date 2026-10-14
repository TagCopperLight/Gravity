from ..system_state import SystemState
from .force_generator import ForceGenerator
from ..rigid_body_systems.rigid_body import RigidBody


class GravitationForceGenerator(ForceGenerator):
    def __init__(self) -> None:
        ...

    def apply(self, system_state: SystemState) -> None:
        ...

    def energy(self, bodies: list[RigidBody]) -> float:
        ...
