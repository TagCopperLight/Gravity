from ..system_state import SystemState
from ..rigid_body_systems.rigid_body import RigidBody


class ForceGenerator:
    def __init__(self) -> None:
        raise NotImplementedError

    def apply(self, system_state: SystemState) -> None:
        raise NotImplementedError

    def energy(self, bodies: list[RigidBody]) -> float:
        """Potential energy stored by the force, 0 for forces that do not derive from a potential."""
        return 0