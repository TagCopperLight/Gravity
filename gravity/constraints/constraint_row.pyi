from ..system_state import SystemState

from ..math.vector import Vector2


class JacobianEntry:
    index: int
    linear: Vector2
    angular: float

    def __init__(self, index: int, linear: Vector2, angular: float) -> None:
        ...


class ConstraintRow:
    value: float
    bias: float
    entries: list[JacobianEntry]

    def __init__(self, value: float, bias: float, entries: list[JacobianEntry] = ...) -> None:
        ...

    def effective_mass(self, system_state: SystemState) -> float:
        ...

    def velocity(self, system_state: SystemState) -> float:
        ...

    def acceleration(self, system_state: SystemState) -> float:
        ...

    def apply_force(self, system_state: SystemState, magnitude: float) -> None:
        ...

    def apply_impulse(self, system_state: SystemState, magnitude: float) -> None:
        ...

    def apply_displacement(self, system_state: SystemState, magnitude: float) -> None:
        ...
