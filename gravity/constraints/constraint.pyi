from ..system_state import SystemState
from .constraint_row import ConstraintRow


class Constraint:
    def __init__(self) -> None:
        ...

    def rows(self, system_state: SystemState) -> list[ConstraintRow]:
        ...
