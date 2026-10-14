from ..system_state import SystemState
from .constraint_row import ConstraintRow


class Constraint:
    def __init__(self) -> None:
        raise NotImplementedError

    def rows(self, system_state: SystemState) -> list[ConstraintRow]:
        """The scalar constraints this constraint is made of, evaluated at the current state."""
        raise NotImplementedError
