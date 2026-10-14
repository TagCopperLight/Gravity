from ..system_state import SystemState
from ..constraints.constraint import Constraint


class ConstraintSolver:
    def solve(self, system_state: SystemState, constraints: list[Constraint]) -> None:
        ...

    def stabilize(self, system_state: SystemState, constraints: list[Constraint]) -> None:
        ...
