from ..system_state import SystemState
from ..constraints.constraint import Constraint
from ..constraints.constraint_row import ConstraintRow
from .constraint_solver import ConstraintSolver


class NgsConstraintSolver(ConstraintSolver):
    def __init__(self, iterations: int = ..., position_iterations: int = ...) -> None:
        self.iterations: int
        self.position_iterations: int

        self.lambdas: list[float]

    @staticmethod
    def evaluate(system_state: SystemState, constraints: list[Constraint]) -> list[ConstraintRow]:
        ...

    def solve(self, system_state: SystemState, constraints: list[Constraint]) -> None:
        ...

    def stabilize(self, system_state: SystemState, constraints: list[Constraint]) -> None:
        ...
