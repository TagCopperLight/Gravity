from ..system_state import SystemState
from ..constraints.constraint import Constraint
from ..constraints.constraint_row import ConstraintRow
from .constraint_solver import ConstraintSolver


class NgsConstraintSolver(ConstraintSolver):
    """Gauss-Seidel constraint solver with Nonlinear Gauss-Seidel (NGS) position stabilization.

    solve() finds the constraint forces J^T * lambda with (J W J^T) lambda = -(J W F + bias) by Gauss-Seidel,
    relaxing one row at a time against the forces already applied by the others. Lambdas are kept between
    calls to warm start the next solve, which is close to the last one between ODE stages and substeps.

    stabilize() runs NGS on positions: each row is re-linearised at the current positions and moved by
    -C / (J W J^T) along W J^T, then the velocities are projected the same way onto dC/dt = 0.
    """

    def __init__(self, iterations: int = 10, position_iterations: int = 4) -> None:
        self.iterations = iterations
        self.position_iterations = position_iterations

        self.lambdas: list[float] = []

    @staticmethod
    def evaluate(system_state: SystemState, constraints: list[Constraint]) -> list[ConstraintRow]:
        return [row for constraint in constraints for row in constraint.rows(system_state)]

    def solve(self, system_state: SystemState, constraints: list[Constraint]) -> None:
        rows = self.evaluate(system_state, constraints)
        masses = [row.effective_mass(system_state) for row in rows]

        if len(self.lambdas) != len(rows):
            self.lambdas = [0.0] * len(rows)

        for row, magnitude in zip(rows, self.lambdas):
            row.apply_force(system_state, magnitude)

        for _ in range(self.iterations):
            for i, row in enumerate(rows):
                delta = -row.acceleration(system_state) * masses[i]
                self.lambdas[i] += delta
                row.apply_force(system_state, delta)

    def stabilize(self, system_state: SystemState, constraints: list[Constraint]) -> None:
        for _ in range(self.position_iterations):
            for constraint in constraints:
                for row in constraint.rows(system_state):
                    row.apply_displacement(system_state, -row.value * row.effective_mass(system_state))

        rows = self.evaluate(system_state, constraints)
        masses = [row.effective_mass(system_state) for row in rows]

        for _ in range(self.iterations):
            for row, mass in zip(rows, masses):
                row.apply_impulse(system_state, -row.velocity(system_state) * mass)
