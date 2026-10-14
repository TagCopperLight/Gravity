from ..system_state import SystemState
from ..constraints.constraint import Constraint


class ConstraintSolver:
    def solve(self, system_state: SystemState, constraints: list[Constraint]) -> None:
        """Adds the constraint forces keeping d2C/dt2 = 0 to the forces accumulated in the system state."""
        raise NotImplementedError

    def stabilize(self, system_state: SystemState, constraints: list[Constraint]) -> None:
        """Projects positions and velocities back onto C = 0 and dC/dt = 0 to remove integration drift."""
        raise NotImplementedError
