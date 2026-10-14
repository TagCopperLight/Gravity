from .rigid_body_system import RigidBodySystem
from .rigid_body import RigidBody
from ..force_generators.force_generator import ForceGenerator
from ..constraints.constraint import Constraint
from ..ode_solvers.ode_solver import OdeSolver
from ..constraint_solvers.constraint_solver import ConstraintSolver


class GenericRigidBodySystem(RigidBodySystem):
    def __init__(self, bodies: list[RigidBody], force_generators: list[ForceGenerator], ode_solver: OdeSolver, constraints: list[Constraint] | None = ..., constraint_solver: ConstraintSolver | None = ...) -> None:
        self.ode_solver: OdeSolver
        self.constraint_solver: ConstraintSolver

    def process(self, dt: float, steps: int) -> None:
        ...