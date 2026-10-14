from dataclasses import dataclass, field

from ..system_state import SystemState

from ..math.vector import Vector2


@dataclass(slots=True)
class JacobianEntry:
    """The part of a constraint row acting on one body: dC/dx for its position (linear) and its angle (angular)."""

    index: int
    linear: Vector2
    angular: float


@dataclass(slots=True)
class ConstraintRow:
    """One scalar constraint C(q) = 0, linearised at the current state.

    value is C, entries hold the Jacobian J, and bias is the velocity product dJ/dt * dq/dt
    so that the constraint acceleration is J * W * F + bias, with W the inverse mass matrix.
    """

    value: float
    bias: float
    entries: list[JacobianEntry] = field(default_factory=list)

    def effective_mass(self, system_state: SystemState) -> float:
        """1 / (J W J^T), the diagonal term Gauss-Seidel divides by. 0 if no body can move."""
        inverse = 0.0
        for entry in self.entries:
            inverse += system_state.inverse_masses[entry.index] * entry.linear.length_squared()
            inverse += system_state.inverse_inertias[entry.index] * entry.angular ** 2

        return 0 if inverse == 0 else 1 / inverse

    def velocity(self, system_state: SystemState) -> float:
        """dC/dt = J * v"""
        velocity = 0.0
        for entry in self.entries:
            velocity += entry.linear.dot(system_state.velocities[entry.index])
            velocity += entry.angular * system_state.angular_velocities[entry.index]

        return velocity

    def acceleration(self, system_state: SystemState) -> float:
        """d2C/dt2 = J * W * F + bias, with the forces currently accumulated in the system state."""
        acceleration = self.bias
        for entry in self.entries:
            acceleration += system_state.inverse_masses[entry.index] * entry.linear.dot(system_state.forces[entry.index])
            acceleration += system_state.inverse_inertias[entry.index] * entry.angular * system_state.torques[entry.index]

        return acceleration

    def apply_force(self, system_state: SystemState, magnitude: float) -> None:
        """F += J^T * magnitude"""
        for entry in self.entries:
            system_state.forces[entry.index] += entry.linear * magnitude
            system_state.torques[entry.index] += entry.angular * magnitude

    def apply_impulse(self, system_state: SystemState, magnitude: float) -> None:
        """v += W * J^T * magnitude"""
        for entry in self.entries:
            system_state.velocities[entry.index] += entry.linear * (magnitude * system_state.inverse_masses[entry.index])
            system_state.angular_velocities[entry.index] += entry.angular * magnitude * system_state.inverse_inertias[entry.index]

    def apply_displacement(self, system_state: SystemState, magnitude: float) -> None:
        """q += W * J^T * magnitude"""
        for entry in self.entries:
            system_state.positions[entry.index] += entry.linear * (magnitude * system_state.inverse_masses[entry.index])
            system_state.angles[entry.index] += entry.angular * magnitude * system_state.inverse_inertias[entry.index]
