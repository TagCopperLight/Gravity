from .rigid_body_system import RigidBodySystem
from ..solvers.ode_solver import OdeSolver
from ..rigid_body_systems.rigid_body import RigidBody
from ..force_generators.force_generator import ForceGenerator


class GenericRigidBodySystem(RigidBodySystem):
    def __init__(self, bodies: list[RigidBody], force_generators: list[ForceGenerator], ode_solver: OdeSolver) -> None:
        super().__init__(bodies, force_generators)
        self.ode_solver = ode_solver

    def process(self, dt: float, steps: int) -> None:
        self.populate_system_state()
        
        for _ in range(steps):
            self.ode_solver.start(self.system_state, dt / steps)
            
            done = True
            while done:
                done = self.ode_solver.step(self.system_state)
                
                self.process_forces()

                for i in range(len(self.system_state.bodies)):
                    self.system_state.accelerations[i] = self.system_state.forces[i] * self.system_state.inverse_masses[i]
                    self.system_state.angular_accelerations[i] = self.system_state.torques[i] * self.system_state.inverse_inertias[i]

                self.ode_solver.solve(self.system_state)
            
            self.ode_solver.end()

        for i in range(len(self.bodies)):
            self.bodies[i].position = self.system_state.positions[i]
            self.bodies[i].velocity = self.system_state.velocities[i]
            
            self.bodies[i].angle = self.system_state.angles[i]
            self.bodies[i].angular_velocity = self.system_state.angular_velocities[i]