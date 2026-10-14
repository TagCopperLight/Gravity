# Gravity

A small 2D rigid body physics engine in pure Python. Bodies are moved by force
generators (uniform gravity, N-body gravitation, springs, static forces) and
integrated with an interchangeable ODE solver (Euler, semi-implicit Euler, RK4).

## Install

Gravity has no dependencies and needs Python 3.10 or newer. Add it to a uv project
from a local checkout:

```sh
uv add --editable ./Gravity
```

## Usage

```python
from gravity.math.vector import Vector2
from gravity.rigid_body_systems.rigid_body import RigidBody
from gravity.rigid_body_systems.generic_rigid_body_system import GenericRigidBodySystem
from gravity.force_generators.gravity_force_generator import GravityForceGenerator
from gravity.force_generators.spring_force_generator import SpringForceGenerator
from gravity.ode_solvers.rk4_ode_solver import Rk4OdeSolver

anchor = RigidBody(Vector2(0, 0), Vector2(), mass=0, angle=0, angular_velocity=0, inertia=0)
weight = RigidBody(Vector2(1, -1), Vector2(), mass=1, angle=0, angular_velocity=0, inertia=0.1)

system = GenericRigidBodySystem(
    [anchor, weight],
    [GravityForceGenerator(), SpringForceGenerator(1, 50, 0.5, anchor, weight)],
    Rk4OdeSolver(),
)

for _ in range(60):
    system.process(dt=1 / 60, steps=4)

print(weight.position)
```