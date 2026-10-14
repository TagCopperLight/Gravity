#!/usr/bin/env python3
"""
Benchmarks the constraint pipeline: steps constrained scenes for a fixed number of frames and reports
the time per frame, the share spent in the constraint solver, and how well the constraints are held.

Every scene is stepped at 60 frames per second with RK4, like simulation/stream.py.

  chain       8 point masses hanging from a static anchor by distance constraints (the "chain" stream scene)
  long_chain  the same chain with 64 links
  bars        6 rotating bars pinned at the top and linked end to end, exercising the angular terms
  cloth       a 12 x 8 grid of point masses linked by distance constraints, hanging slack from two pinned corners

Usage:
  bench/constraints.py [--scene NAME ...] [--solver NAME] [--frames N] [--rounds N]
  bench/constraints.py --save ref.json      # store the final state of every scene
  bench/constraints.py --check ref.json     # compare the final state against a stored one
  bench/constraints.py --profile chain      # cProfile one scene and print the hottest functions
"""

import argparse
import cProfile
import json
import pstats
import statistics
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from math import cos, sin
from pathlib import Path

from gravity.math.vector import Vector2
from gravity.rigid_body_systems.rigid_body import RigidBody
from gravity.rigid_body_systems.generic_rigid_body_system import GenericRigidBodySystem
from gravity.force_generators.gravity_force_generator import GravityForceGenerator
from gravity.constraints.constraint import Constraint
from gravity.constraints.distance_constraint import DistanceConstraint
from gravity.constraints.fixed_position_constraint import FixedPositionConstraint
from gravity.constraint_solvers.constraint_solver import ConstraintSolver
from gravity.constraint_solvers.ngs_constraint_solver import NgsConstraintSolver
from gravity.constraint_solvers.ldl_constraint_solver import LdlConstraintSolver
from gravity.ode_solvers.rk4_ode_solver import Rk4OdeSolver

FPS = 60
STEPS = 4

SOLVERS: dict[str, Callable[[], ConstraintSolver]] = {
    "ngs": lambda: NgsConstraintSolver(iterations=20, position_iterations=10),
    "ldl": LdlConstraintSolver,
}


def body(x: float, y: float, mass: float, inertia: float = 0, angle: float = 0) -> RigidBody:
    return RigidBody(Vector2(x, y), Vector2(), mass=mass, angle=angle, angular_velocity=0, inertia=inertia)


def chain_constraints(links: int, spacing: float = 0.35) -> tuple[list[RigidBody], list[Constraint]]:
    anchor = body(-1.4, 1.4, mass=0)
    chain = [anchor, *(body(-1.4 + spacing * (i + 1), 1.4, mass=0.2) for i in range(links))]
    return chain, [DistanceConstraint(spacing, a, b) for a, b in zip(chain, chain[1:])]


def chain_scene() -> tuple[list[RigidBody], list[Constraint]]:
    return chain_constraints(8)


def long_chain_scene() -> tuple[list[RigidBody], list[Constraint]]:
    return chain_constraints(64, spacing=0.05)


def bars_scene() -> tuple[list[RigidBody], list[Constraint]]:
    """Bars of length 0.4 laid out horizontally; each one's right end is linked to the next one's left end."""
    count, half = 6, 0.2
    left, right = Vector2(-half, 0), Vector2(half, 0)
    bars = [body(-1.2 + (2 * half + 0.05) * i, 1.0, mass=0.5, inertia=0.5 * (2 * half) ** 2 / 12, angle=0.1 * i) for i in range(count)]

    constraints: list[Constraint] = [FixedPositionConstraint(bars[0], bars[0].local_to_world(left), left)]
    constraints += [DistanceConstraint(0.05, a, b, right, left) for a, b in zip(bars, bars[1:])]
    return bars, constraints


def cloth_scene() -> tuple[list[RigidBody], list[Constraint]]:
    """The top row starts with some slack between the pins: a taut row of rigid links cannot sag without stretching."""
    columns, rows, spacing, slack_angle = 12, 8, 0.15, 0.4

    top = [Vector2(-0.8, 1.2)]
    for c in range(columns - 1):
        angle = slack_angle * (2 * c / (columns - 2) - 1)
        top.append(top[-1] + Vector2(cos(angle), sin(angle)) * spacing)

    grid = [[body(top[c].x, top[c].y - spacing * r, mass=0.05) for c in range(columns)] for r in range(rows)]

    constraints: list[Constraint] = [FixedPositionConstraint(grid[0][0], grid[0][0].position), FixedPositionConstraint(grid[0][-1], grid[0][-1].position)]
    for r in range(rows):
        for c in range(columns):
            if c + 1 < columns:
                constraints.append(DistanceConstraint(spacing, grid[r][c], grid[r][c + 1]))
            if r + 1 < rows:
                constraints.append(DistanceConstraint(spacing, grid[r][c], grid[r + 1][c]))

    return [b for row in grid for b in row], constraints


SCENES: dict[str, Callable[[], tuple[list[RigidBody], list[Constraint]]]] = {
    "chain": chain_scene,
    "long_chain": long_chain_scene,
    "bars": bars_scene,
    "cloth": cloth_scene,
}


class TimedSolver(ConstraintSolver):
    """Forwards to a constraint solver and accumulates the time spent in solve() and stabilize()."""

    def __init__(self, solver: ConstraintSolver) -> None:
        self.solver = solver
        self.solve_time = 0.0
        self.stabilize_time = 0.0

    def solve(self, system_state, constraints) -> None:
        start = time.perf_counter()
        self.solver.solve(system_state, constraints)
        self.solve_time += time.perf_counter() - start

    def stabilize(self, system_state, constraints) -> None:
        start = time.perf_counter()
        self.solver.stabilize(system_state, constraints)
        self.stabilize_time += time.perf_counter() - start


@dataclass
class Run:
    """One scene stepped for a number of frames."""

    scene: str
    solver: str
    frame_times: list[float] = field(default_factory=list)
    solve_time: float = 0.0
    stabilize_time: float = 0.0
    position_error: float = 0.0
    velocity_error: float = 0.0
    energy_drift: float = 0.0
    state: list[list[float]] = field(default_factory=list)

    @property
    def total_time(self) -> float:
        return sum(self.frame_times)

    @property
    def frame_ms(self) -> float:
        return statistics.median(self.frame_times) * 1000


def constraint_errors(system: GenericRigidBodySystem) -> tuple[float, float]:
    """The largest |C| and |dC/dt| over every constraint row, at the bodies' current state."""
    system.populate_system_state()
    rows = [row for constraint in system.constraints for row in constraint.rows(system.system_state)]
    return max(abs(row.value) for row in rows), max(abs(row.velocity(system.system_state)) for row in rows)


def run_scene(scene: str, solver: str, frames: int) -> Run:
    bodies, constraints = SCENES[scene]()
    timed = TimedSolver(SOLVERS[solver]())
    system = GenericRigidBodySystem(bodies, [GravityForceGenerator()], Rk4OdeSolver(), constraints, timed)

    run = Run(scene, solver)
    initial_energy = system.energy()
    for _ in range(frames):
        start = time.perf_counter()
        system.process(1 / FPS, STEPS)
        run.frame_times.append(time.perf_counter() - start)

    run.solve_time, run.stabilize_time = timed.solve_time, timed.stabilize_time
    run.position_error, run.velocity_error = constraint_errors(system)
    run.energy_drift = (system.energy() - initial_energy) / abs(initial_energy)
    run.state = [[b.position.x, b.position.y, b.angle] for b in bodies]
    return run


def best_of(scene: str, solver: str, frames: int, rounds: int) -> Run:
    """Every round restarts the scene, so all rounds simulate the same thing; the fastest one is kept."""
    return min((run_scene(scene, solver, frames) for _ in range(rounds)), key=lambda run: run.total_time)


def print_runs(runs: list[Run]) -> None:
    header = f"{'scene':<11} {'solver':<8} {'frame ms':>9} {'solve %':>8} {'stab %':>7} {'max |C|':>9} {'max |dC/dt|':>11} {'energy drift':>12}"
    print(header)
    print("-" * len(header))
    for run in runs:
        print(
            f"{run.scene:<11} {run.solver:<8} {run.frame_ms:>9.3f} {run.solve_time / run.total_time:>8.0%} {run.stabilize_time / run.total_time:>7.0%}"
            f" {run.position_error:>9.2e} {run.velocity_error:>11.2e} {run.energy_drift:>12.2%}"
        )


def check_runs(runs: list[Run], reference_path: Path) -> None:
    """Prints the largest difference in position and angle between each scene's final state and the reference."""
    reference = json.loads(reference_path.read_text())
    print(f"\nFinal state against {reference_path}:")
    for run in runs:
        if run.scene not in reference:
            print(f"  {run.scene:<11} not in reference")
            continue
        difference = max(abs(a - b) for body, ref in zip(run.state, reference[run.scene]) for a, b in zip(body, ref))
        print(f"  {run.scene:<11} max difference {difference:.3e}")


def profile_scene(scene: str, solver: str, frames: int) -> None:
    profiler = cProfile.Profile()
    profiler.runcall(run_scene, scene, solver, frames)
    pstats.Stats(profiler).sort_stats("tottime").print_stats(15)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", nargs="+", choices=SCENES, default=list(SCENES))
    parser.add_argument("--solver", nargs="+", choices=SOLVERS, default=["ngs"])
    parser.add_argument("--frames", type=int, default=120, help="frames simulated per round (default 120, two seconds)")
    parser.add_argument("--rounds", type=int, default=3, help="rounds per scene, the fastest is reported (default 3)")
    parser.add_argument("--json", action="store_true", help="print one JSON line per run instead of a table")
    parser.add_argument("--save", type=Path, help="store the final state of every scene in this file")
    parser.add_argument("--check", type=Path, help="compare the final state of every scene with this file")
    parser.add_argument("--profile", choices=SCENES, help="profile one scene instead of benchmarking")
    args = parser.parse_args()

    if args.profile:
        profile_scene(args.profile, args.solver[0], args.frames)
        return

    runs = [best_of(scene, solver, args.frames, args.rounds) for scene in args.scene for solver in args.solver]

    if args.json:
        for run in runs:
            print(json.dumps({
                "scene": run.scene, "solver": run.solver, "frame_ms": run.frame_ms,
                "solve_share": run.solve_time / run.total_time, "stabilize_share": run.stabilize_time / run.total_time,
                "max_c": run.position_error, "max_dc": run.velocity_error, "energy_drift": run.energy_drift,
            }))
    else:
        print_runs(runs)

    if args.save:
        args.save.write_text(json.dumps({run.scene: run.state for run in runs}))
    if args.check:
        check_runs(runs, args.check)


if __name__ == "__main__":
    main()
