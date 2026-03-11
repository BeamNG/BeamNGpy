"""
Visual hand-test for the `cling` parameter of the DebugApi functions.

Spawns all clingable debug shapes in one scenario on "tech_ground":
  - GREEN row (cling=True)  : shapes should appear ON the ground (z ≈ 0)
  - RED   row (cling=False) : shapes should float 5 m above the ground (z = 5)

Layout - 7 columns, 5 m apart along X, two rows along Y:
  Y =  0 : cling=True  (green)
  Y = 10 : cling=False (red)

  Col  X   Shape
   0   0   Sphere
   1   5   Cylinder
   2  10   Polyline
   3  15   Triangle
   4  20   Rectangle
   5  25   Text
   6  30   Square prism
"""

import numpy as np

from beamngpy import BeamNGpy, Scenario, set_up_simple_logging

Z      = 5.0          # z value passed to every shape
COL_W  = 5.0          # column width (metres)
GREEN  = (0.0, 1.0, 0.0, 1.0)
RED    = (1.0, 0.0, 0.0, 1.0)


def _col(i: int, dx: float = 0.0, y: float = 0.0) -> tuple:
    """Return (x, y, Z) for column i with optional x/y offsets."""
    return (i * COL_W + dx, y, Z)


def spawn_all(bng: BeamNGpy, y: float, color: tuple, cling: bool) -> None:
    """Spawn one of every clingable debug shape at the given Y row."""

    # 0 - Sphere
    bng.debug.add_spheres(
        [_col(0, y=y)],
        [0.6],
        [color],
        cling=cling,
    )

    # 1 - Cylinder  (two end-caps 2 m apart along X)
    bng.debug.add_cylinder(
        [_col(1, dx=0.0, y=y), _col(1, dx=2.0, y=y)],
        0.4,
        color,
        cling=cling,
    )

    # 2 - Polyline  (three points in a small triangle pattern)
    bng.debug.add_polyline(
        [_col(2, dx=0.0, y=y), _col(2, dx=2.0, y=y), _col(2, dx=1.0, y=y + 2.0)],
        color,
        cling=cling,
    )

    # 3 - Triangle
    center = _col(3, dx=1.0, y=y)
    bng.debug.add_triangle(
        [
            (center[0], center[1], center[2]),         # bottom
            (center[0] - 1.0, center[1], center[2] + 2.0),  # left, up
            (center[0] + 1.0, center[1], center[2] + 2.0),  # right, up
        ],
        color,
        cling=cling,
    )

    # 4 - Rectangle
    bng.debug.add_rectangle(
        [
            _col(4, dx=0.0, y=y),
            _col(4, dx=2.0, y=y),
            _col(4, dx=2.0, y=y + 2.0),
            _col(4, dx=0.0, y=y + 2.0),
        ],
        color,
        cling=cling,
    )

    # 5 - Text
    bng.debug.add_text(
        _col(5, y=y),
        f"cling={cling}",
        color,
        cling=cling,
    )

    # 6 - Square prism  (two end-caps 2 m apart along X, 1x1 m cross-section)
    bng.debug.add_square_prism(
        [_col(6, dx=0.0, y=y), _col(6, dx=2.0, y=y)],
        [(1.0, 1.0), (1.0, 1.0)],
        color,
        cling=cling,
    )


def main():
    set_up_simple_logging()

    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)

    scenario = Scenario("tech_ground", "test_cling_visual")
    scenario.make(beamng)
    beamng.scenario.load(scenario)
    beamng.scenario.start()

    # Green row at Y=0  - cling=True  → should sit ON the ground
    spawn_all(beamng, y=0.0,  color=GREEN, cling=True)

    # Red row at Y=10   - cling=False → should float at z=5
    spawn_all(beamng, y=10.0, color=RED,   cling=False)

    print("Shapes spawned. Green (Y=0) should be ON the ground, Red (Y=10) should float at z=5.")
    input("Press Enter to exit...")
    beamng.disconnect()


if __name__ == "__main__":
    main()
