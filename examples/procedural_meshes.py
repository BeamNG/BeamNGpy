"""
Example code that places procedurally generated primitive
shapes in the environment.
"""

from beamngpy import (
    BeamNGpy,
    ProceduralBump,
    ProceduralCone,
    ProceduralCube,
    ProceduralCylinder,
    ProceduralRing,
    Scenario,
    Vehicle,
    set_up_simple_logging,
)


def main():
    set_up_simple_logging()
    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)

    scenario = Scenario("tech_ground", "mesh_test")

    cylinder = ProceduralCylinder(
        name="cylinder", radius=3.5, height=5, pos=(10, -10, 0), rot_quat=(0, 0, 0, 1)
    )
    scenario.add_procedural_mesh(cylinder)

    bump = ProceduralBump(
        name="bump",
        pos=(-10, -10, 0),
        rot_quat=(0, 0, 0, 1),
        width=5,
        length=7,
        height=2,
        upper_length=2,
        upper_width=2,
    )
    scenario.add_procedural_mesh(bump)

    cone = ProceduralCone(
        name="cone", pos=(-10, -20, 0), rot_quat=(0, 0, 0, 1), radius=3.5, height=5
    )
    scenario.add_procedural_mesh(cone)

    cube = ProceduralCube(
        name="cube", pos=(0, -20, 0), rot_quat=(0, 0, 0, 1), size=(5, 2, 3)
    )
    scenario.add_procedural_mesh(cube)

    ring = ProceduralRing(
        name="ring",
        pos=(10, -20, 0),
        rot_quat=(0, 0.7071068, 0, 0.7071068),
        radius=2,
        thickness=1,
    )
    scenario.add_procedural_mesh(ring)

    vehicle = Vehicle("ego_vehicle", model="etk800")
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1))

    scenario.make(beamng)

    beamng.scenario.load(scenario)
    beamng.scenario.start()
    input("Press Enter when done...")
    beamng.disconnect()


if __name__ == "__main__":
    main()
