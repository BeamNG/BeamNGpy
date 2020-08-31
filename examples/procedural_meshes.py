"""
.. module:: procedural_meshes

    :platform: Windows
    :synopsis: Example code that places procedurally generated primitive
               shapes in the environment.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""

from beamngpy import BeamNGpy, Scenario, Vehicle, setup_logging
from beamngpy import ProceduralCylinder, ProceduralCone, ProceduralCube


def main():
    beamng = BeamNGpy('localhost', 64256)

    scenario = Scenario('smallgrid', 'mesh_test')

    cylinder = ProceduralCylinder(name='cylinder',
                                  pos=(10, -10, 0),
                                  rot=(0, 0, 0),
                                  radius=3.5,
                                  height=5)
    scenario.add_procedural_mesh(cylinder)

    cone = ProceduralCone(name='cone',
                          pos=(-10, -10, 0),
                          rot=(45, 0, 0),
                          radius=3.5,
                          height=5)
    scenario.add_procedural_mesh(cone)

    cube = ProceduralCube(name='cube',
                          pos=(0, -20, 0),
                          rot=(0, 0, 0),
                          size=(5, 2, 3))
    scenario.add_procedural_mesh(cube)

    vehicle = Vehicle('ego_vehicle', model='etk800')
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot=(0, 0, 0))

    scenario.make(beamng)

    bng = beamng.open(launch=True)
    try:
        bng.load_scenario(scenario)
        bng.start_scenario()
        input('Press enter when done...')
    finally:
        bng.close()


if __name__ == '__main__':
    main()
