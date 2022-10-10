"""
.. module:: checkpoints
    :platform: Windows
    :synopsis: Simple demo on how to use checkpoints in a scenario.

.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>

"""

from beamngpy import BeamNGpy, Scenario, Vehicle

if __name__ == '__main__':
    with BeamNGpy('localhost', 64256) as bng:
        scenario = Scenario('smallgrid', 'waypoint demo')
        vehicle = Vehicle('test_car', model='etk800', rot_quat=(0, 0, 0, 1))
        scenario.add_vehicle(vehicle)

        positions = [(0, -10, 0), (0, -20, 0), (0, -30, 0)]
        scales = [(1, 1, 1)] * 3
        scenario.add_checkpoints(positions, scales)
        scenario.make(bng)

        bng.load_scenario(scenario)
        bng.start_scenario()
        input("press 'Enter' to exit demo")
