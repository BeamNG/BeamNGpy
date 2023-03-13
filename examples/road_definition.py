"""
.. module:: road_definition

    :platform: Windows
    :synopsis: Example code making a scenario that defines new roads to drive
               on.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>

"""

from beamngpy import BeamNGpy, MeshRoad, Road, Scenario, Vehicle


def main():
    beamng = BeamNGpy('localhost', 64256)
    beamng.open(launch=True)

    scenario = Scenario('gridmap_v2', 'road_test')
    vehicle = Vehicle('ego_vehicle', model='etk800')
    scenario.add_vehicle(vehicle, pos=(-25, 300, 100), rot_quat=(0, 0, 0, 1))

    # create a decal road
    road_a = Road('track_editor_C_center', rid='circle_road', looped=True)
    road_a.add_nodes(
        (-25, 300, 100, 5),
        (25, 300, 100, 6),
        (25, 350, 100, 4),
        (-25, 350, 100, 5),
    )
    scenario.add_road(road_a)

    road_b = Road('track_editor_C_center', rid='center_road')
    road_b.add_nodes(
        (0, 325, 100, 5),
        (50, 375, 100, 5)
    )
    scenario.add_road(road_b)
    
    # create a mesh road
    mesh_road = MeshRoad('track_editor_C_center', rid='main_road')
    mesh_road.add_nodes(
        (-107, 20, 100),
        (-157, 20, 110),
        (-207, 25, 120, 15, 10), # the 4th and 5th arguments are width and depth of the node
        (-257, 15, 110),
        (-307, 20, 100, 30), 
    )
    scenario.add_mesh_road(mesh_road)

    scenario.make(beamng)

    try:
        beamng.scenario.load(scenario)
        beamng.scenario.start()
        input('Press Enter to teleport to the other road...')
        vehicle.teleport((-107, 20, 100), reset=False)
        input('Press Enter when done...')
    finally:
        beamng.close()


if __name__ == '__main__':
    main()
