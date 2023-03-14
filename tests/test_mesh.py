from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Mesh

def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy('localhost', 64256)
    bng.open(launch=True)
    vehicle = Vehicle('ego_vehicle', model='etki', licence='PYTHON', color='Red')                       # Create a vehicle.
    scenario = Scenario('gridmap', 'mesh_test', description='Mesh analysis')                            # Create a scenario.
    scenario.add_vehicle(vehicle)                                                                       # Add the vehicle to the scenario.
    scenario.make(bng)
    bng.settings.set_deterministic(60)                                                                  # Set simulator to 60hz temporal resolution
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    print('Mesh test start.')

    # Get the mesh data for the vehicle.
    mesh = Mesh('mesh1', bng, vehicle)
    triangles = mesh.get_triangle_data()
    print("Raw triangles data from simulator (the nodes of each triangle in triplet lists):")
    print(triangles)

    wheel1 = mesh.get_wheel_mesh(0)
    wheel2 = mesh.get_wheel_mesh(1)
    wheel3 = mesh.get_wheel_mesh(2)
    wheel4 = mesh.get_wheel_mesh(3)
    print("Raw triangles (for just a single wheel):")
    print(wheel1)

    nodes = mesh.get_node_positions()
    print("Current node positions from simulator:")
    print(nodes)

    map = mesh.get_nodes_to_triangles_map()
    print("Map from nodes to triangles (the triangles indexed by node numbers):")
    print(map)
    nb = mesh.get_neighbor_nodes(145)
    print ("The neighbor nodes of node 145 (those which share a triangle edge): ", nb)
    nt = mesh.get_neighbor_triangles(52)
    print("The neighbor triangles of node 52 (those which share any of triangle 52's nodes): ", nt)

    cl1 = mesh.get_closest_mesh_point_to_point((100, 100, 100))
    print("closest mesh point to (100, 100, 100): ", cl1)
    ct = mesh.get_closest_vehicle_triangle_to_point((100, 100, 100), True)
    print("closest vehicle triangle to point (100, 100, 100): ", ct)

    bng.close()


if __name__ == '__main__':
    main()
