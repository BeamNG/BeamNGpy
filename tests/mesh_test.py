from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Mesh

def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy('localhost', 64256)
    bng.open(launch=True)
    vehicle = Vehicle('ego_vehicle', model='etki', licence='PYTHON', color='Red')                       # Create a vehicle.
    scenario = Scenario('gridmap', 'mesh_test2', description='Mesh analysis')                            # Create a scenario.
    scenario.add_vehicle(vehicle)                                                                       # Add the vehicle to the scenario.
    scenario.make(bng)
    bng.settings.set_deterministic(60)                                                                  # Set simulator to 60hz temporal resolution
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    print('Mesh test start.')

    # Get the mesh data for the vehicle.
    mesh = Mesh('mesh1', bng, vehicle)

    nodes = mesh.get_node_positions()
    print("Current node positions from simulator:")
    print(nodes)

    #vehicle.ai.set_mode('span')
    for _ in range(100000):
        sleep(5)
        data = mesh.poll()
        print(data)
        print("data time: ", data['time'])
        mesh.mesh_plot()
        mesh.mass_distribution_plot(data['nodes'])
        mesh.force_distribution_plot(data['nodes'])
        mesh.force_direction_plot(data['nodes'])
        mesh.velocity_distribution_plot(data['nodes'])
        mesh.velocity_direction_plot(data['nodes'])

    bng.close()


if __name__ == '__main__':
    main()
