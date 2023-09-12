from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Mesh


def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy('localhost', 64256)
    bng.open(launch=True)
    # Create a vehicle.
    vehicle = Vehicle('ego_vehicle', model='etki', licence='PYTHON', color='Red')
    # Create a scenario.
    scenario = Scenario('smallgrid', 'mesh_test', description='Mesh analysis')
    # Add the vehicle to the scenario.
    scenario.add_vehicle(vehicle)
    scenario.make(bng)
    # Set simulator to 60hz temporal resolution
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # Get the mesh data for the vehicle.
    mesh = Mesh('mesh1', bng, vehicle)

    print("Driving around, polling the mesh data every 5 seconds...")
    vehicle.ai.set_mode('span')
    for _ in range(100000):
        sleep(5)
        data = mesh.poll()
        # Note: there may be other readings in indices > 0, depending on mesh sensor update rates.
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
