from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import GPS

import matplotlib.pyplot as plt


def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    bng.open(launch=True)

    # Create a vehicle.
    vehicle = Vehicle("ego_vehicle", model="etki", licence="PYTHON", color="Red")

    # Create a scenario.
    scenario = Scenario("italy", "GPS_Trajectory", description="GPS")

    # Add the vehicle to the scenario.
    scenario.add_vehicle(
        vehicle,
        pos=(245.11, -906.94, 247.46),
        rot_quat=(0.0010, 0.1242, 0.9884, -0.0872),
    )
    scenario.make(bng)

    # Set simulator to 60hz temporal resolution
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # Set the reference longitude and lattitude (this should be where the map origin is on the world spherical surface).
    ref_lon, ref_lat = 8.8017, 53.0793

    # Create two GPS sensors to sit above the vehicle (for easy visibility) - one towards the front of the car, one towards the rear.
    # Note: we use the same reference longitude and lattitude for both sensors, to sync their positions.
    gps_front = GPS(
        "front",
        bng,
        vehicle,
        pos=(0, 1.5, 2.0),
        ref_lon=ref_lon,
        ref_lat=ref_lat,
        is_visualised=True,
    )
    gps_rear = GPS(
        "rear",
        bng,
        vehicle,
        pos=(0, -1.5, 2.0),
        ref_lon=ref_lon,
        ref_lat=ref_lat,
        is_visualised=True,
    )

    print("Driving around, storing GPS readings every second, for a few seconds...")
    vehicle.ai.set_mode("traffic")
    sleep(3.0)
    front_lon, front_lat, rear_lon, rear_lat = [], [], [], []
    for _ in range(20):
        data_front = gps_front.poll()
        front_latest_reading = data_front[0]
        front_lon.append(front_latest_reading["lon"])
        front_lat.append(front_latest_reading["lat"])

        data_rear = gps_rear.poll()
        rear_latest_reading = data_rear[0]
        rear_lon.append(rear_latest_reading["lon"])
        rear_lat.append(rear_latest_reading["lat"])

        sleep(1.0)

    gps_front.remove()
    gps_rear.remove()
    bng.ui.show_hud()

    # Plot the recorded trajectory.
    fig, ax = plt.subplots()
    ax.set(xlabel="Longitude", ylabel="Lattitude", title="Vehicle Trajectory")
    ax.plot(front_lon, front_lat, "r")
    ax.plot(rear_lon, rear_lat, "b")
    ax.plot(front_lon, front_lat, "ro")
    ax.plot(rear_lon, rear_lat, "bo")
    plt.legend(["GPS front", "GPS rear"])
    plt.show()

    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
