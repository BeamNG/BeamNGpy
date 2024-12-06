from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging, angle_to_quat
from beamngpy.sensors import IdealRadar

import matplotlib.pyplot as plt


def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    bng.open(launch=True)

    # Create the vehicles.
    vehicle1 = Vehicle("ego_vehicle", model="etki", licence="ego vehicle", color="Red")
    vehicle2 = Vehicle("other_vehicle", model="etki", licence="vehicle2", color="Blue")

    # Create a scenario.
    scenario = Scenario("italy", "IdealRADAR_plots", description="ideal RADAR")

    # Add the vehicles to the scenario.
    scenario.add_vehicle(
        vehicle1,
        pos=(-353.76326635601, 1169.0963008935, 168.6981158547),
        rot_quat=angle_to_quat((0, 0, 90)),
    )
    scenario.add_vehicle(
        vehicle2,
        pos=(-361.76326635601, 1169.0963008935, 168.69811585470),
        rot_quat=angle_to_quat((0, 0, 90)),
    )
    scenario.make(bng)

    # Set simulator to 60hz temporal resolution
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # set the sensor
    idealRADAR1 = IdealRadar(
        "idealRADAR1",
        bng,
        vehicle1,
        is_send_immediately=False,
        physics_update_time=0.01,
    )

    print("Collecting ideal RADAR readings...")
    vehicle2.ai.set_mode("traffic")
    vehicle1.ai.set_mode("follow")

    sleep(3.0)
    (
        listrelDistX,
        listrelDistY,
        listrelVelX,
        listrelVelY,
        listrelAccX,
        listrelAccY,
        listtime,
    ) = ([], [], [], [], [], [], [])

    for _ in range(60):
        data_all = idealRADAR1.poll()
        latest_reading = data_all[0]
        data1stVehicle = latest_reading["closestVehicles1"]
        if data1stVehicle != []:
            listrelDistX.append(data1stVehicle["relDistX"])
            listrelDistY.append(data1stVehicle["relDistY"])
            listrelVelX.append(data1stVehicle["relVelX"])
            listrelVelY.append(data1stVehicle["relVelY"])
            listrelAccX.append(data1stVehicle["relAccX"])
            listrelAccY.append(data1stVehicle["relAccY"])
            listtime.append(latest_reading["time"])
        sleep(1.0)
    bng.disconnect()

    # Plot the data
    fig, ax = plt.subplots(3, 2, figsize=(9, 3), sharey=False)
    ax[0, 0].set(
        xlabel="time (s)", ylabel="distance (m)", title="longitudinal relative distance"
    )
    ax[0, 0].plot(listtime, listrelDistX, "r")

    ax[0, 1].set(
        xlabel="time (s)", ylabel="distance (m)", title="lateral relative distance"
    )
    ax[0, 1].plot(listtime, listrelDistY, "b")

    ax[1, 0].set(
        xlabel="time (s)",
        ylabel="velocity (m/s)",
        title="longitudinal relative velocity",
    )
    ax[1, 0].plot(listtime, listrelVelX, "r")

    ax[1, 1].set(
        xlabel="time (s)", ylabel="velocity (m/s)", title="lateral relative velocity"
    )
    ax[1, 1].plot(listtime, listrelVelY, "b")

    ax[2, 0].set(
        xlabel="time (s)",
        ylabel="acceleration (m/s2)",
        title="longitudinal relative acceleration",
    )
    ax[2, 0].plot(listtime, listrelAccX, "r")

    ax[2, 1].set(xlabel="time (s)", title="lateral relative acceleration")
    ax[2, 1].plot(listtime, listrelAccY, "b")

    plt.show()


if __name__ == "__main__":
    main()
