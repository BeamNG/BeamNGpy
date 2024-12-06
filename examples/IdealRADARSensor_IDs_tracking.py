from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging, angle_to_quat
from beamngpy.sensors import IdealRadar

import matplotlib.pyplot as plt


def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    bng.open(launch=True)

    # Create the vehicles
    vehicleA = Vehicle("ego_vehicle", model="etki", licence="ego vehicle", color="Red")
    vehicleB = Vehicle("other_vehicle1", model="etki", licence="vehicleB", color="Blue")
    vehicleC = Vehicle(
        "other_vehicle2", model="etki", licence="vehicleC", color="Green"
    )
    vehicleD = Vehicle(
        "other_vehicle3", model="etki", licence="vehicleD", color="Black"
    )
    # Create a scenario.
    scenario = Scenario("italy", "IdealRADAR_tracking", description="ideal RADAR")

    # Add the vehicles to the scenario.
    scenario.add_vehicle(
        vehicleA,
        pos=(469.91375179455, 1195.1165732879, 168.42811134217),
        rot_quat=angle_to_quat((0, 0, 270)),
    )
    scenario.add_vehicle(
        vehicleB,
        pos=(481.91375179455, 1195.1165732879, 168.42811134217),
        rot_quat=angle_to_quat((0, 0, 270)),
    )
    scenario.add_vehicle(
        vehicleC,
        pos=(488.91375179455, 1195.1165732879, 168.42811134217),
        rot_quat=angle_to_quat((0, 0, 270)),
    )
    scenario.add_vehicle(
        vehicleD,
        pos=(495.91375179455, 1195.1165732879, 168.42811134217),
        rot_quat=angle_to_quat((0, 0, 270)),
    )

    scenario.make(bng)

    # Set simulator to 60hz temporal resolution
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    idealRADAR1 = IdealRadar(
        "idealRADAR1",
        bng,
        vehicleA,
        is_send_immediately=False,
        physics_update_time=0.01,
    )

    print("Collecting ideal RADAR readings...")
    vehicleB.ai.set_mode("traffic")
    vehicleB.ai.set_speed(150.0, "set")
    vehicleD.ai.set_mode("traffic")
    vehicleA.ai.set_mode("traffic")
    vehicleC.ai.set_target("vehicleB", "flee")
    sleep(3.0)
    listIDs1st, listIDs2nd, listIDs3rd, listtime1st, listtime2nd, listtime3rd = (
        [],
        [],
        [],
        [],
        [],
        [],
    )

    for _ in range(10):
        data_all = idealRADAR1.poll()
        latest_reading = data_all[0]
        data1stVehicle = latest_reading["closestVehicles1"]
        data2ndVehicle = latest_reading["closestVehicles2"]
        data3rdVehicle = latest_reading["closestVehicles3"]
        if data1stVehicle != []:
            listIDs1st.append(data1stVehicle["vehicleID"])
            listtime1st.append(latest_reading["time"])
        if data2ndVehicle != []:
            listIDs2nd.append(data2ndVehicle["vehicleID"])
            listtime2nd.append(latest_reading["time"])
        if data3rdVehicle != []:
            listIDs3rd.append(data3rdVehicle["vehicleID"])
            listtime3rd.append(latest_reading["time"])
        sleep(1.0)

    bng.disconnect()

    # Plot the data
    fig, ax = plt.subplots(1, 3, figsize=(9, 3), sharey=False)
    ax[0].set(
        xlabel="time (s)", ylabel="ID of 1st vehicle", title="1st closest vehicle"
    )
    ax[0].step(listtime1st, listIDs1st, "r")  # step-wise plot
    # Add label of the y-value to each step of the plot
    previousyi = 0
    for _, (xi, yi) in enumerate(zip(listtime1st, listIDs1st)):
        if yi != previousyi:
            ax[0].text(xi, yi, f"ID = {int(yi)}", ha="left", va="bottom")
            previousyi = yi

    ax[1].set(
        xlabel="time (s)", ylabel="ID of 2nd vehicle", title="2nd closest vehicle"
    )
    ax[1].step(listtime2nd, listIDs2nd, "b")
    previousyi = 0
    for _, (xi, yi) in enumerate(zip(listtime2nd, listIDs2nd)):
        if yi != previousyi:
            ax[1].text(xi, yi, f"ID = {int(yi)}", ha="left", va="bottom")
            previousyi = yi

    ax[2].set(
        xlabel="time (s)", ylabel="ID of 3nd vehicle", title="3rd closest vehicle"
    )
    ax[2].step(listtime3rd, listIDs3rd, "g")
    previousyi = 0
    for _, (xi, yi) in enumerate(zip(listtime3rd, listIDs3rd)):
        if yi != previousyi:
            ax[2].text(xi, yi, f"ID = {int(yi)}", ha="left", va="bottom")
            previousyi = yi
    plt.show()


if __name__ == "__main__":
    main()
