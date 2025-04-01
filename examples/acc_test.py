from time import sleep

import matplotlib.pyplot as plt

from beamngpy import BeamNGpy, Scenario, Vehicle, angle_to_quat, set_up_simple_logging
from beamngpy.sensors import IdealRadar


def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    bng.open(None, "-gfx", "dx11")

    # Create the vehicles.
    vehicle1 = Vehicle("ego_vehicle", model="etk800", licence="EGO", color="Red")
    vehicle2 = Vehicle("other_vehicle", model="etk800", licence="OTHER", color="Blue")

    # Create a scenario.
    scenario = Scenario("italy", "ACCtest", description="ACC1")

    # Add the vehicles to the scenario.
    scenario.add_vehicle(
        vehicle1,
        pos=(-350.76326635601, 1169.0963008935, 168.6981158547),
        rot_quat=angle_to_quat((0, 0, 90)),
    )
    scenario.add_vehicle(
        vehicle2,
        pos=(-365.56326635601, 1169.0963008935, 168.69811585470),
        rot_quat=angle_to_quat((0, 0, 90)),
    )
    scenario.make(bng)

    # Set simulator to 60hz temporal resolution
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # set the ideal sensor
    idealRADAR1 = IdealRadar(
        "idealRADAR1",
        bng,
        vehicle1,
        is_send_immediately=False,
        physics_update_time=0.01,
    )
    vehicle1.sensors.poll()
    vehicle1.sensors["state"]
    print("Collecting ideal RADAR readings...")
    vehicle2.ai.set_mode("traffic")
    vehicle1.ai.set_mode("traffic")
    sleep(5)
    vehicle1.acc.start(vehicle1.vid, 13.0, True)  # ego vehicle should have ACC
    print("Started ACC...")

    sleep(3.0)
    (
        listrelDistX,
        listrelDistY,
        listrelVelX,
        listrelVelY,
        listrelAccX,
        listrelAccY,
        listtime,
        vel_ego,
        vel_egox,
        vel_egoy,
        time_ego,
    ) = ([], [], [], [], [], [], [], [], [], [], [])
    (
        pos_ego,
        posx_ego,
        posy_ego,
        pos_veh2,
        posx_veh2,
        posy_veh2,
        vel_veh2,
        velx_veh2,
        vely_veh2,
    ) = ([], [], [], [], [], [], [], [], [])
    for _ in range(30):
        data_all = idealRADAR1.poll()
        latest_reading = data_all[0]
        data1stVehicle = latest_reading["closestVehicles1"]
        vehicle1.sensors.poll()
        state_ego = vehicle1.sensors["state"]
        vel_ego = state_ego["vel"]
        pos_ego = state_ego["pos"]
        time_ego.append(state_ego["time"])
        print("sensors poll...")
        if data1stVehicle != []:
            listrelDistX.append(data1stVehicle["relDistX"])
            listrelDistY.append(data1stVehicle["relDistY"])
            listrelVelX.append(data1stVehicle["relVelX"])
            listrelVelY.append(data1stVehicle["relVelY"])
            listrelAccX.append(data1stVehicle["relAccX"])
            listrelAccY.append(data1stVehicle["relAccY"])

            vel_veh2 = data1stVehicle["velBB"]
            pos_veh2 = data1stVehicle["positionB"]
            listtime.append(latest_reading["time"])
            vel_egox.append(vel_ego[0])
            vel_egoy.append(vel_ego[1])
            posx_ego.append(pos_ego[0])
            posy_ego.append(pos_ego[1])

            px = pos_veh2["x"]
            posx_veh2.append(px)
            py = pos_veh2["y"]
            posy_veh2.append(py)

            velx_veh2.append(vel_veh2["x"])
            vely_veh2.append(vel_veh2["y"])
        sleep(1.0)
    bng.ui.show_hud()
    bng.disconnect()

    # Plot the data
    print("data plot...")

    fig, ax = plt.subplots(6, 1, figsize=(9, 3), sharey=False)
    ax[0].set(
        xlabel="time (s)", ylabel="distance (m)", title="longitudinal relative distance"
    )
    ax[0].plot(listtime, listrelDistX, "r")

    ax[1].set(
        xlabel="time (s)", ylabel="position - ego (m)", title="longitudinal - position"
    )
    ax[1].plot(time_ego, posx_ego, "r")

    ax[2].set(
        xlabel="time (s)", ylabel="position - 2nd (m)", title="longitudinal - position"
    )
    ax[2].plot(listtime, posx_veh2, "r")

    ax[3].set(
        xlabel="time (s)",
        ylabel="velocity (m/s)",
        title="longitudinal relative velocity",
    )
    ax[3].plot(listtime, listrelVelX, "r")

    ax[4].set(
        xlabel="time (s)", ylabel="velocity - ego (m/s)", title="longitudinal velocity"
    )
    ax[4].plot(time_ego, vel_egox, "r")

    ax[5].set(
        xlabel="time (s)", ylabel="velocity - 2nd (m/s)", title="longitudinal velocity"
    )
    ax[5].plot(listtime, velx_veh2, "r")

    plt.show()


if __name__ == "__main__":
    main()
