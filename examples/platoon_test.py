from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging, angle_to_quat
from beamngpy.sensors import IdealRadar, State
from beamngpy.api.beamng.platoon import PlatoonApi


import matplotlib.pyplot as plt
import math
import csv
import os


def main():
    set_up_simple_logging()

    # Start up the simulator.

    bng = BeamNGpy("localhost", 64256)
    bng.open()
    platoon_api = PlatoonApi(bng)

    # Create the vehicles.
    vehicle1 = Vehicle(
        "ego_vehicle", model="pickup", licence="ego vehicle", color="Red"
    )
    vehicle2 = Vehicle(
        "relay_vehicle1", model="pickup", licence="vehicle2", color="Blue"
    )
    vehicle3 = Vehicle(
        "relay_vehicle2", model="pickup", licence="vehicle3", color="Green"
    )

    vehicle7 = Vehicle("leader3", model="bastion", licence="vehicle7", color="Orange")
    vehicle8 = Vehicle(
        "relayVehicle31", model="bastion", licence="vehicle8", color="Purple"
    )
    vehicle9 = Vehicle(
        "relayVehicle32", model="bastion", licence="vehicle9", color="Yellow"
    )
    vehicle10 = Vehicle(
        "relayVehicle33", model="bastion", licence="vehicle10", color="Black"
    )

    # Create a scenario.
    scenario = Scenario(
        "italy", "platoontest", description="platoon"
    )  # what is platoontest

    scenario.add_vehicle(
        vehicle1,
        pos=(252.56326635601, 1205.0963008935, 169.6981158547),
        rot_quat=angle_to_quat((0, 0, 90)),
    )
    scenario.add_vehicle(
        vehicle2,
        pos=(262.56326635601, 1205.7963008935, 169.69811585470),
        rot_quat=angle_to_quat((0, 0, 90)),
    )
    scenario.add_vehicle(
        vehicle3,
        pos=(270.76326635601, 1206.5963008935, 169.69811585470),
        rot_quat=angle_to_quat((0, 0, 90)),
    )

    scenario.add_vehicle(
        vehicle7,
        pos=(190.56326635601, 1202.0963008935, 169.6981158547),
        rot_quat=angle_to_quat((0, 0, 90)),
    )
    scenario.add_vehicle(
        vehicle8,
        pos=(200.56326635601, 1202.0963008935, 169.69811585470),
        rot_quat=angle_to_quat((0, 0, 90)),
    )
    scenario.add_vehicle(
        vehicle9,
        pos=(212.76326635601, 1203.0963008935, 169.69811585470),
        rot_quat=angle_to_quat((0, 0, 90)),
    )
    scenario.add_vehicle(
        vehicle10,
        pos=(222.76326635601, 1203.0963008935, 169.69811585470),
        rot_quat=angle_to_quat((0, 0, 90)),
    )

    scenario.make(bng)

    # Set simulator to 60hz temporal resolution
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.ui.show_hud()
    bng.scenario.start()

    vehicle1.connect(bng)
    vehicle2.connect(bng)
    vehicle3.connect(bng)

    vehicle7.connect(bng)
    vehicle8.connect(bng)
    vehicle9.connect(bng)
    vehicle10.connect(bng)

    sleep(5)

    speed1 = 20.0
    speed3 = 20.0
    trafficMode = 0
    platoon_api.load(
        vehicle1.vid, vehicle2.vid, vehicle3.vid, None, speed1, True
    )  # second vehicle joins platoon
    platoon_api.load(
        vehicle7.vid, vehicle8.vid, vehicle9.vid, vehicle10.vid, speed3, True
    )

    oldTime = 0

    time = []

    originV1x = 0
    originV1y = 0
    originV1z = 0

    originV2x = 0
    originV2y = 0
    originV2z = 0

    originV3x = 0
    originV3y = 0
    originV3z = 0

    positionV1 = []
    positionV2 = []
    positionV3 = []

    velocityV1 = []
    velocityV2 = []
    velocityV3 = []

    accelV1 = []
    accelV2 = []
    accelV3 = []

    diffPosV1V2 = []
    diffPosV2V3 = []

    originV7x = 0
    originV8y = 0
    originV9z = 0
    originV10z = 0

    originV7x = 0
    originV8y = 0
    originV9z = 0
    originV10z = 0

    originV7x = 0
    originV8y = 0
    originV9z = 0
    originV10z = 0

    positionV7 = []
    positionV8 = []
    positionV9 = []
    positionV10 = []

    velocityV7 = []
    velocityV8 = []
    velocityV9 = []
    velocityV10 = []

    accelV7 = []
    accelV8 = []
    accelV9 = []
    accelV10 = []

    diffPosV7V8 = []
    diffPosV8V9 = []
    diffPosV9V10 = []

    sleep(3.0)
    for i in range(60):

        vehicle1.sensors.poll()
        vehicle2.sensors.poll()
        vehicle3.sensors.poll()

        vehicle7.sensors.poll()
        vehicle8.sensors.poll()
        vehicle9.sensors.poll()
        vehicle10.sensors.poll()

        data1 = vehicle1.sensors["state"]
        data2 = vehicle2.sensors["state"]
        data3 = vehicle3.sensors["state"]

        data7 = vehicle7.sensors["state"]
        data8 = vehicle8.sensors["state"]
        data9 = vehicle9.sensors["state"]
        data10 = vehicle10.sensors["state"]

        time.append(data1["time"])

        pos1x, pos1y, pos1z = data1["pos"]

        if len(positionV1) == 0:
            originV1x = pos1x
            originV1y = pos1y
            originV1z = pos1z

        pos1 = math.sqrt(
            (pos1x - originV1x) ** 2
            + (pos1y - originV1y) ** 2
            + (pos1z - originV1z) ** 2
        )
        positionV1.append(pos1)

        pos2x, pos2y, pos2z = data2["pos"]

        if len(positionV2) == 0:
            originV2x = pos2x
            originV2y = pos2y
            originV2z = pos2z

        pos2 = math.sqrt(
            (pos2x - originV2x) ** 2
            + (pos2y - originV2y) ** 2
            + (pos2z - originV2z) ** 2
        )

        positionV2.append(pos2)

        pos3x, pos3y, pos3z = data3["pos"]

        if len(positionV3) == 0:
            originV3x = pos3x
            originV3y = pos3y
            originV3z = pos3z

        pos3 = math.sqrt(
            (pos3x - originV3x) ** 2
            + (pos3y - originV3y) ** 2
            + (pos3z - originV3z) ** 2
        )

        positionV3.append(pos3)

        pos7x, pos7y, pos7z = data7["pos"]

        if len(positionV7) == 0:
            originV7x = pos7x
            originV7y = pos7y
            originV7z = pos7z

        pos7 = math.sqrt(
            (pos7x - originV7x) ** 2
            + (pos7y - originV7y) ** 2
            + (pos7z - originV7z) ** 2
        )

        positionV7.append(pos7)

        pos8x, pos8y, pos8z = data8["pos"]

        if len(positionV8) == 0:
            originV8x = pos8x
            originV8y = pos8y
            originV8z = pos8z

        pos8 = math.sqrt(
            (pos8x - originV8x) ** 2
            + (pos8y - originV8y) ** 2
            + (pos8z - originV8z) ** 2
        )

        positionV8.append(pos8)

        pos9x, pos9y, pos9z = data9["pos"]

        if len(positionV9) == 0:
            originV9x = pos9x
            originV9y = pos9y
            originV9z = pos9z

        pos9 = math.sqrt(
            (pos9x - originV9x) ** 2
            + (pos9y - originV9y) ** 2
            + (pos9z - originV9z) ** 2
        )

        positionV9.append(pos9)

        pos10x, pos10y, pos10z = data10["pos"]

        if len(positionV10) == 0:
            originV10x = pos10x
            originV10y = pos10y
            originV10z = pos10z

        pos10 = math.sqrt(
            (pos10x - originV10x) ** 2
            + (pos10y - originV10y) ** 2
            + (pos10z - originV10z) ** 2
        )
        positionV10.append(pos10)

        vel1x, vel1y, vel1z = data1["vel"]
        vel1 = (math.sqrt((vel1x**2 + vel1y**2))) * 3.6
        velocityV1.append(vel1)

        vel2x, vel2y, vel2z = data2["vel"]
        vel2 = math.sqrt(((vel2x**2 + vel2y**2))) * 3.6
        velocityV2.append(vel2)

        vel3x, vel3y, vel3z = data3["vel"]
        vel3 = (math.sqrt((vel3x**2 + vel3y**2))) * 3.6
        velocityV3.append(vel3)

        vel7x, vel7y, vel7z = data7["vel"]
        vel7 = (math.sqrt((vel7x**2 + vel7y**2))) * 3.6
        velocityV7.append(vel7)

        vel8x, vel8y, vel8z = data8["vel"]
        vel8 = (math.sqrt((vel8x**2 + vel8y**2))) * 3.6
        velocityV8.append(vel8)

        vel9x, vel9y, vel9z = data9["vel"]
        vel9 = (math.sqrt((vel9x**2 + vel9y**2))) * 3.6
        velocityV9.append(vel9)

        vel10x, vel10y, vel10z = data10["vel"]
        vel10 = (math.sqrt((vel10x**2 + vel10y**2))) * 3.6
        velocityV10.append(vel10)

        if oldTime != 0:

            accelV1.append((vel1 / 3.6 - oldVelV1 / 3.6) / (time[i] - oldTime))
            accelV2.append((vel2 / 3.6 - oldVelV2 / 3.6) / (time[i] - oldTime))
            accelV3.append((vel3 / 3.6 - oldVelV3 / 3.6) / (time[i] - oldTime))

            accelV7.append((vel7 / 3.6 - oldVelV7 / 3.6) / (time[i] - oldTime))
            accelV8.append((vel8 / 3.6 - oldVelV8 / 3.6) / (time[i] - oldTime))
            accelV9.append((vel9 / 3.6 - oldVelV9 / 3.6) / (time[i] - oldTime))
            accelV10.append((vel10 / 3.6 - oldVelV10 / 3.6) / (time[i] - oldTime))

        else:
            accelV1.append(0)
            accelV2.append(0)
            accelV3.append(0)

            accelV7.append(0)
            accelV8.append(0)
            accelV9.append(0)
            accelV10.append(0)

        oldTime = time[i]
        oldVelV1 = vel1
        oldVelV2 = vel2
        oldVelV3 = vel3

        oldVelV7 = vel7
        oldVelV8 = vel8
        oldVelV9 = vel9
        oldVelV10 = vel10

        diffPosV1V2.append(math.sqrt((pos1x - pos2x) ** 2 + (pos1y - pos2y) ** 2) - 4.9)
        diffPosV2V3.append(math.sqrt((pos2x - pos3x) ** 2 + (pos2y - pos3y) ** 2) - 4.9)

        diffPosV7V8.append(math.sqrt((pos7x - pos8x) ** 2 + (pos7y - pos8y) ** 2) - 5)
        diffPosV8V9.append(math.sqrt((pos8x - pos9x) ** 2 + (pos8y - pos9y) ** 2) - 5)
        diffPosV9V10.append(
            math.sqrt((pos9x - pos10x) ** 2 + (pos9y - pos10y) ** 2) - 5
        )

        bng.step(20)
    bng.close()

    fig, ax = plt.subplots(2, 2, figsize=(9, 6), sharey=False)

    ax[0, 0].set(
        xlabel="Simulation Time (s)", ylabel="Speed (Km/h)", title="Speed vs. Time"
    )
    ax[0, 0].plot(time, velocityV1, "r", label="Vehicle 1 Velocity (Km/h)")
    ax[0, 0].plot(time, velocityV2, "b", label="Vehicle 2 Velocity (Km/h)")
    ax[0, 0].plot(time, velocityV3, "g", label="Vehicle 3 Velocity (Km/h)")
    ax[0, 0].legend()

    ax[0, 1].set(
        xlabel="Simulation Time (s)",
        ylabel="Vehicle Position (m)",
        title="Position vs. Time",
    )
    ax[0, 1].plot(time, positionV1, "r", label="Vehicle 1 Position")
    ax[0, 1].plot(time, positionV2, "b", label="Vehicle 2 Position")
    ax[0, 1].plot(time, positionV3, "g", label="Vehicle 3 Position")
    ax[0, 1].legend()

    ax[1, 0].set(
        xlabel="Simulation Time (s)",
        ylabel=" Position difference between vehicles(m)",
        title="interVehicular Distance",
    )
    ax[1, 0].plot(time, diffPosV1V2, "m", label="Leader and follower 1")
    ax[1, 0].plot(time, diffPosV2V3, "c", label="Follower 1 and follower2")
    ax[1, 0].legend()

    ax[1, 1].set(
        xlabel="Simulation Time (s)",
        ylabel="Vehicle Acceleration (m/s^2)",
        title="Acceleration vs. Time",
    )
    ax[1, 1].plot(time, accelV1, "r", label="Vehicle 1 acceleration")
    ax[1, 1].plot(time, accelV2, "b", label="Vehicle 2 acceleration")
    ax[1, 1].plot(time, accelV3, "g", label="Vehicle 3 acceleration")
    ax[1, 1].legend()

    plt.tight_layout()
    plt.show()

    fig, ax = plt.subplots(2, 2, figsize=(9, 6), sharey=False)

    ax[0, 0].set(
        xlabel="Simulation Time (s)", ylabel="Speed (Km/h)", title="Speed vs. Time"
    )
    ax[0, 0].plot(time, velocityV7, "r", label="Vehicle 1 Velocity (Km/h)")
    ax[0, 0].plot(time, velocityV8, "b", label="Vehicle 2 Velocity (Km/h)")
    ax[0, 0].plot(time, velocityV9, "g", label="Vehicle 3 Velocity (Km/h)")
    ax[0, 0].plot(time, velocityV10, "purple", label="Vehicle 4 Velocity (Km/h)")
    ax[0, 0].legend()

    ax[0, 1].set(
        xlabel="Simulation Time (s)",
        ylabel="Vehicle Position (m)",
        title="Position vs. Time",
    )
    ax[0, 1].plot(time, positionV7, "r", label="Vehicle 1 Position")
    ax[0, 1].plot(time, positionV8, "b", label="Vehicle 2 Position")
    ax[0, 1].plot(time, positionV9, "g", label="Vehicle 3 Position")
    ax[0, 1].plot(time, positionV10, "purple", label="Vehicle 4 Position")
    ax[0, 1].legend()

    ax[1, 0].set(
        xlabel="Simulation Time (s)",
        ylabel=" Position difference between vehicles(m)",
        title="interVehicular Distance",
    )
    ax[1, 0].plot(time, diffPosV7V8, "r", label="Leader and follower 1")
    ax[1, 0].plot(time, diffPosV8V9, "g", label="Follower 1 and follower2")
    ax[1, 0].plot(time, diffPosV9V10, "b", label="Follower 2 and follower3")
    ax[1, 0].legend()

    ax[1, 1].set(
        xlabel="Simulation Time (s)",
        ylabel="Vehicle Acceleration (m/s^2)",
        title="Acceleration vs. Time",
    )
    ax[1, 1].plot(time, accelV7, "r", label="Vehicle 1 acceleration")
    ax[1, 1].plot(time, accelV8, "b", label="Vehicle 2 acceleration")
    ax[1, 1].plot(time, accelV9, "g", label="Vehicle 3 acceleration")
    ax[1, 1].plot(time, accelV10, "purple", label="Vehicle 4 acceleration")
    ax[1, 1].legend()

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
