import random
from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Lidar


def main():
    random.seed(1703)
    set_up_simple_logging()

    beamng = BeamNGpy("localhost", 25252)
    bng = beamng.open(launch=True)

    scenario = Scenario(
        "west_coast_usa",
        "LiDAR_demo",
        description="Spanning the map with a LiDAR sensor",
    )

    vehicle = Vehicle("ego_vehicle", model="etk800", license="RED", color="Red")

    scenario.add_vehicle(
        vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795)
    )
    scenario.make(bng)

    bng.settings.set_deterministic(60)  # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.ui.hide_hud()
    bng.scenario.start()

    # NOTE: Create sensor after scenario has started.

    # Unsilence the demo which you want to run:
    lidar = Lidar(
        "lidar1",
        bng,
        vehicle,
        requested_update_time=0.01,
        is_using_shared_memory=True,
        is_360_mode=True,  # [DEMO: DEFAULT - 360 MODE].  Uses shared memory.
    )

    # lidar = Lidar('lidar1', bng, vehicle, requested_update_time=0.01, is_using_shared_memory=True,      # [DEMO: ROTATE MODE].  Uses shared memory.
    #              is_rotate_mode=True, is_360_mode=False, horizontal_angle=60, frequency=3)

    # lidar = Lidar('lidar1', bng, vehicle, requested_update_time=0.01, is_using_shared_memory=True,      # [DEMO: 360 MODE, WITH UPWARDS TILT].  Uses shared memory.
    #              is_360_mode=True, dir=(0, -1, 0.5))

    # Lidar = Lidar('lidar1', bng, vehicle, requested_update_time=0.01, is_using_shared_memory=True,      # [DEMO: STATIC MODE, WITH 120 DEGREE APERTURE].  Uses shared memory.
    #              is_rotate_mode=False, is_360_mode=False, horizontal_angle=120)

    # lidar = Lidar('lidar1', bng, vehicle, requested_update_time=0.01, is_using_shared_memory=False)    # Does NOT use shared memory.  Sends data through lua socket instead.

    vehicle.ai.set_mode("traffic")
    print("Driving around, polling the LiDAR sensor every 5 seconds...")
    for i in range(6):
        sleep(5)
        readings_data = (
            lidar.poll()
        )  # Fetch the latest readings from the sensor, over either shared memory or the lua socket.
        print(
            "LiDAR Point Cloud Data after ",
            i * 5,
            " seconds: \n",
            readings_data["pointCloud"][0:10],
        )  # The first 10 points from LiDAR point cloud data.
        # print('LiDAR Colour Data after ', i * 5, ' seconds: ', readings_data['colours'][0:10])             # The colour data (corresponds to each point).

    lidar.remove()
    vehicle.ai.set_mode("disabled")
    bng.ui.show_hud()
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
