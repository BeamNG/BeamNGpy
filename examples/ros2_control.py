from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging


def main():
    set_up_simple_logging()

    bng = BeamNGpy("localhost", 25252).open(None, "-disable-sandbox")

    scenario = Scenario(
        "smallgrid",
        "ros2_control",
        description="Control the BeamNG ROS 2 integration from BeamNGpy",
    )
    vehicle = Vehicle("ego_vehicle", model="etk800", license="ROS2", color="Blue")
    scenario.add_vehicle(vehicle, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1))
    scenario.make(bng)

    bng.scenario.load(scenario)
    bng.scenario.start()

    try:
        # The library must be available in Bin64 / BinLinux, or pass a full path here.
        bng.ros2.load_bridge("ros2_beamng_lyrical.dll")

        electrics_id = bng.ros2.add_sensor(
            vehicle,
            "Electrics",
            fields=["throttle", "brake", "steering", "wheelspeed", "rpm", "gear"],
        )
        lidar_id = bng.ros2.add_sensor(
            vehicle,
            "Lidar",
            position=[0.5, -0.3, 1.8],
            direction=[1, 0, 0],
            up=[0, 0, 1],
            config={"update_rate_hz": 20},
        )

        bng.ros2.start(vehicle)

        print("ROS 2 topics:")
        print(bng.ros2.get_topic_name(vehicle, electrics_id))
        print(bng.ros2.get_topic_name(vehicle, lidar_id))
        print("Status:")
        print(bng.ros2.get_status(vehicle))

    finally:
        bng.ros2.stop(vehicle)
        bng.disconnect()


if __name__ == "__main__":
    main()
