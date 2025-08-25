from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import AdvancedIMU

from smallgrid_IMU import create_analysis_plots

def main():
    set_up_simple_logging()

    beamng = BeamNGpy("localhost", 25252)
    bng = beamng.open(launch=True)

    scenario = Scenario(
        "west_coast_usa",
        "advanced_IMU_demo",
        description="Spanning the map with an advanced IMU sensor",
    )
    vehicle = Vehicle("ego_vehicle", model="etk800", license="RED", color="Red")

    scenario.add_vehicle(
        vehicle, pos=(-717.121, 101, 118.675), rot_quat=(0, 0, 0.3826834, 0.9238795)
    )
    scenario.make(bng)

    bng.settings.set_deterministic(60)  # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.scenario.start()

    # NOTE: Create sensor after scenario has started.
    imu = AdvancedIMU(
        "accel1", bng, vehicle,
        # From Lua: args.pos / dir / up
        pos=(0, 0, 5),  # placing the IMU on vehicle roof (with snapping)
        dir=(0, -1, 0),
        up=(-0, 0, 1),

        # Update intervals - set to 2000Hz
        gfx_update_time=0.0005,
        physics_update_time=0.0005,

        # Smoothing strength
        smoother_strength=3.0,

        # Sensor behavior flags
        is_using_gravity=False,
        is_visualised=True,
        is_snapping_desired=True,
        is_force_inside_triangle=False,
        is_allow_wheel_nodes=False
    )

    vehicle.ai.set_mode("traffic")

    print("Driving around, polling the advanced IMU sensor in bulk after a while...")
    imu.poll()
    sleep(40)

    data = imu.poll()

    time_data = [data[key]['time'] for key in data.keys()]
    acc_x_data = [data[key]['accSmooth'][0] for key in data.keys()]  # X-axis (forward/back)
    acc_y_data = [data[key]['accSmooth'][1] for key in data.keys()]  # Y-axis (up/down)
    acc_z_data = [data[key]['accSmooth'][2] for key in data.keys()]  # Z-axis (left/right)

    create_analysis_plots(time_data, acc_x_data, acc_y_data, acc_z_data)

    imu.remove()
    vehicle.ai.set_mode("disabled")
    input("Press Enter to exit...")
    bng.disconnect()

# This script demonstrates bulk IMU data collection from a vehicle driving on the west coast map,
# collecting accelerometer data over an extended period and creating analysis plots. See also smallgrid_IMU.py
# for an example of periodic data collection.
if __name__ == "__main__":
    main()
