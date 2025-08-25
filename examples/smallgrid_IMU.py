import matplotlib.pyplot as plt
from time import sleep
from beamngpy.misc.quat import angle_to_quat

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import AdvancedIMU
from beamngpy import BeamNGpy, Scenario, Vehicle, ProceduralCylinder

def main():
    set_up_simple_logging()

    beamng = BeamNGpy("localhost", 25252)
    bng = beamng.open(launch=True)

    scenario = Scenario(
        "smallgrid",
        "advanced_IMU_demo",
        description="Spanning the map with an advanced IMU sensor",
    )
    vehicle = Vehicle("ego_vehicle", model="etk800", license="RED", color="Red")
    scenario.add_vehicle(vehicle)

    # Using a gentle cylinder as a bump - much safer than ProceduralBump
    cylinder_bump = ProceduralCylinder(
        pos=(0, -100, 0.05),  # Slightly elevated
        radius=4.0,  # Much wider - covers full vehicle width
        height=0.1,  # Low height to prevent damage
        name='cylinder_bump',
        rot_quat=angle_to_quat((0, 0, 0))
    )
    scenario.add_procedural_mesh(cylinder_bump)
    print("Added wide cylinder bump for IMU testing")
    scenario.make(bng)

    bng.settings.set_deterministic(60)  # Set simulator to 60hz temporal resolution
    bng.scenario.load(scenario)
    bng.scenario.start()

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
        smoother_strength=1.0,

        # Sensor behavior flags
        is_using_gravity=True,
        is_visualised=True,
        is_snapping_desired=True,
        is_force_inside_triangle=False,
        is_allow_wheel_nodes=False
    )

    # Technical recommendation: Plot data over time for all three axes and look for patterns
    print("\nAdvanced analysis: Collecting data for plotting and pattern recognition...")
    print("Looking for:")
    print("- Vehicle vibration modes")
    print("- Road surface irregularity effects")
    print("- Noise spikes from mechanical vibration")

    # Data collection arrays for plotting recommendation
    time_data = []
    acc_x_data = []
    acc_y_data = []
    acc_z_data = []

    vehicle.control(throttle=0.05, steering=0, brake=0)
    print("\nAccelerating over bump, polling the advanced IMU sensor at regular intervals...")

    # Collect data
    for i in range(2000):
        sleep(0.01)  # Include a small delay between each reading.
        data = imu.poll()  # Fetch the latest readings from the sensor.

        if data and len(data) > 0:
            reading = data[0]  # Get the latest reading
            time_data.append(reading['time'])
            acc_x_data.append(reading['accSmooth'][0])  # forward/back
            acc_y_data.append(reading['accSmooth'][1])  # up/down
            acc_z_data.append(reading['accSmooth'][2])  # right/left

            # Print every 30 samples to show progress
            if i % 30 == 0:
                print(f"Sample {i}: forward/back={reading['accSmooth'][0]:6.3f}, up/down={reading['accSmooth'][1]:6.3f}, right/left={reading['accSmooth'][2]:6.3f} m/s²")

    print(f"\nCollected {len(time_data)} samples for analysis")

    # Technical recommendation: Plot data over time for all three axes
    print("\nCreating recommended plots...")
    create_analysis_plots(time_data, acc_x_data, acc_y_data, acc_z_data)

    imu.remove()
    input("Press Enter to exit...")
    bng.disconnect()


def create_analysis_plots(time_data, acc_x_data, acc_y_data, acc_z_data):
    """
    Create recommended plots showing IMU data over time for all three axes
    """
    if not time_data:
        print("No data collected for plotting")
        return

    # Normalize time data to start at 0
    time_offset = min(time_data)
    time_data = [t - time_offset for t in time_data]

    # Calculate padding for y-axis limits
    all_data = acc_x_data + acc_y_data + acc_z_data
    overall_min = min(all_data)
    overall_max = max(all_data)
    padding = (overall_max - overall_min) * 0.02
    y_min = overall_min - padding
    y_max = overall_max + padding

    fig, axes = plt.subplots(3, 1, figsize=(14, 12))

    # Plot forward/backward acceleration
    axes[0].plot(time_data, acc_x_data,
                 color='blue', linewidth=2)
    axes[0].set_title('Forward/Backward Acceleration')
    axes[0].set_ylabel('Acceleration (m/s²)')
    axes[0].set_ylim(y_min, y_max)
    axes[0].grid(True, alpha=0.3, which='major')
    axes[0].grid(True, alpha=0.15, which='minor')
    axes[0].minorticks_on()

    # Plot down/up acceleration
    axes[1].plot(time_data, acc_y_data,
                 color='green', linewidth=2)
    axes[1].set_title('Up/Down Acceleration')
    axes[1].set_ylabel('Acceleration (m/s²)')
    axes[1].set_ylim(y_min, y_max)
    axes[1].grid(True, alpha=0.3, which='major')
    axes[1].grid(True, alpha=0.15, which='minor')
    axes[1].minorticks_on()

    # Plot right/left acceleration
    axes[2].plot(time_data, acc_z_data,
                 color='red', linewidth=2)
    axes[2].set_title('Right/Left Acceleration')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Acceleration (m/s²)')
    axes[2].set_ylim(y_min, y_max)
    axes[2].grid(True, alpha=0.3, which='major')
    axes[2].grid(True, alpha=0.15, which='minor')
    axes[2].minorticks_on()

    plt.show()


# This script demonstrates IMU data collection from an accelerating vehicle using a single
# 2000Hz IMU sensor and creates plots showing acceleration data over time for all three axes.
if __name__ == "__main__":
    main()
