import random
import matplotlib.pyplot as plt
import numpy as np
from time import sleep
from beamngpy.misc.quat import angle_to_quat

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import AdvancedIMU
from beamngpy import BeamNGpy, Road, Scenario, ScenarioObject, Vehicle, MeshRoad, ProceduralBump, ProceduralCylinder

def main():
    random.seed(1703)
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
    # bng.ui.hide_hud()
    bng.scenario.start()
    imu = AdvancedIMU(
        "accel1", bng, vehicle, 
        # From Lua: args.pos / dir / up
        pos=(0.0, 0.2575, 0.504),
        dir=(0.0086, -0.9847, -0.1739),
        up=(-0.0017, -0.1739, 0.9848),

        # # Update intervals
        gfx_update_time=0.00001,
        physics_update_time=0.00001,

        # # Window-based smoothing
        accel_window_width=10.0,
        gyro_window_width=2.0,

        # Optional: Add custom low-pass filters if needed
        # accel_frequency_cutoff=50.0,
        # gyro_frequency_cutoff=20.0,

        # # Sensor behavior flags
        # is_send_immediately=True,
        is_using_gravity=True,
        is_visualised=True,
        is_snapping_desired=False,
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
    # vehicle.control(throttle=throttle, steering=steering, brake=brake)
    # vehicle.ai.set_mode("traffic")
    print("\nDriving around, polling the advanced IMU sensor at regular intervals...")
    
    for i in range(300):
        sleep(0.1)  # Include a small delay between each reading.
        data = imu.poll()  # Fetch the latest readings from the sensor.
        
        if data and len(data) > 0:
            reading = data[0]  # Get the latest reading
            time_data.append(reading['time'])
            acc_x_data.append(reading['accSmooth'][0])  # X-axis (forward/back)
            acc_y_data.append(reading['accSmooth'][1])  # Y-axis (left/right)
            acc_z_data.append(reading['accSmooth'][2])  # Z-axis (up/down)
            
            # Print every 30 samples to show progress
            if i % 30 == 0:
                print(f"Sample {i}: X={reading['accSmooth'][0]:6.3f}, Y={reading['accSmooth'][1]:6.3f}, Z={reading['accSmooth'][2]:6.3f} m/s²")
    
    print(f"\nCollected {len(time_data)} samples for analysis")
    

    
    # Technical recommendation: Plot data over time for all three axes
    print("\nCreating recommended plots...")
    create_analysis_plots(time_data, acc_x_data, acc_y_data, acc_z_data)

    imu.remove()
    bng.ui.show_hud()
    vehicle.ai.set_mode("disabled")

    
    input("Press Enter to exit...")
    bng.disconnect()


def create_analysis_plots(time_data, acc_x_data, acc_y_data, acc_z_data):
    """
    Create recommended plots showing IMU data over time for all three axes
    """
    if not time_data:
        print("No data collected for plotting")
        return
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle("IMU Analysis: Acceleration Data Over Time", fontsize=16)
    
    # Plot X-axis (forward/backward)
    axes[0].plot(time_data, acc_x_data, 'b-', linewidth=1)
    axes[0].set_title('X-axis: Forward/Backward Acceleration')
    axes[0].set_ylabel('Acceleration (m/s²)')
    axes[0].grid(True, alpha=0.3)
    std_x = np.std(acc_x_data)
    axes[0].text(0.02, 0.95, f'Std Dev: {std_x:.3f} m/s²', transform=axes[0].transAxes,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.8))
    
    # Plot Y-axis (left/right)
    axes[1].plot(time_data, acc_y_data, 'g-', linewidth=1)
    axes[1].set_title('Y-axis: Left/Right Acceleration')
    axes[1].set_ylabel('Acceleration (m/s²)')
    axes[1].grid(True, alpha=0.3)
    std_y = np.std(acc_y_data)
    axes[1].text(0.02, 0.95, f'Std Dev: {std_y:.3f} m/s²', transform=axes[1].transAxes,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.8))
    
    # Plot Z-axis (up/down)
    axes[2].plot(time_data, acc_z_data, 'r-', linewidth=1)
    axes[2].set_title('Z-axis: Up/Down Acceleration')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Acceleration (m/s²)')
    axes[2].grid(True, alpha=0.3)
    std_z = np.std(acc_z_data)
    axes[2].text(0.02, 0.95, f'Std Dev: {std_z:.3f} m/s²', transform=axes[2].transAxes,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcoral", alpha=0.8))
    
    plt.tight_layout()
    plt.savefig('imu_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()
    



if __name__ == "__main__":
    main()
