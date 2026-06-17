from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging, angle_to_quat

from time import sleep
import matplotlib.pyplot as plt
import math


def main():
    set_up_simple_logging()

    # Start up the simulator.
    bng = BeamNGpy("localhost", 25252)
    bng.open()

    # Define the platoons.
    platoons = [
        {
            "vehicles": [
                ("leader3", "bastion", "vehicle7", "Orange", (190.56, 1202.09, 169.69)),
                ("relay_vehicle31", "bastion", "vehicle8", "Purple", (200.56, 1202.09, 169.69)),
                ("relay_vehicle32", "bastion", "vehicle9", "Yellow", (212.76, 1203.09, 169.69)),
                ("relay_vehicle33", "bastion", "vehicle10", "Black", (222.76, 1203.09, 169.69)),
            ],
            "speed": 20.0,
            "gap": 5,
            "plot_colors": ["r", "b", "g", "purple"],
            "diff_colors": ["r", "g", "b"],
            "diff_labels": [
                "Leader and follower 1",
                "Follower 1 and follower2",
                "Follower 2 and follower3",
            ],
        },
        {
            "vehicles": [
                ("ego_vehicle", "pickup", "ego vehicle", "Red", (252.56, 1205.09, 169.69)),
                ("relay_vehicle1", "pickup", "vehicle2", "Blue", (262.56, 1205.79, 169.69)),
                ("relay_vehicle2", "pickup", "vehicle3", "Green", (270.76, 1206.59, 169.69)),
            ],
            "speed": 20.0,
            "gap": 4.9,
            "plot_colors": ["r", "b", "g"],
            "diff_colors": ["m", "c"],
            "diff_labels": ["Leader and follower 1", "Follower 1 and follower2"],
        },
    ]

    scenario = Scenario(
        "italy", "platooning", description="platoon example"
    )

    # Add the vehicles to the scenario.
    vehicles_by_platoon = []
    for platoon in platoons:
        vehicles = []
        for name, model, license, color, pos in platoon["vehicles"]:
            vehicle = Vehicle(name, model=model, license=license, color=color)
            scenario.add_vehicle(
                vehicle, pos=pos, rot_quat=angle_to_quat((0, 0, 90))
            )
            vehicles.append(vehicle)
        vehicles_by_platoon.append(vehicles)

    scenario.make(bng)

    # Set simulator to 60hz temporal resolution
    bng.settings.set_deterministic(60)
    bng.scenario.load(scenario)
    bng.ui.show_hud()
    bng.scenario.start()

    sleep(5)

    # Create and launch the platoons.
    for platoon, vehicles in zip(platoons, vehicles_by_platoon):
        platoon_id = bng.platoon.create(vehicles[0], vehicles[1])
        for vehicle in vehicles[2:]:
            bng.platoon.join(platoon_id, vehicle)
        bng.platoon.launch(platoon_id, 2, platoon["speed"])
        sleep(5) # wait for platoon in front to get going

    all_vehicles = [vehicle for vehicles in vehicles_by_platoon for vehicle in vehicles]
    n_vehicles = len(all_vehicles)

    time = []
    origins = [None] * n_vehicles
    positions = [[] for _ in range(n_vehicles)]
    velocities = [[] for _ in range(n_vehicles)]
    accels = [[] for _ in range(n_vehicles)]
    old_vels = [0.0] * n_vehicles
    platoon_diffs = [
        [[] for _ in range(len(vehicles) - 1)]
        for vehicles in vehicles_by_platoon
    ]

    old_time = 0

    sleep(3.0)
    for i in range(120):
        states = []
        for vehicle in all_vehicles:
            vehicle.sensors.poll()
            states.append(vehicle.sensors["state"])

        time.append(states[0]["time"])
        current_positions = []

        for idx, data in enumerate(states):
            posx, posy, posz = data["pos"]
            current_positions.append((posx, posy, posz))

            if not positions[idx]:
                origins[idx] = (posx, posy, posz)

            ox, oy, oz = origins[idx]
            positions[idx].append(
                math.sqrt(
                    (posx - ox) ** 2 + (posy - oy) ** 2 + (posz - oz) ** 2
                )
            )

            velx, vely, _ = data["vel"]
            vel = math.sqrt(velx**2 + vely**2) * 3.6
            velocities[idx].append(vel)

            if old_time != 0:
                accels[idx].append(
                    (vel / 3.6 - old_vels[idx] / 3.6) / (time[i] - old_time)
                )
            else:
                accels[idx].append(0)
            old_vels[idx] = vel

        offset = 0
        for p_idx, vehicles in enumerate(vehicles_by_platoon):
            gap = platoons[p_idx]["gap"]
            for pair_idx in range(len(vehicles) - 1):
                p1 = current_positions[offset + pair_idx]
                p2 = current_positions[offset + pair_idx + 1]
                platoon_diffs[p_idx][pair_idx].append(
                    math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) - gap
                )
            offset += len(vehicles)

        old_time = time[i]
        bng.step(20)

    bng.disconnect()

    vehicle_offset = 0
    for platoon, vehicles, diffs in zip(platoons, vehicles_by_platoon, platoon_diffs):
        n = len(vehicles)
        colors = platoon["plot_colors"]
        platoon_positions = positions[vehicle_offset : vehicle_offset + n]
        platoon_velocities = velocities[vehicle_offset : vehicle_offset + n]
        platoon_accels = accels[vehicle_offset : vehicle_offset + n]
        vehicle_offset += n

        fig, ax = plt.subplots(2, 2, figsize=(9, 6), sharey=False)

        ax[0, 0].set(
            xlabel="Simulation Time (s)",
            ylabel="Speed (Km/h)",
            title="Speed vs. Time",
        )
        for v_idx in range(n):
            ax[0, 0].plot(
                time,
                platoon_velocities[v_idx],
                colors[v_idx],
                label=f"Vehicle {v_idx + 1} Velocity (Km/h)",
            )
        ax[0, 0].legend()

        ax[0, 1].set(
            xlabel="Simulation Time (s)",
            ylabel="Vehicle Position (m)",
            title="Position vs. Time",
        )
        for v_idx in range(n):
            ax[0, 1].plot(
                time,
                platoon_positions[v_idx],
                colors[v_idx],
                label=f"Vehicle {v_idx + 1} Position",
            )
        ax[0, 1].legend()

        ax[1, 0].set(
            xlabel="Simulation Time (s)",
            ylabel=" Position difference between vehicles(m)",
            title="interVehicular Distance",
        )
        for pair_idx, diff in enumerate(diffs):
            ax[1, 0].plot(
                time,
                diff,
                platoon["diff_colors"][pair_idx],
                label=platoon["diff_labels"][pair_idx],
            )
        ax[1, 0].legend()

        ax[1, 1].set(
            xlabel="Simulation Time (s)",
            ylabel="Vehicle Acceleration (m/s^2)",
            title="Acceleration vs. Time",
        )
        for v_idx in range(n):
            ax[1, 1].plot(
                time,
                platoon_accels[v_idx],
                colors[v_idx],
                label=f"Vehicle {v_idx + 1} acceleration",
            )
        ax[1, 1].legend()

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    main()
