"""
Keep spawning traffic vehicles while a scenario is already running, without the
camera ever switching away from the player's vehicle.

Passing ``auto_enter=False`` prevents the new vehicle from becoming the player
vehicle, so camera and input focus stay where they are.

Each traffic vehicle is spawned ahead of the player, so they keep appearing as
the player drives. The player vehicle id is printed after every spawn to show
that focus never moves.
"""

from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

TRAFFIC_COUNT = 6  # how many traffic vehicles to spawn over the run
SPAWN_INTERVAL = 5.0  # seconds between spawns
SPAWN_AHEAD = 60.0  # meters ahead of the player to spawn at
TRAFFIC_MODELS = ("pickup", "covet", "midsize", "sunburst2")


def spawn_keep_focus(bng, vehicle, pos, rot_quat=(0, 0, 0, 1)):
    """Spawn ``vehicle`` without moving camera/input focus."""
    return bng.vehicles.spawn(
        vehicle,
        pos,
        rot_quat=rot_quat,
        safe_spawn=True,
        auto_enter=False,
    )


def main():
    set_up_simple_logging()

    beamng = BeamNGpy("localhost", 25252)
    beamng.open(launch=True)

    scenario = Scenario("west_coast_usa", "spawn_without_camera_switch")
    player = Vehicle("participant", model="etk800", license="PLAYER")
    scenario.add_vehicle(player, pos=(-769.1, 400.8, 142.8), rot_quat=(0, 0, 1, 0))
    scenario.make(beamng)

    beamng.scenario.load(scenario)
    beamng.scenario.start()

    # Drive the player around so the traffic vehicles keep appearing in front of a moving car.
    # A participant driving manually behaves the same way.
    player.ai.set_mode("span")

    print(f"focus after scenario start: {beamng.vehicles.get_player_vehicle_id()['vid']}")

    traffic_vehicles = []
    for i in range(TRAFFIC_COUNT):
        sleep(SPAWN_INTERVAL)

        player.sensors.poll()
        pos = player.state["pos"]
        direction = player.state["dir"]
        rotation = player.state["rotation"]
        spawn_pos = (
            pos[0] + direction[0] * SPAWN_AHEAD,
            pos[1] + direction[1] * SPAWN_AHEAD,
            pos[2] + 0.5,
        )

        traffic = Vehicle(f"traffic_{i + 1}", model=TRAFFIC_MODELS[i % len(TRAFFIC_MODELS)])
        if spawn_keep_focus(beamng, traffic, spawn_pos, rot_quat=rotation):
            traffic.ai.set_mode("traffic")
            traffic_vehicles.append(traffic)
            print(f"spawned {traffic.vid} ({traffic.model})")
        else:
            print(f"failed to spawn {traffic.vid}")

        focus = beamng.vehicles.get_player_vehicle_id()["vid"]
        print(f"  vehicles: {len(traffic_vehicles) + 1}, focus: {focus}")
        if focus != player.vid:
            print(f"  WARNING: focus moved off {player.vid}")

    sleep(10.0)
    beamng.disconnect()
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
