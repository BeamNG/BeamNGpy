"""
Keep spawning NPC vehicles while a scenario is already running, without the
camera ever switching away from the player's vehicle.

Passing ``auto_enter=False`` prevents the new vehicle from becoming the player
vehicle, so camera and input focus stay where they are.

Each NPC is spawned ahead of the player, so they keep appearing as the player
drives. The player vehicle id is printed after every spawn to show that focus
never moves.
"""

from time import sleep

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging

NPC_COUNT = 6  # how many NPCs to spawn over the run
SPAWN_INTERVAL = 5.0  # seconds between spawns
SPAWN_AHEAD = 60.0  # meters ahead of the player to spawn at
NPC_MODELS = ("pickup", "covet", "midsize", "sunburst2")


def spawn_keep_focus(bng, vehicle, pos, rot_quat=(0, 0, 0, 1)):
    """Spawn ``vehicle`` without moving camera/input focus."""
    return bng.vehicles.spawn(
        vehicle,
        pos,
        rot_quat=rot_quat,
        safe_spawn=True,
        auto_enter=False,
        check_vehicle_limit=False,
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

    # Drive the player around so the NPCs keep appearing in front of a moving car.
    # A participant driving manually behaves the same way.
    player.ai.set_mode("span")

    print(f"focus after scenario start: {beamng.vehicles.get_player_vehicle_id()['vid']}")

    npcs = []
    for i in range(NPC_COUNT):
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

        npc = Vehicle(f"npc_{i + 1}", model=NPC_MODELS[i % len(NPC_MODELS)])
        if spawn_keep_focus(beamng, npc, spawn_pos, rot_quat=rotation):
            npc.ai.set_mode("traffic")
            npcs.append(npc)
            print(f"spawned {npc.vid} ({npc.model})")
        else:
            print(f"failed to spawn {npc.vid}")

        focus = beamng.vehicles.get_player_vehicle_id()["vid"]
        print(f"  vehicles: {len(npcs) + 1}, focus: {focus}")
        if focus != player.vid:
            print(f"  WARNING: focus moved off {player.vid}")

    sleep(10.0)
    beamng.disconnect()
    input("Press Enter to exit...")


if __name__ == "__main__":
    main()
