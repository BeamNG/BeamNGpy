"""
``BeamNGpy.traffic.start`` — hand already-spawned vehicles to the traffic system.

Traffic needs a navgraph. Empty ``tech_ground`` has none, so this example writes
``add_simple_road_network`` into the user folder **before** the level loads, then
spawns two vehicles on that loop and puts both under traffic AI. Initial focus is
only for the camera; both cars are traffic. Contrast ``traffic.spawn``, which
creates new traffic vehicles from game settings.
"""

from beamngpy import BeamNGpy, Scenario, Vehicle, angle_to_quat, set_up_simple_logging
from beamngpy.tools.tech_ground_util import add_simple_road_network, clear_tech_ground

# Simple network loop (see tech_proceduralGenerators): X 260–330, Y 68–188.
# East straight runs N–S at x=330; clockwise traffic heads south (-Y).
_EAST_X = 330.0
_FACE_SOUTH = angle_to_quat((0, 0, 180))


def main():
    set_up_simple_logging()

    bng = BeamNGpy("localhost", 25252)
    bng.open(launch=True)

    # Writes road/navgraph files; load below is the required level reload.
    add_simple_road_network(bng)

    scenario = Scenario("tech_ground", "traffic_participants_demo")
    # ~60 m apart on the east straight so they do not spawn colliding.
    ego_vehicle = Vehicle("ego_vehicle", model="etk800", license="EGO")
    traffic_a = Vehicle("traffic_a", model="pickup", license="TRAF-A", color="red")
    traffic_b = Vehicle("traffic_b", model="pickup", license="TRAF-B", color="blue")
    scenario.add_vehicle(ego_vehicle, pos=(_EAST_X, 130, 0), rot_quat=_FACE_SOUTH)
    scenario.add_vehicle(traffic_a, pos=(_EAST_X, 160, 0), rot_quat=_FACE_SOUTH)
    scenario.add_vehicle(traffic_b, pos=(_EAST_X, 100, 0), rot_quat=_FACE_SOUTH)
    scenario.set_initial_focus(ego_vehicle.vid)
    scenario.make(bng)

    bng.scenario.load(scenario)
    bng.scenario.start()

    bng.traffic.start([traffic_a, traffic_b])

    input("Both vehicles in traffic. Press Enter to stop...")
    bng.traffic.stop(stop=True)
    clear_tech_ground(bng)
    bng.disconnect()


if __name__ == "__main__":
    main()
