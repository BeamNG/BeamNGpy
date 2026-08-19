from __future__ import annotations

import time
import uuid

from beamngpy import BeamNGpy, Scenario, Vehicle


def test_traffic_signals_api(beamng: BeamNGpy) -> None:
    rid = uuid.uuid4().hex[:8]
    with beamng as bng:
        scenario = Scenario("west_coast_usa", f"ts_{rid}")
        vehicle = Vehicle(f"ego_{rid}", model="etk800", license="TS1")
        scenario.add_vehicle(
            vehicle,
            pos=(-717.121, 101, 118.675),
            rot_quat=(0, 0, 0.9238795, -0.3826834),
        )
        scenario.make(bng)
        bng.scenario.load(scenario, precompile_shaders=False)
        bng.scenario.start()
        bng.control.resume()

        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            if vehicle.vid in bng.get_current_vehicles_info(include_config=False):
                vehicle.connect(bng)
                break
            time.sleep(0.35)
        else:
            raise TimeoutError(f"{vehicle.vid!r} not spawned")

        ts = bng.traffic_signals
        name = ts.list_instances()[0]["name"]

        assert ts.get_map_node_signals()
        editor = ts.get_editor_signals(name)
        assert editor["controller"]["states"]
        assert ts.get_timing_snapshot()["controllers"]

        light = ts.get_traffic_light(name, distance_m=12.0)
        vehicle.teleport(light["pos"], light["rot"], reset=True)

        assert ts.set_instance_strict_state(name, 1)["after"]["state"]
        assert ts.set_instance_strict_state(name, None)["after"]["state"]

        ctrl = editor["controller"]
        old = float(ctrl["states"][0]["duration"])
        ts.set_controller_state_duration(ctrl["name"], 1, old + 0.25)
        assert abs(
            float(ts.get_editor_signals(name)["controller"]["states"][0]["duration"]) - (old + 0.25)
        ) < 1e-3
        ts.set_controller_state_duration(ctrl["name"], 1, old)
