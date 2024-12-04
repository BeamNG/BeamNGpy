from __future__ import annotations

import itertools
import random

import numpy as np
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle, angle_to_quat, sensors
from beamngpy.logging import BNGValueError
from beamngpy.types import Float3


def test_get_available_vehicles(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario("tech_ground", "spawn_test")
        vehicle = Vehicle("irrelevant", model="pickup")
        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

        bng.scenario.load(scenario)
        bng.scenario.start()
        resp = bng.vehicles.get_available()
        assert len(resp) > 0


def assert_continued_movement(bng: BeamNGpy, vehicle: Vehicle, start_pos: Float3):
    last_pos = np.array(start_pos)
    for _ in range(5):
        bng.control.step(120, wait=True)
        vehicle.sensors.poll()
        current_pos = np.array(vehicle.sensors["state"]["pos"])
        assert np.linalg.norm(current_pos - last_pos) > 0.5
        last_pos = current_pos


def assert_non_movement(bng: BeamNGpy, vehicle: Vehicle, start_pos: Float3):
    last_pos = np.array(start_pos)
    for _ in range(5):
        bng.control.step(60, wait=True)
        vehicle.sensors.poll()
        current_pos = np.array(vehicle.sensors["state"]["pos"])
        assert np.linalg.norm(current_pos - last_pos) < 0.5
        last_pos = current_pos


def test_vehicle_move(beamng: BeamNGpy):
    with beamng as bng:
        bng.settings.set_deterministic(50)

        scenario = Scenario("tech_ground", "move_test")
        vehicle = Vehicle("test_car", model="etk800")
        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(bng)
        bng.control.pause()
        bng.scenario.load(scenario)
        bng.scenario.start()
        vehicle.control(throttle=1)
        bng.control.step(100, wait=True)
        vehicle.sensors.poll()
        assert np.linalg.norm(vehicle.sensors["state"]["pos"]) > 1
        scenario.close()
        scenario.delete(beamng)


def test_vehicle_ai(beamng: BeamNGpy):
    with beamng as bng:
        bng.settings.set_deterministic(50)

        scenario = Scenario("west_coast_usa", "ai_test")
        vehicle = Vehicle("test_car", model="etk800")
        other = Vehicle("other", model="etk800")
        pos = (-717.121, 101, 118.675)
        scenario.add_vehicle(vehicle, pos=pos, rot_quat=angle_to_quat((0, 0, -45)))
        scenario.add_vehicle(
            other, pos=(-453, 700, 75), rot_quat=angle_to_quat((0, 0, 45))
        )
        scenario.make(bng)

        bng.control.pause()
        bng.scenario.load(scenario)
        bng.scenario.start()

        vehicle.switch()

        vehicle.ai.set_mode("traffic")
        bng.control.step(500)
        assert_continued_movement(bng, vehicle, pos)

        bng.scenario.restart()
        bng.control.pause()

        vehicle.ai.set_waypoint("Bridge26_2")
        bng.control.step(500)
        assert_continued_movement(bng, vehicle, pos)

        bng.scenario.restart()
        bng.control.pause()

        vehicle.ai.set_speed(80 / 3.6)
        vehicle.ai.set_target("other", mode="chase")
        bng.control.step(500)
        assert_continued_movement(bng, vehicle, pos)

        bng.scenario.restart()
        bng.control.pause()

        vehicle.ai.set_target("other", mode="flee")
        bng.control.step(500)
        assert_continued_movement(bng, vehicle, pos)

        bng.scenario.restart()
        vehicle.teleport(
            (-719.19, 106.10, 118.53), rot_quat=angle_to_quat((0, 0, 45)), reset=False
        )
        bng.control.pause()

        script = [
            {"x": -719.0, "y": 106.0, "z": 118.0, "t": 0.0},
            {"x": -758.0, "y": 67.0, "z": 118.0, "t": 5.0},
            {"x": -825.0, "y": 0.0, "z": 117.0, "t": 12.0},
        ]
        vehicle.ai.set_script(script)
        bng.control.step(670, wait=True)
        vehicle.sensors.poll()
        ref = (script[2]["x"], script[2]["y"], script[2]["z"])
        pos = vehicle.sensors["state"]["pos"]
        ref, pos = np.array(ref), np.array(pos)
        assert np.linalg.norm(ref - pos) < 2.5

        scenario.close()
        scenario.delete(beamng)


def test_dynamic_vehicle_spawn(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario("tech_ground", "dynamic spawn test")
        unique_vehicle_name = "unique"
        vehicle = Vehicle(unique_vehicle_name, model="pickup")
        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

        bng.scenario.load(scenario)
        bng.scenario.start()

        duplicate = Vehicle(unique_vehicle_name, model="etk800")

        with pytest.raises(BNGValueError):
            scenario.add_vehicle(duplicate, (0, 10, 0))


def test_vehicle_spawn(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario("tech_ground", "spawn_test")
        vehicle = Vehicle("irrelevant", model="pickup")
        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.make(beamng)

        bng.scenario.load(scenario)
        bng.scenario.start()

        other = Vehicle("relevant", model="etk800")
        scenario.add_vehicle(other, pos=(10, 10, 0))
        other.sensors.poll()
        assert other.is_connected()
        assert "pos" in other.sensors["state"]
        bng.control.step(120, wait=True)
        scenario.remove_vehicle(other)
        bng.control.step(600, wait=True)
        assert not other.is_connected()


def test_vehicle_bbox(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario("west_coast_usa", "bbox_test")
        vehicle_a = Vehicle("vehicle_a", model="etk800")
        vehicle_b = Vehicle("vehicle_b", model="etk800")
        pos = (-717.121, 101, 118.675)
        scenario.add_vehicle(vehicle_a, pos=pos, rot_quat=angle_to_quat((0, 0, 45)))
        pos = (-453, 700, 75)
        scenario.add_vehicle(vehicle_b, pos=pos, rot_quat=angle_to_quat((0, 0, 45)))
        scenario.make(beamng)

        bng.control.pause()
        bng.scenario.load(scenario)
        bng.scenario.start()

        bbox_beg = vehicle_a.get_bbox()
        vehicle_a.ai.set_mode("traffic")
        bng.control.step(2000, wait=True)
        bbox_end = vehicle_a.get_bbox()

        for k, v in bbox_beg.items():
            assert k in bbox_end
            assert v != bbox_end[k]


def _get_matching_entries(a, b):
    ret = {}
    for k, _ in a.items():
        ret[k] = b[k]
    return ret


def _check_lights(target, data, msg_fmt):
    data = _get_matching_entries(target, data)

    for l, v in target.items():
        msg = msg_fmt.format(target, l)
        assert v == data[l], msg


def test_lights(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario("tech_ground", "bbox_test")
        config = "vehicles/etk800/854_police_A.pc"
        vehicle = Vehicle("vehicle", model="etk800", part_config=config)
        other = Vehicle("other", model="pickup")

        vehicle.sensors.attach("electrics", sensors.Electrics())
        other.sensors.attach("electrics", sensors.Electrics())
        scenario.add_vehicle(vehicle, pos=(0, 0, 0))
        scenario.add_vehicle(other, pos=(10, 10, 0))
        scenario.make(beamng)

        binary = {"left_signal", "right_signal", "hazard_signal"}
        ternary = {"headlights", "fog_lights", "lightbar"}

        all_off = {}
        possible = []
        for light in binary:
            all_off[light] = False
            possible.append((light, False))
            possible.append((light, True))
        for light in ternary:
            all_off[light] = 0
            for i in range(3):
                possible.append((light, i))

        bng.control.pause()
        bng.scenario.load(scenario)
        bng.scenario.start()

        bng.control.step(100)

        for r in range(len(binary) + len(ternary)):
            r = r + 1
            for idx, combo in enumerate(itertools.combinations(possible, r)):
                if idx > 1024:
                    break

                vals = {}
                for light, value in combo:
                    vals[light] = value

                expected = dict(**vals)

                if "hazard_signal" in expected and expected["hazard_signal"]:
                    # left/right signals are overridden by the hazard light
                    # so we skip tests that set both
                    if "left_signal" in vals:
                        continue
                    if "right_signal" in vals:
                        continue
                else:
                    # Signals negate each other so we skip tests that set both
                    if "left_signal" in vals and vals["left_signal"]:
                        if "right_signal" in vals and vals["right_signal"]:
                            continue
                    if "right_signal" in vals and vals["right_signal"]:
                        if "left_signal" in vals and vals["left_signal"]:
                            continue

                # bng.step(1, wait=True)
                vehicle.set_lights(**vals)
                bng.control.step(1, wait=True)

                vehicle.sensors.poll()
                data = vehicle.sensors["electrics"]
                msg_fmt = (
                    "Setting the combination of {} did not result in "
                    "corresponding light states in the case of {}."
                )
                _check_lights(expected, data, msg_fmt)

                other.sensors.poll()
                data = other.sensors["electrics"]
                msg_fmt = (
                    "Other vehicle has lights on when it should not. "
                    "Expected {} but got mismatch in the case of: {}"
                )
                _check_lights(all_off, data, msg_fmt)

                vehicle.set_lights(**all_off)
                bng.control.step(1, wait=True)

                vehicle.sensors.poll()
                data = vehicle.sensors["electrics"]
                msg_fmt = (
                    "Lights did not turn off correctly. Expected {} "
                    "but got mismatch in the case of: {}"
                )
                _check_lights(all_off, data, msg_fmt)


def test_traffic(beamng: BeamNGpy):
    with beamng as bng:
        bng.settings.set_deterministic(50)

        scenario = Scenario("west_coast_usa", "traffic_test")
        vehicle = Vehicle("ego", model="etk800", color="maroon")
        other = Vehicle("traffic", model="etk800", color="darkgreen")
        pos = (-717.121, 101, 118.675)
        scenario.add_vehicle(vehicle, pos=pos, rot_quat=angle_to_quat((0, 0, 45)))
        pos = (-425, 675, 75)
        scenario.add_vehicle(other, pos=pos, rot_quat=angle_to_quat((0, 0, -45)))
        scenario.make(bng)

        bng.control.pause()
        bng.scenario.load(scenario)
        bng.scenario.start()

        bng.traffic.start([other])
        other.switch()

        bng.control.step(300, wait=True)  # Give vehicle ~5 seconds to start

        try:
            assert_continued_movement(bng, other, pos)
        finally:
            bng.control.resume()


def test_part_configs(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario("tech_ground", "parts_test")
        vehicle = Vehicle("ego", model="etk800")
        scenario.add_vehicle(vehicle)
        scenario.make(bng)

        bng.scenario.load(scenario)
        bng.scenario.start()

        options = vehicle.get_part_options()
        assert len(options) > 0

        config = {}
        for k, v in options.items():
            if k.startswith("etk800"):
                config[k] = random.choice(v)
        vehicle.set_part_config({"parts": config})

        current = vehicle.get_part_config()
        for k, v in config.items():
            assert v == current["parts"][k]
