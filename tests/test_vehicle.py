from __future__ import annotations

import random
from typing import Iterator

import numpy as np
import pytest

from beamngpy import BeamNGpy, Scenario, Vehicle, angle_to_quat, sensors
from beamngpy.logging import BNGValueError
from beamngpy.types import Float3, StrDict

from tests.helpers import loaded_scenario, restart_vehicles

_ISLAND_EGO_POS: Float3 = (-241.901, -379.48, 31.719)
_ISLAND_OTHER_POS: Float3 = (148.242, 292.909, 41.847)
_ISLAND_EGO_ROT = angle_to_quat((0, 0, -85))
_ISLAND_OTHER_ROT = angle_to_quat((0, 0, 95))
_SPAWN_NEAR_EGO: Float3 = (-231.901, -379.48, 31.719)


def assert_continued_movement(bng: BeamNGpy, vehicle: Vehicle, start_pos: Float3):
    last_pos = np.array(start_pos)
    for _ in range(5):
        bng.control.step(120, wait=True)
        vehicle.sensors.poll()
        current_pos = np.array(vehicle.sensors["state"]["pos"])
        assert np.linalg.norm(current_pos - last_pos) > 0.5
        last_pos = current_pos


class TestVehicleShared:
    @pytest.fixture(scope="class")
    def scene(
        self, beamng: BeamNGpy
    ) -> Iterator[tuple[BeamNGpy, Vehicle, Vehicle, Scenario]]:
        config = "vehicles/etk800/854_police_A.pc"
        vehicle = Vehicle("test_car", model="etk800", part_config=config)
        other = Vehicle("other", model="etk800")
        vehicle.sensors.attach("electrics", sensors.Electrics())
        other.sensors.attach("electrics", sensors.Electrics())
        with loaded_scenario(
            beamng,
            level="small_island",
            name="vehicle_shared",
            vehicles=[
                (vehicle, {"pos": _ISLAND_EGO_POS, "rot_quat": _ISLAND_EGO_ROT}),
                (other, {"pos": _ISLAND_OTHER_POS, "rot_quat": _ISLAND_OTHER_ROT}),
            ],
            hz=50,
            pause=True,
        ) as (bng, scenario):
            yield bng, vehicle, other, scenario

    @pytest.fixture(autouse=True)
    def _reset_shared_vehicles(self, scene):
        bng, vehicle, other, _ = scene
        restart_vehicles(bng, vehicle, other)

    def test_get_available_vehicles(self, scene):
        bng, *_ = scene
        resp = bng.vehicles.get_available()
        assert len(resp) > 0

    def test_vehicle_move(self, scene):
        bng, vehicle, *_ = scene
        vehicle.sensors.poll()
        start = np.array(vehicle.sensors["state"]["pos"])
        vehicle.control(throttle=1)
        bng.control.step(100, wait=True)
        vehicle.sensors.poll()
        assert np.linalg.norm(np.array(vehicle.sensors["state"]["pos"]) - start) > 1

    def test_lights(self, scene):
        bng, vehicle, other, _ = scene

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

        bng.control.step(10)

        cases = [{light: value} for light, value in possible]
        cases.append({"headlights": 2, "fog_lights": 1, "lightbar": 1})
        cases.append({"left_signal": True, "headlights": 1})
        cases.append({"hazard_signal": True, "fog_lights": 2})
        for vals in cases:
            expected = dict(**vals)

            if expected.get("hazard_signal"):
                if "left_signal" in vals or "right_signal" in vals:
                    continue
            elif vals.get("left_signal") and vals.get("right_signal"):
                continue

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

    def test_vehicle_ai(self, scene):
        bng, vehicle, other, _ = scene
        vehicle.switch()

        def traffic():
            vehicle.ai.set_mode("traffic")

        def waypoint():
            vehicle.ai.set_waypoint("wp_1")

        def chase():
            vehicle.ai.set_speed(80 / 3.6)
            vehicle.ai.set_target("other", mode="chase")

        def flee():
            vehicle.ai.set_target("other", mode="flee")

        for i, setup in enumerate((traffic, waypoint, chase, flee)):
            if i:
                restart_vehicles(bng, vehicle, other)
            setup()
            bng.control.step(500)
            assert_continued_movement(bng, vehicle, _ISLAND_EGO_POS)

        restart_vehicles(bng, vehicle, other)
        vehicle.teleport(
            _ISLAND_EGO_POS,
            rot_quat=_ISLAND_EGO_ROT,
            reset=False,
        )
        bng.control.pause()

        script = [
            {"x": -241.9, "y": -379.5, "z": 31.7, "t": 0.0},
            {"x": -231.9, "y": -380.3, "z": 31.7, "t": 5.0},
            {"x": -221.9, "y": -381.2, "z": 31.7, "t": 10.0},
        ]
        vehicle.ai.set_script(script)
        bng.control.step(670, wait=True)
        vehicle.sensors.poll()
        ref = (script[2]["x"], script[2]["y"], script[2]["z"])
        pos = vehicle.sensors["state"]["pos"]
        ref, pos = np.array(ref), np.array(pos)
        assert np.linalg.norm(ref - pos) < 2.5

    def test_vehicle_bbox(self, scene):
        bng, vehicle_a, *_ = scene
        bbox_beg = vehicle_a.get_bbox()
        vehicle_a.ai.set_mode("traffic")
        bng.control.step(2000, wait=True)
        bbox_end = vehicle_a.get_bbox()

        for k, v in bbox_beg.items():
            assert k in bbox_end
            assert v != bbox_end[k]

    def test_traffic(self, scene):
        bng, _, other, _ = scene
        bng.traffic.start([other])
        other.switch()

        bng.control.step(600, wait=True)

        try:
            assert_continued_movement(bng, other, _ISLAND_OTHER_POS)
        finally:
            bng.traffic.stop()
            bng.control.pause()

    def test_spawn_without_entering_vehicle(self, scene):
        bng, vehicle, *_ = scene

        npc = Vehicle("npc", model="pickup")
        try:
            assert bng.vehicles.spawn(
                npc,
                pos=_SPAWN_NEAR_EGO,
                auto_enter=False,
            )
            assert bng.vehicles.get_player_vehicle_id()["vid"] == vehicle.vid
        finally:
            bng.vehicles.despawn(npc)

    def test_dynamic_vehicle_spawn(self, scene):
        _, vehicle, _, scenario = scene
        duplicate = Vehicle(vehicle.vid, model="etk800")
        with pytest.raises(BNGValueError):
            scenario.add_vehicle(duplicate, _SPAWN_NEAR_EGO)

    def test_vehicle_spawn(self, scene):
        bng, _, _, scenario = scene

        extra = Vehicle("relevant", model="etk800")
        scenario.add_vehicle(extra, pos=_SPAWN_NEAR_EGO)
        extra.sensors.poll()
        assert extra.is_connected()
        assert "pos" in extra.sensors["state"]
        bng.control.step(120, wait=True)
        scenario.remove_vehicle(extra)
        bng.control.step(600, wait=True)
        assert not extra.is_connected()

    def test_part_configs(self, scene):
        _, vehicle, *_ = scene

        options = vehicle.get_part_config()
        assert len(options) > 0

        options = _generate_random_config(options)
        vehicle.set_part_config(options)

        current = vehicle.get_part_config()

        options_flat = _flatten_config_tree(options)
        current_flat = _flatten_config_tree(current)
        for k, v in current_flat.items():
            if k in options_flat:
                assert v == options_flat[k]


def _check_lights(target, data, msg_fmt):
    data = {light: data[light] for light in target}

    for light, value in target.items():
        msg = msg_fmt.format(target, light)
        assert value == data[light], msg


def _generate_random_config(options: StrDict):
    nodes = [options['partsTree']]
    while nodes:
        node = nodes.pop()
        if node['suitablePartNames']:
            node['chosenPartName'] = random.choice(node['suitablePartNames'])
        node['decisionMethod'] = 'user'
        children = node.get('children', {})
        if len(children) > 0:
            nodes.extend(children.values())
    return options


def _flatten_config_tree(config):
    parts = {}
    nodes = [config['partsTree']]
    while nodes:
        node = nodes.pop()
        if not 'id' in node:
            continue
        parts[node['path']] = node.get('chosenPartName')
        children = node.get('children', {})
        if len(children) > 0:
            nodes.extend(children.values())
    return parts
