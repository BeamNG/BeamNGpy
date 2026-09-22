from __future__ import annotations

from math import sqrt
from time import sleep

import pytest

from beamngpy.sensors import Camera, Lidar, Radar, Ultrasonic
from tests.sensors.helpers import assert_vec3_close, module_scene, using_sensor

_SENSOR_TYPES = ("camera", "lidar", "radar", "ultrasonic")
_MOUNT = (-5.0, 0.0, 1.0)
_DIR = (0, -1, 0)
_STATIC_POS = (12.0, -3.0, 2.5)


_ATTACHED = {
    "camera": lambda name, bng, v: Camera(
        name, bng, v, pos=_MOUNT, dir=_DIR, is_using_shared_memory=False,
    ),
    "lidar": lambda name, bng, v: Lidar(name, bng, v, pos=_MOUNT, dir=_DIR),
    "radar": lambda name, bng, v: Radar(name, bng, v, pos=_MOUNT, dir=_DIR),
    "ultrasonic": lambda name, bng, v: Ultrasonic(name, bng, v, pos=_MOUNT, dir=_DIR),
}

_STATIC = {
    "camera": lambda name, bng: Camera(
        name, bng, None, is_static=True, is_using_shared_memory=False, pos=_STATIC_POS, dir=_DIR,
    ),
    "lidar": lambda name, bng: Lidar(name, bng, vehicle=None, is_static=True, pos=_STATIC_POS, dir=_DIR),
    "radar": lambda name, bng: Radar(name, bng, vehicle=None, is_static=True, pos=_STATIC_POS, dir=_DIR),
    "ultrasonic": lambda name, bng: Ultrasonic(
        name, bng, vehicle=None, is_static=True, pos=_STATIC_POS, dir=_DIR,
    ),
}

FAR_ORIGIN = (100_000.0, 100.0, 0.21)
STEP_M = 0.05
MOTION_STEPS = 25

world_position_scene = module_scene(
    "sensor_world_position_test",
    "World-position precision tests for attached sensors",
    "Sensor world position test",
    color="Green",
)


@pytest.mark.parametrize("sensor_type", _SENSOR_TYPES)
def test_world_position_api(world_position_scene, sensor_type: str):
    """Near-origin world-space API contract for attached and static sensors."""
    bng, vehicle = world_position_scene
    attached_name = f"{sensor_type}_world_api"
    static_name = f"{sensor_type}_static_world_api"

    with using_sensor(_ATTACHED[sensor_type](attached_name, bng, vehicle)) as sensor:
        vehicle.sensors.poll("state")
        vehicle_pos = vehicle.state["pos"]
        stored_pos = sensor.get_position(world_space=False)
        world_pos = sensor.get_position()

        dist_vehicle = sqrt(sum((world_pos[i] - vehicle_pos[i]) ** 2 for i in range(3)))
        dist_stored = sqrt(sum((world_pos[i] - stored_pos[i]) ** 2 for i in range(3)))
        assert dist_vehicle <= 20.0, (
            f"world position {world_pos} is implausibly far from vehicle {vehicle_pos}"
        )
        assert dist_stored >= 0.05, (
            f"world position {world_pos} should differ from stored position {stored_pos}"
        )

        before = sensor.get_position(world_space=False)
        sensor.set_position((100.0, 100.0, 100.0), world_space=True)
        after = sensor.get_position(world_space=False)
        assert_vec3_close(after, before)

    with using_sensor(_STATIC[sensor_type](static_name, bng)) as static_sensor:
        static_pos = (15.0, 1.0, 3.0)
        static_sensor.set_position(static_pos, world_space=True)
        assert_vec3_close(static_sensor.get_position(), static_pos)


@pytest.mark.parametrize("sensor_type", _SENSOR_TYPES)
def test_world_position_smooth_far_from_origin(world_position_scene, sensor_type: str):
    """Far from the origin, attached sensor world position should move smoothly with the vehicle."""
    bng, vehicle = world_position_scene
    name = f"{sensor_type}_world_far"

    vehicle.teleport(FAR_ORIGIN, reset=True)
    bng.control.step(60, wait=True)

    with using_sensor(_ATTACHED[sensor_type](name, bng, vehicle)) as sensor:
        bng.control.pause()
        vehicle_positions: list[tuple[float, float, float]] = []
        world_positions: list[tuple[float, float, float]] = []
        stored_positions: list[tuple[float, float, float]] = []
        motion_axis = 0

        for i in range(MOTION_STEPS):
            pos = list(FAR_ORIGIN)
            pos[motion_axis] += i * STEP_M
            vehicle.teleport(tuple(pos), reset=False)
            bng.control.step(60, wait=True)
            vehicle.sensors.poll("state")
            vehicle_positions.append(tuple(vehicle.state["pos"]))
            world_positions.append(sensor.get_position())
            stored_positions.append(sensor.get_position(world_space=False))

        stuck_frames = 0
        for i in range(1, len(world_positions)):
            delta_world = world_positions[i][motion_axis] - world_positions[i - 1][motion_axis]
            delta_vehicle = vehicle_positions[i][motion_axis] - vehicle_positions[i - 1][motion_axis]
            if abs(delta_vehicle) > 1e-9:
                if abs(delta_world) < 1e-6:
                    stuck_frames += 1
                assert abs(delta_world - delta_vehicle) <= 0.002, (
                    f"world delta {delta_world} != vehicle delta {delta_vehicle} at step {i}"
                )

        assert stuck_frames <= 0, (
            f"world position stuck for {stuck_frames} frames while vehicle moved"
        )

        offsets = [
            tuple(world_positions[i][j] - vehicle_positions[i][j] for j in range(3))
            for i in range(len(world_positions))
        ]
        ref = offsets[0]
        max_spread = max(
            sqrt(sum((offset[j] - ref[j]) ** 2 for j in range(3)))
            for offset in offsets[1:]
        )
        assert max_spread <= 0.012, (
            f"world-vehicle offset varied by {max_spread} m, max allowed 0.012"
        )

        for stored in stored_positions:
            assert_vec3_close(stored, _MOUNT)

        if sensor_type == "radar":
            bng.control.step(60, wait=True)
            sleep(1)
            readings = sensor.poll()
            assert len(readings.shape) == 2


def test_world_position_matrix_matches_api(world_position_scene):
    """getSensorMatrix translation should match get_position() for cameras."""
    bng, vehicle = world_position_scene
    name = "camera_matrix_world"

    vehicle.teleport(FAR_ORIGIN, reset=True)
    bng.control.step(60, wait=True)

    with using_sensor(_ATTACHED["camera"](name, bng, vehicle)) as sensor:
        api_pos = sensor.get_position()
        resp = bng.control.queue_lua_command(
            f"""
local name = '{name}'
for _, id in ipairs(extensions.tech_sensors.getActiveCameraSensors()) do
  if extensions.tech_sensors.getCameraSensorName(id) == name then
    local m = Research.SensorMatrixManager.getSensorMatrixExternal(id)
    return string.format('%f,%f,%f', m['03'], m['13'], m['23'])
  end
end
return 'not_found'
""",
            response=True,
        )
        assert resp is not None and resp != "not_found", f"camera sensor {name} not found in lua"
        matrix_pos = tuple(float(v) for v in resp.split(","))
        assert_vec3_close(matrix_pos, api_pos, tol=0.01)
