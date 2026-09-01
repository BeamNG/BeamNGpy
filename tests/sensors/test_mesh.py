from __future__ import annotations

from contextlib import contextmanager
from time import sleep
from typing import Iterator

from beamngpy import BeamNGpy, Vehicle
from beamngpy.sensors import Mesh
from tests.sensors.helpers import loaded_scenario, make_module_scene_fixture

@contextmanager
def mesh_scenario(beamng: BeamNGpy) -> Iterator[tuple[BeamNGpy, Vehicle]]:
    vehicle = Vehicle("ego_vehicle", model="etki", licence="PYTHON", color="Red")
    with loaded_scenario(
        beamng,
        level="tech_ground",
        name="mesh_test2",
        description="Mesh analysis",
        vehicles=[(vehicle, {})],
        label="Mesh test",
    ) as scene:
        yield scene

mesh_scene = make_module_scene_fixture(mesh_scenario)

def test_mesh(mesh_scene, steps: int = 3):
    bng, vehicle = mesh_scene
    mesh = Mesh("mesh1", bng, vehicle)

    nodes = mesh.get_node_positions()
    print("Current node positions from simulator:")
    print(nodes)

    for _ in range(steps):
        sleep(1)
        data = mesh.poll()
        print(data)
        print("data time: ", data["time"])
        mesh.mesh_plot()
        mesh.mass_distribution_plot(data["nodes"])
        mesh.force_distribution_plot(data["nodes"])
        mesh.force_direction_plot(data["nodes"])
        mesh.velocity_distribution_plot(data["nodes"])
        mesh.velocity_direction_plot(data["nodes"])
