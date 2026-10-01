from __future__ import annotations

from time import sleep

from beamngpy.sensors import Mesh
from tests.helpers import exercise_roundtrip, module_scene, using_sensor

mesh_scene = module_scene("mesh_test2", "Mesh analysis", "Mesh test")


def test_mesh(mesh_scene):
    bng, vehicle = mesh_scene
    with using_sensor(Mesh("mesh1", bng, vehicle), "mesh sensor removed.") as mesh:
        nodes = mesh.get_node_positions()
        print("Current node positions from simulator:")
        print(nodes)

        for _ in range(3):
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


def test_mesh_property_roundtrips(mesh_scene):
    """Assert getter/setter round-trip for mesh update time."""
    bng, vehicle = mesh_scene
    with using_sensor(Mesh("mesh_props", bng, vehicle), "mesh sensor removed.") as mesh:
        exercise_roundtrip(mesh)
