from __future__ import annotations

import socket

from beamngpy import (BeamNGpy, Scenario, ScenarioObject, Vehicle,
                      angle_to_quat, set_up_simple_logging)


def test_quats(beamng: BeamNGpy):
    with beamng as bng:
        set_up_simple_logging()

        scenario = Scenario("tech_ground", "test_quat")

        blue_etk = Vehicle("ego_vehicle", model="etk800", color="Blue", license="angle")
        scenario.add_vehicle(blue_etk, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1))

        blue_etk = Vehicle(
            "ego_vehicle2", model="etk800", color="Green", license="quat"
        )
        rot_quat = (-0.00333699025, -0.00218820246, -0.689169466, 0.724589229)
        scenario.add_vehicle(blue_etk, pos=(5, 0, 0), rot_quat=rot_quat)

        rb = ScenarioObject(
            oid="roadblock",
            name="sawhorse",
            otype="BeamNGVehicle",
            pos=(-10, -5, 0),
            scale=(1, 1, 1),
            rot_quat=(0, 0, 0, 1),
            JBeam="sawhorse",
            datablock="default_vehicle",
        )
        scenario.add_object(rb)

        cn = ScenarioObject(
            oid="cones",
            name="cones",
            otype="BeamNGVehicle",
            pos=(0, -5, 0),
            scale=(1, 1, 1),
            rot_quat=(0, 0, 0, 1),
            JBeam="cones",
            datablock="default_vehicle",
        )
        scenario.add_object(cn)

        scenario.make(beamng)

        bng.scenario.load(scenario)
        bng.scenario.start()

        white_etk = Vehicle("ego_vehicle3", model="etk800", color="White")
        scenario.add_vehicle(white_etk, (-10, 0, 0), rot_quat=(0, 0, 0, 1))

        pickup = Vehicle("ego_vehicle4", model="pickup")
        pos = (-15, 0, 0)
        scenario.add_vehicle(pickup, pos, rot_quat=(0, 0, 0, 1))
        resp = bng.vehicles.get_current()
        assert len(resp) == 6

        pickup.connect(bng)

        pickup.sensors.poll()
        pos_before = pickup.state["pos"]
        pickup.teleport(pos, rot_quat=angle_to_quat((0, 45, 0)))
        pickup.sensors.poll()
        pos_after = pickup.state["pos"]
        assert pos_before != pos_after

        pickup.sensors.poll()
        pos_before = pickup.state["pos"]
        rot_quat = (-0.00333699025, -0.00218820246, -0.689169466, 0.724589229)
        pickup.teleport(pos, rot_quat=rot_quat)
        pickup.sensors.poll()
        pos_after = pickup.state["pos"]
        assert pos_before != pos_after

        try:
            bng.scenario.teleport_object(
                rb, (-10, 5, 0), rot_quat=angle_to_quat((-45, 0, 0))
            )
            assert True
        except socket.timeout:
            assert False

        try:
            rot_quat = (-0.003337, -0.0021882, -0.6891695, 0.7245892)
            bng.scenario.teleport_object(rb, (-10, 5, 0), rot_quat=rot_quat)
            assert True
        except socket.timeout:
            assert False


if __name__ == "__main__":
    bng = BeamNGpy("localhost", 25252)
    test_quats(bng)
