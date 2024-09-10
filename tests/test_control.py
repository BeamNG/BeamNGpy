from __future__ import annotations

import time

from beamngpy import BeamNGpy, Scenario, Vehicle


def test_camera_control(beamng: BeamNGpy):
    with beamng as bng:
        scenario = Scenario("tech_ground", "camera_control_test")
        ego = Vehicle("ego", model="etk800")
        other = Vehicle("other", model="etk800")

        scenario.add_vehicle(ego, pos=(0, 0, 0))
        scenario.add_vehicle(other, pos=(5, 5, 0))
        scenario.make(bng)

        bng.scenario.load(scenario)
        bng.scenario.start()

        vehicle_info = bng.vehicles.get_current_info()
        ego_id_num = vehicle_info[ego.vid]["id"]

        camera_configs = bng.camera.get_player_modes(ego)
        for camera in camera_configs.keys():
            if camera != "crash":
                bng.camera.set_player_mode(ego, camera, {})
            else:
                bng.camera.set_player_mode(ego, camera, {}, {"veh1Id": ego_id_num})
            time.sleep(5)
            current_camera = bng.camera.get_player_modes(ego)
            assert current_camera[camera]["focused"]

        time.sleep(10)

        assert "orbit" in camera_configs

        bng.camera.set_player_mode(ego, "orbit", {"distance": 50})
        time.sleep(5)
        current_camera = bng.camera.get_player_modes(ego)

        assert current_camera["orbit"]["focused"]
        assert (abs(current_camera["orbit"]["camDist"])-50) < 0.5
