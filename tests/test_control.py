import time

import pytest
from beamngpy import BeamNGpy, Scenario, Vehicle


@pytest.fixture()
def beamng():
    beamng = BeamNGpy('localhost', 64256)
    return beamng


def test_camera_control(beamng):
    with beamng as bng:
        scenario = Scenario('smallgrid', 'camera_control_test')
        ego = Vehicle('ego', model='etk800')
        other = Vehicle('other', model='etk800')

        scenario.add_vehicle(ego, pos=(0, 0, 0))
        scenario.add_vehicle(other, pos=(5, 5, 0))
        scenario.make(bng)

        bng.load_scenario(scenario)
        bng.start_scenario()

        vehicle_info = bng.get_current_vehicles_info()
        ego_id_num = vehicle_info['ego']['id']

        camera_configs = bng.get_player_camera_modes(ego.vid)
        for camera in camera_configs.keys():
            if camera != 'crash':
                bng.set_player_camera_mode(ego.vid, camera, {})
            else:
                bng.set_player_camera_mode(ego.vid, camera, {}, {'veh1Id': ego_id_num})
            time.sleep(5)
            current_camera = bng.get_player_camera_modes(ego.vid)
            assert current_camera[camera]['focused']

        time.sleep(10)

        assert 'orbit' in camera_configs

        bng.set_player_camera_mode(ego.vid, 'orbit', {'distance': 50})
        time.sleep(5)
        current_camera = bng.get_player_camera_modes(ego.vid)

        assert current_camera['orbit']['focused']
        assert abs(current_camera['orbit']['camDist'] - 50) < 0.5
