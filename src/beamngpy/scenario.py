"""
.. module:: scenario
    :platform: Windows
    :synopsis: Contains the main :py:class:`.beamngpy.Scenario` class used to
               define scenarios.
"""

import json
import logging as log
import math

from pathlib import Path

import numpy as np

from jinja2 import Environment
from jinja2.loaders import PackageLoader

TEMPLATE_ENV = Environment(loader=PackageLoader('beamngpy'))


def compute_rotation_matrix(angles):
    """
    Calculates the rotation matrix string for the given triplet of Euler angles
    to be used in a scenario prefab.

    Args:
        angles (tuple): Euler angles for the (x,y,z) axes.

    Return:
        The rotation matrix encoded as a string.
    """
    angles = [np.radians(a) for a in angles]

    sin_a = math.sin(angles[0])
    cos_a = math.cos(angles[0])
    sin_b = math.sin(angles[1])
    cos_b = math.cos(angles[1])
    sin_c = math.sin(angles[2])
    cos_c = math.cos(angles[2])

    mat_a = np.array(((1, 0, 0), (0, cos_a, -sin_a), (0, sin_a, cos_a)))
    mat_b = np.array(((cos_b, 0, sin_b), (0, 1, 0), (-sin_b, 0, cos_b)))
    mat_c = np.array(((cos_c, -sin_c, 0), (sin_c, cos_c, 0), (0, 0, 1)))

    mat = np.matmul(np.matmul(mat_a, mat_b), mat_c)
    mat = mat.reshape(9)

    mat_str = ('{} ' * 9).strip()
    mat_str = mat_str.format(*mat)

    return mat_str


class Scenario:
    """
    The scenario class contains information for setting up and executing
    simulation scenarios along with methods to extract data during their
    execution.
    """

    def __init__(self, level, name, **options):
        """
        Instantiates a scenario instance with the given name taking place in
        the given level.

        Args:
            level (str): Name of the level to place this scenario in. This has
                         to be a level known to the simulation.
            name (str): The name of this scenario. Should be unique for the
                        level it's taking place in to avoid file collisions.
        """
        self.level = level
        self.name = name
        self.options = options

        self.path = None

        self.vehicles = dict()

        self.vehicle_states = dict()

        self.cameras = dict()

    def _find_path(self, bng):
        """
        Attempts to find the appropriate path for this scenario and creates it
        iff it does not exist.

        Args:
            bng (:class:`.BeamNGpy`): The BeamNGpy instance to find the path
                                      for.
        """
        self.path = bng.home / 'levels'
        self.path = self.path / self.level
        if not self.path.exists():
            log.warn('Level for scenario does not exist: %s', self.path)
        self.path = self.path / 'scenarios'
        if not self.path.exists():
            self.path.mkdir(parents=True)

    def _get_info_dict(self):
        """
        Generates a dictionary of information to be written to the scenario's
        files in the simulation diretory and returns it.

        Returns:
            Dictionary of information to write into the scenario files of the
            simulator.
        """
        info = dict()
        info['name'] = self.options.get('human_name', self.name)
        info['description'] = self.options.get('description', None)
        info['difficulty'] = self.options.get('difficulty', 0)
        info['authors'] = self.options.get('authors', 'BeamNGpy')

        focused = False
        vehicles_dict = dict()
        for vehicle in self.vehicles.keys():
            vehicles_dict[vehicle.vid] = {'playerUsable': True}
            if not focused:
                # Make sure one car has startFocus set
                vehicles_dict[vehicle.vid]['startFocus'] = True
                focused = True

        info['vehicles'] = vehicles_dict
        info['prefabs'] = ['levels/{}/scenarios/{}.prefab'.format(self.level,
                                                                  self.name)]

        return info

    def _get_prefab(self):
        """
        Generates prefab code to describe this scenario to the simulation
        engine and returns it as a string.

        Returns:
            Prefab code for the simulator.
        """
        template = TEMPLATE_ENV.get_template('prefab')
        vehicles = list()
        for vehicle, data in self.vehicles.items():
            pos, rot = data
            vehicle_dict = dict(vid=vehicle.vid)
            vehicle_dict.update(vehicle.options)
            vehicle_dict['position'] = ' '.join([str(p) for p in pos])
            vehicle_dict['rotationMatrix'] = compute_rotation_matrix(rot)
            vehicles.append(vehicle_dict)
        return template.render(vehicles=vehicles)

    def _write_info_file(self):
        """
        Writes the information for this scenario to an appropriate file for
        the simulator to read.
        """
        info_path = self.get_info_path()
        info_dict = self._get_info_dict()
        with open(info_path, 'w') as out_file:
            info = json.dumps([info_dict], indent=4, sort_keys=True)
            out_file.write(info)
        return info_path

    def _write_prefab_file(self):
        """
        Writes the prefab code describing this scenario to an appropriate file
        for the simulator to read.

        Returns:
            The path to the prefab file written to.
        """
        prefab_path = self.get_prefab_path()
        prefab = self._get_prefab()
        with open(prefab_path, 'w') as out_file:
            out_file.write(prefab)
        return prefab_path

    def get_info_path(self):
        """
        Returns: The path for the information file.
        """
        info_path = self.path / '{}.json'.format(self.name)
        return str(info_path)

    def get_prefab_path(self):
        """
        Returns: The path for the prefab file.
        """
        prefab_path = self.path / '{}.prefab'.format(self.name)
        return str(prefab_path)

    def add_vehicle(self, vehicle, pos=(0, 0, 0), rot=(0, 0, 0)):
        """
        Adds a vehicle to this scenario at the given position with the given
        orientation. This method has to be called before a scenario is started.

        Args:
            pos (tuple): (x,y,z) tuple specifying the position of the vehicle.
            rot (tuple): (x,y,z) tuple expressing the rotation of the vehicle
                         in Euler angles around each axis.
        """
        self.vehicles[vehicle] = (pos, rot)

    def add_camera(self, camera, name):
        self.cameras[name] = camera
        camera.attach(None, name)

    def connect(self, bng):
        for name, cam in self.cameras.items():
            cam.connect(bng, None)

    def decode_frames(self, camera_data):
        response = dict()
        for name, data in camera_data.items():
            cam = self.cameras[name]
            data = cam.decode_response(data)
            response[name] = data
        return response

    def encode_requests(self):
        requests = dict()
        for name, cam in self.cameras.items():
            request = cam.encode_engine_request()
            requests[name] = request

        requests = dict(type='SensorRequest', sensors=requests)
        return requests

    def get_engine_flags(self):
        flags = dict()
        for name, cam in self.cameras.items():
            camera_flags = cam.get_engine_flags()
            flags.update(camera_flags)
        return flags

    def make(self, bng):
        """
        Generates necessary files to describe the scenario in the simulation
        and outputs them to the simulator.

        Args:
            bng (:class:`.BeamNGpy`): The BeamNGpy instance to generate the
                                      scenario for.

        Returns:
            The path to the information file of this scenario in the simulator.
        """
        self._find_path(bng)
        self._write_prefab_file()
        info_path = self._write_info_file()
        return info_path

    def update(self, bng):
        """
        Polls sensors of every vehicle contained in the scenario.
        """
        for vehicle in self.vehicles.keys():
            bng.poll_sensors(vehicle)
