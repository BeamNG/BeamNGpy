"""
.. module:: scenario
    :platform: Windows
    :synopsis: Contains the main :py:class:`.Scenario` class used to define
    scenarios.
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

    def __init__(self, level, name, **options):
        self.level = level
        self.name = name
        self.options = options

        self.path = None

        self.vehicles = dict()

    def _find_path(self, bng):
        self.path = bng.home / 'levels'
        self.path = self.path / self.level
        if not self.path.exists():
            log.warn('Level for scenario does not exist: %s', self.path)
        self.path = self.path / 'scenarios'
        if not self.path.exists():
            self.path.mkdir(parents=True)

    def _get_info_dict(self):
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
        info_path = self.get_info_path()
        info_dict = self._get_info_dict()
        with open(info_path, 'w') as out_file:
            info = json.dumps([info_dict], indent=4, sort_keys=True)
            out_file.write(info)
        return info_path

    def _write_prefab_file(self):
        prefab_path = self.get_prefab_path()
        prefab = self._get_prefab()
        with open(prefab_path, 'w') as out_file:
            out_file.write(prefab)
        return prefab_path

    def get_info_path(self):
        info_path = self.path / '{}.json'.format(self.name)
        return str(info_path)

    def get_prefab_path(self):
        prefab_path = self.path / '{}.prefab'.format(self.name)
        return str(prefab_path)

    def add_vehicle(self, vehicle, pos=(0, 0, 0), rot=(0, 0, 0)):
        self.vehicles[vehicle] = (pos, rot)

    def make(self, bng):
        self._find_path(bng)
        self._write_prefab_file()
        info_path = self._write_info_file()
        return info_path
