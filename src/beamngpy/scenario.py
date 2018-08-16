"""
.. module:: scenario
    :platform: Windows
    :synopsis: Contains the main :py:class:`.Scenario` class used to define
    scenarios.
"""

import json
import logging as log

from pathlib import Path


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

        vehicles_dict = dict()
        for vehicle in self.vehicles.keys():
            vehicles_dict[vehicle.vid] = {'playerUsable': True}
        info['vehicles'] = vehicles_dict

        return info

    def _write_info_file(self):
        info_path = self.get_info_path()
        info_dict = self._get_info_dict()
        with open(info_path, 'w') as out_file:
            info = json.dumps([info_dict], indent=4, sort_keys=True)
            out_file.write(info)
        return info_path

    def get_info_path(self):
        info_path = self.path / '{}.json'.format(self.name)
        return str(info_path)

    def add_vehicle(self, vehicle, pos=(0, 0, 0)):
        self.vehicles[vehicle] = pos

    def make(self, bng):
        self._find_path(bng)
        # self.write_prefab()
        info_path = self._write_info_file()
        return info_path
