"""
.. module:: scenario
    :platform: Windows
    :synopsis: Contains the main :py:class:`.beamngpy.Scenario` class used to
               define scenarios.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
.. moduleauthor:: Sedonas <https://github.com/Sedonas>
.. moduleauthor:: Adam Ivora <aivora@beamng.gmbh>
"""

import copy
from logging import DEBUG, getLogger

from jinja2 import Environment
from jinja2.loaders import PackageLoader

from .beamng import Level
from .beamngcommon import (LOGGER_ID, BNGError, BNGValueError,
                           create_warning, quat_as_rotation_mat_str)

TEMPLATE_ENV = Environment(loader=PackageLoader('beamngpy'))


module_logger = getLogger(f'{LOGGER_ID}.scenario')
module_logger.setLevel(DEBUG)


class Scenario:
    """
    The scenario class contains information for setting up and executing
    simulation scenarios along with methods to extract data during their
    execution.
    """

    game_classes = {
        'MissionGroup': lambda d: SceneObject(d),
        'DecalRoad': lambda d: __import__("beamngpy").DecalRoad(d),
    }

    @staticmethod
    def from_dict(d):
        if 'sourceFile' in d:
            path = d['sourceFile']
            del d['sourceFile']
        else:
            path = None

        if 'levelName' in d:
            level = d['levelName']
            del d['levelName']
        else:
            level = 'unknown'

        if 'name' in d:
            name = d['name']
            del d['name']
        else:
            name = 'unknown'

        scenario = Scenario(level, name, path, **d)

        return scenario

    def __init__(self, level, name, path=None, **options):
        """
        Instantiates a scenario instance with the given name taking place in
        the given level.

        Args:
            level: Either the name of the level this scenario takes place in
                   as a string or as an instance of :class:`.Level`
            name (str): The name of this scenario. Should be unique for the
                        level it's taking place in to avoid file collisions.
        """
        self.level = level
        self.name = name
        self.path = path
        self.options = options

        self.vehicles = set()
        self.transient_vehicles = set()  # Vehicles added during scenario
        self._vehicle_locations = {}
        self._focus_vehicle = None

        self.roads = list()
        self.mesh_roads = list()
        self.waypoints = list()
        self.checkpoints = list()
        self.proc_meshes = list()
        self.objects = list()

        self.cameras = dict()

        self.scene = None

        self.bng = None

        self.logger = getLogger(f'{LOGGER_ID}.Scenario')
        self.logger.setLevel(DEBUG)

    def _get_objects_list(self):
        """
        Encodes extra objects to be placed in the scene as dictionaries for the
        prefab template.

        Returns:
            A list of dictionaries representing :class:`.ScenarioObject`
            instances to be placed in the prefab.
        """
        objs = list()
        for obj in self.objects:
            obj_dict = dict(type=obj.type, id=obj.id)
            obj_dict['options'] = copy.deepcopy(obj.opts)

            pos_str = '{} {} {}'.format(*obj.pos)
            rot_mat = quat_as_rotation_mat_str(obj.rot)
            scale_str = '{} {} {}'.format(*obj.scale)
            obj_dict['options']['position'] = pos_str
            obj_dict['options']['rotationMatrix'] = rot_mat
            obj_dict['options']['scale'] = scale_str

            objs.append(obj_dict)
        self.logger.debug(f'The scenario {self.name} has {len(objs)} '
                          'objects of type `beamngpy.ScenarioObject`')
        return objs

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
        info['lapConfig'] = self.checkpoints

        vehicles_dict = dict()
        for vehicle in self.vehicles:
            vehicles_dict[vehicle.vid] = {'playerUsable': True}

        if self.vehicles:
            if self._focus_vehicle is None:
                self._focus_vehicle = next(iter(self.vehicles)).vid

            vehicles_dict[self._focus_vehicle]['startFocus'] = True

        info['vehicles'] = vehicles_dict
        info['prefabs'] = ['levels/{}/scenarios/{}.prefab'.format(self.level,
                                                                  self.name)]

        return info

    def _get_vehicles_list(self):
        """
        Gets the vehicles contained in this scenario encoded as a dict and
        put into one list, including their position and rotation as a matrix
        ready to be placed in the simulator.

        Returns:
            All vehicles as a dict including position and rotation.
        """
        vehicles = list()
        for vehicle in self.vehicles:
            pos, rot = self._vehicle_locations[vehicle.vid]
            vehicle_dict = dict(vid=vehicle.vid)
            vehicle_dict.update(vehicle.options)
            vehicle_dict['position'] = ' '.join([str(p) for p in pos])
            vehicle_dict['rotationMatrix'] = quat_as_rotation_mat_str(rot)
            vehicles.append(vehicle_dict)
        vehicle_names = [v.vid for v in self.vehicles]
        self.logger.debug(f'The scenario {self.name} has {len(vehicles)} '
                          f'vehicles: {", ".join(vehicle_names)}')
        return vehicles

    def _get_roads_list(self):
        """
        Gets the roads defined in this scenario encoded as a dict and put into
        one list ready to be placed in the simulator.

        Returns:
            All roads encoded as a dict in one list.
        """
        ret = list()
        for idx, road in enumerate(self.roads):
            road_dict = dict(**road.__dict__)

            if road.rid is None:
                road_id = 'beamngpy_road_{}_{:03}'.format(self.name, idx)
            else:
                road_id = road.rid
            road_dict['road_id'] = road_id
            road_dict['render_priority'] = idx

            ret.append(road_dict)
        self.logger.debug(f'The scenario {self.name} has {len(ret)} '
                          'scenario-specific roads.')
        return ret

    def _get_mesh_roads_list(self):
        """
        Gets the mesh roads defined in this scenario encoded as a dict and put into
        one list ready to be placed in the simulator.

        Returns:
            All mesh roads encoded as a dict in one list.
        """
        ret = list()
        for idx, road in enumerate(self.mesh_roads):
            road_dict = dict(**road.__dict__)

            if road.rid is None:
                road_id = 'beamngpy_mesh_road_{}_{:03}'.format(self.name, idx)
            else:
                road_id = road.rid
            road_dict['road_id'] = road_id
            road_dict['render_priority'] = idx

            ret.append(road_dict)
        self.logger.debug(f'The scenario {self.name} has {len(ret)} '
                          'scenario-specific mesh roads.')
        return ret

    def _get_prefab(self):
        """
        Generates prefab code to describe this scenario to the simulation
        engine and returns it as a string.

        Returns:
            Prefab code for the simulator.
        """
        template = TEMPLATE_ENV.get_template('prefab')

        vehicles = self._get_vehicles_list()
        roads = self._get_roads_list()
        mesh_roads = self._get_mesh_roads_list()
        objs = self._get_objects_list()

        return template.render(vehicles=vehicles, roads=roads, mesh_roads=mesh_roads, objects=objs)

    def _get_level_name(self):
        if isinstance(self.level, Level):
            return self.level.name
        else:
            return self.level

    def _get_existing_vehicles(self, bng):
        current_vehicles = set(bng.get_current_vehicles().values())
        self.logger.debug(
            f'Got {len(current_vehicles)} vehicles from scenario.')
        self.transient_vehicles = current_vehicles.copy()

        for vehicle in self.vehicles:
            self.transient_vehicles.discard(vehicle)
            current_vehicles.discard(vehicle)
            current_vehicles.add(vehicle)

        self.vehicles = current_vehicles

    def add_object(self, obj):
        """
        Adds an extra object to be placed in the prefab. Objects are expected
        to be :class:`.ScenarioObject` instances with additional, type-
        specific properties in that class's opts dictionary.
        """
        self.objects.append(obj)

    def add_vehicle(self, vehicle, pos=(0, 0, 0), rot_quat=(0, 0, 0, 1), cling=True):
        """
        Adds a vehicle to this scenario at the given position with the given
        orientation.

        Args:
            pos (tuple): (x,y,z) tuple specifying the position of the vehicle.
            rot_quat (tuple, optional): (x, y, z, w) tuple specifying
                                        the rotation as quaternion
        """
        if self.name == vehicle.vid:
            error = 'Cannot have vehicle with the same name as the scenario:' \
                    f' Scenario={self.name}, Vehicle={vehicle.vid}'
            raise BNGValueError(error)

        if vehicle in self.vehicles:
            error = f'The vehicle \'{vehicle.vid}\' is already in the scenario.'
            raise BNGValueError(error)

        if vehicle.connection:
            vehicle.disconnect()

        self.vehicles.add(vehicle)
        self._vehicle_locations[vehicle.vid] = (pos, rot_quat)
        self.logger.debug(f'Added vehicle with id \'{vehicle.vid}\'.')

        if self.bng:
            self.bng.spawn_vehicle(
                vehicle, pos, rot_quat=rot_quat, cling=cling)
            self.transient_vehicles.add(vehicle)
            vehicle.connect(self.bng)
        else:
            self.logger.debug('No beamngpy instance available. '
                              f'Did not spawn vehicle with id \'{vehicle.vid}\'.')

    def remove_vehicle(self, vehicle):
        """
        Removes the given :class:`.Vehicle`: from this scenario. If the
        scenario is currently loaded, the vehicle will be despawned.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to remove.
        """
        if vehicle in self.vehicles:
            if self.bng:
                self.bng.despawn_vehicle(vehicle)
                self.transient_vehicles.discard(vehicle)
            else:
                self.logger.debug('No beamngpy instance available, cannot '
                                  f'despawn vehicle with id \'{vehicle.vid}\'')

            if vehicle.vid in self._vehicle_locations:
                del self._vehicle_locations[vehicle.vid]
            self.vehicles.remove(vehicle)
        else:
            self.logger.debug(f'No vehicle with id {vehicle.vid} found.')

    def get_vehicle(self, vehicle_id):
        """
        Retrieves the vehicle with the given ID from this scenario.

        Args:
            vehicle_id (str): The ID of the vehicle to find.

        Returns:
            The :class:`.Vehicle` with the given ID. None if it wasn't found.
        """
        for vehicle in self.vehicles:
            if vehicle.vid == vehicle_id:
                return vehicle
        self.logger.debug(f'Could not find vehicle with id {vehicle_id}')
        return None

    def set_initial_focus(self, vehicle_id):
        """defines which vehicle has the initial focus

        Args:
            vehicle_id (string): vehicle id of focussed vehicle
        """
        self._focus_vehicle = vehicle_id

    def add_road(self, road):
        """Adds a road to this scenario.

        Args:
            road (:class:`beamngpy.Road`): road to be added to the scenario.
        """
        self.roads.append(road)

    def add_mesh_road(self, road):
        """Adds a mesh road to this scenario.

        Args:
            road (:class:`beamngpy.MeshRoad`): mesh road to be added to the scenario.
        """
        self.mesh_roads.append(road)

    def add_camera(self, camera, name):
        """
        Adds a :class:`beamngpy.sensors.Camera` to this scenario which can be
        used to obtain rendered frames from a location in the world (e.g.
        something like a surveillance camera.)

        Args:
            camera (:class:`beamngpy.sensors.Camera` ): The camera to add.
            name (str): The name the camera should be identified with.
        """
        if name in self.cameras.keys():
            raise BNGValueError('One scenario cannot have multiple cameras'
                                f'with the same name: "{name}"')
        self.cameras[name] = camera
        camera.attach(None, name)

    def add_procedural_mesh(self, mesh):
        """
        Adds a :class:`.ProceduralMesh` to be placed in world to the scenario.

        Args:
            mesh (:class:`.ProceduralMesh`): The mesh to place.
        """
        self.proc_meshes.append(mesh)
        if self.bng:
            mesh.place(self.bng)

    def add_checkpoints(self, positions, scales, ids=None):
        """
        Adds checkpoints to the scenario.

        Args:
            positions(list): positions (tuple of length 3) of individual points
            scales(list): scale (tuple of length 3) of individual points
            ids(list): optional, names of the individual points
        """
        if ids is None:
            ids = [f"wp{i}" for i in range(len(positions))]
        assert(len(positions) == len(scales) == len(ids))
        options = dict(rot_quat=(0, 0, 0, 1),
                       drawDebug='0',
                       directionalWaypoint='0',
                       mode='Ignore',
                       canSave='1',
                       canSaveDynamicFields='1')
        for oid, p, s in zip(ids, positions, scales):
            cp = ScenarioObject(oid=oid,
                                name=oid,
                                otype='BeamNGWaypoint',
                                pos=p,
                                scale=s,
                                **options)
            self.add_object(cp)
        self.checkpoints.extend(ids)

    def _convert_scene_object(self, obj):
        data = self.bng.get_scene_object_data(obj['id'])
        clazz = data['class']
        if clazz in Scenario.game_classes:
            converted = Scenario.game_classes[clazz](data)
        else:
            converted = SceneObject(data)

        if 'children' in obj:
            for child in obj['children']:
                child = self._convert_scene_object(child)
                converted.children.append(child)

        return converted

    def _fill_scene(self):
        scenetree = self.bng.get_scenetree()
        assert scenetree['class'] == 'SimGroup'
        self.scene = self._convert_scene_object(scenetree)

    def sync_scene(self):
        """
        Retrieves the current scene tree of the scenario from the simulator,
        converting them into the most appropriate known (sub)class of
        :class:`.SceneObject`. The result is not returned but rather stored
        in the ``scene`` field of this class.
        """
        self._fill_scene()

    def connect(self, bng, connect_existing=True):
        """
        Connects this scenario to the simulator, hooking up any cameras to
        their counterpart in the simulator.

        Args:
            bng (:class:`.BeamNGpy`): The BeamNGpy instance to generate the
                                      scenario for.
            connect_existing (bool): Whether vehicles spawned already
                                     in the scenario should be connected to
                                     this (:class:``.Scenario``) instance.
        """
        self.bng = bng

        self.logger.debug(f'{len(self.proc_meshes)} procedural meshes.')
        for mesh in self.proc_meshes:
            mesh.place(self.bng)

        self.logger.debug(f'Connecting to {len(self.cameras)} cameras.')
        for _, cam in self.cameras.items():
            cam.connect(self.bng, None)

        if connect_existing:
            self._get_existing_vehicles(bng)

        self.logger.debug(f'Connecting to {len(self.vehicles)} vehicles.')
        for vehicle in self.vehicles:
            vehicle.connect(bng)

        self.logger.info(f'Connected to scenario: {self.name}')

    def decode_frames(self, camera_data):
        """
        Decodes raw camera sensor data as a :class:`.Image`
        """
        response = dict()
        for name, data in camera_data.items():
            cam = self.cameras[name]
            data = cam.decode_response(data)
            response[name] = data
        return response

    def encode_requests(self):
        """
        Encodes the sensor requests of cameras placed in this scenario for the
        simulator.

        Returns:
            Dictionary of camera names to their corresponding sensor requests.
        """
        requests = dict()
        for name, cam in self.cameras.items():
            request = cam.encode_engine_request()
            requests[name] = request
            self.logger.debug('Added engine request for '
                              f'camera with id <{name}>.')

        requests = dict(type='SensorRequest', sensors=requests)
        return requests

    def get_engine_flags(self):
        """
        Gathers engine flags to set for cameras in this scenario to work.

        Returns:
            Dictionary of flag names to their state.
        """
        flags = dict()
        for _, cam in self.cameras.items():
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

        Raises:
            BNGError: If the scenario already has set its info .json file included.
        """
        if self.path is not None:
            raise BNGError('This scenario already has an info file.')

        level_name = self._get_level_name()

        prefab = self._get_prefab()
        info = self._get_info_dict()
        self.logger.debug(f'Generated prefab:\n{prefab}\n')
        self.logger.debug(f'Generated scenarios info dict:\n{info}\n')

        self.path = bng.create_scenario(level_name, self.name, prefab, info)

    def find(self, bng):
        """
        Looks for the files of an existing scenario and returns the path to the
        info file of this scenario, iff one is found.

        Args:
            bng (:class:`.BeamNGpy`): The BeamNGpy instance to look for the
                                      scenario in.

        Returns:
            The path to the information file of his scenario found in the
            simulator as a string, None if it could not be found.
        """
        scenarios = bng.get_level_scenarios(self.level)
        for path, scenario in scenarios:
            if scenario.name == self.name and scenario.level == self.level:
                self.path = path
        return self.path

    def delete(self, bng):
        """
        Deletes files created by this scenario from the given
        :class:`.BeamNGpy`'s home/user path.
        """
        if self.path is None:
            self.find(bng)
        bng.delete_scenario(self.path)
        self.logger.info(f'Deleted scenario from simulation: "{self.name}".')

    def start(self):
        """
        Starts this scenario. Requires the scenario to be loaded into a
        running :class:`.BeamNGpy` instance first.

        Raises:
            BNGError: If the scenario is not loaded.
        """
        if not self.bng:
            raise BNGError('Scenario needs to be loaded into a BeamNGpy '
                           'instance to be started.')

        self.bng.start_scenario()
        self.logger(f'Started scenario: "{self.name}"')

    def restart(self):
        """
        Restarts this scenario. Requires the scenario to be loaded into a
        running :class:`.BeamNGpy` instance first.

        Notes:
            If any vehicles have been added during the scenario after it has
            been started, they will be removed as the scenario is reset to
            its original state.

        Raises:
            BNGError: If the scenario has not been loaded.
        """
        if not self.bng:
            raise BNGError('Scenario needs to be loaded into a BeamNGpy '
                           'instance to be restarted.')

        while self.transient_vehicles:
            vehicle = self.transient_vehicles.pop()
            if vehicle in self.vehicles:
                self.bng.despawn_vehicle(vehicle)
                self.vehicles.discard(vehicle)
        self.logger.info(f'Restarted scenario: "{self.name}"')

    def close(self):
        """
        Closes open connections and allocations of the scenario.
        """
        if not self.bng:
            raise BNGError('Scenario needs to be loaded into a BeamNGpy '
                           'instance to be stopped.')

        for vehicle in self.vehicles:
            vehicle.close()

        self.bng = None
        self.logger.debug('Removed beamngpy instance from scenario class.')

    def find_waypoints(self):
        """
        Finds waypoints placed in the world right now.

        Returns:
            A list of :class:`.ScenarioObject` containing waypoints found in
            the world.

        Raises:
            BNGError: If the scenario is not currently loaded.
        """
        if not self.bng:
            raise BNGError('Scenario needs to be loaded into a BeamNGpy '
                           'instance to find objects.')

        return self.bng.find_objects_class('BeamNGWaypoint')

    def find_procedural_meshes(self):
        """
        Finds procedural meshes placed in the world right now.

        Returns:
            A list of :class:`.ScenarioObject` containing procedural meshes
            found in the world.

        Raises:
            BNGError: If the scenario is not currently loaded.
        """
        if not self.bng:
            raise BNGError('Scenario needs to be loaded into a BeamNGpy '
                           'instance to find objects.')

        return self.bng.find_objects_class('ProceduralMesh')

    def find_static_objects(self):
        """
        Finds static objects placed in the world right now.

        Returns:
            A list of :class:`.ScenarioObject` containing statically placed
            objects found in the world.

        Raises:
            BNGError: If the scenario is not currently loaded.
        """
        if not self.bng:
            raise BNGError('Scenario needs to be loaded into a BeamNGpy '
                           'instance to find objects.')

        return self.bng.find_objects_class('TSStatic')

    def update(self):
        """
        Synchronizes object states of this scenario with the simulator. For
        example, this is used to update the :attr:`.Vehicle.state` fields of
        each vehicle in the scenario.

        Raises:
            BNGError: If the scenario is currently not loaded.
        """
        if not self.bng:
            raise BNGError('Scenario needs to be loaded into a BeamNGpy '
                           'instance to update its state.')

        self.bng.update_scenario()


class ScenarioObject:
    """
    This class is used to represent objects in the simulator's environment. It
    contains basic information like the object type, position, rotation, and
    scale.
    """

    @staticmethod
    def from_game_dict(d):
        oid = None
        name = None
        otype = None
        pos = None
        rot_quat = None
        scale = None
        if 'id' in d:
            oid = d['id']
            del d['id']

        if 'name' in d:
            name = d['name']
            del d['name']

        if 'class' in d:
            otype = d['class']
            del d['class']

        if 'pos' in d:
            pos = d['position']
            del d['position']

        if 'rot' in d:
            rot_quat = d['rotation']
            del d['rotation']

        if 'scale' in d:
            scale = d['scale']
            del d['scale']

        return ScenarioObject(oid, name, otype, pos, scale, rot_quat, **d)

    def __init__(self, oid, name, otype, pos, scale, rot_quat=None, **options):
        """Creates a scenario object with the given parameters.

        Args:
            oid (string): name of the asset
            name (string): asset id
            otype (string): type of the object according to the BeamNG classification
            pos (tuple): x, y, and z coordinates
            scale (tuple): defining the scale along the x,y, and z axis.
            rot_quat (tuple, optional): Quatertnion describing the initial orientation. Defaults to None.
        """
        self.id = oid
        self.name = name
        self.type = otype
        self.pos = pos
        self.rot = rot_quat
        self.scale = scale
        self.opts = options
        self.children = []

    def __eq__(self, other):
        if isinstance(other, type(self)):
            return self.id == other.id

        return False

    def __hash__(self):
        return hash(self.id)

    def __str__(self):
        s = '{} [{}:{}] @ ({:5.2f}, {:5.2f}, {:5.2f})'
        s = s.format(self.type, self.id, self.name, *self.pos)
        return s

    def __repr__(self):
        return str(self)


class SceneObject:
    def __init__(self, options):
        self.id = options.get('id', None)
        if 'id' in options:
            del options['id']

        self.name = options.get('name', None)
        if 'name' in options:
            del options['name']

        self.type = options.get('class', None)
        if 'type' in options:
            del options['type']

        self.pos = options.get('position', [0, 0, 0])
        if 'position' in options:
            del options['position']

        self.rot = options.get('rotation', [0, 0, 0, 0])
        if 'rotation' in options:
            del options['rotation']

        self.scale = options.get('scale', [0, 0, 0])
        if 'scale' in options:
            del options['scale']

        self.options = options
        self.children = []

    def __eq__(self, other):
        if isinstance(other, type(self)):
            return self.id == other.id

        return False

    def __hash__(self):
        return hash(self.id)

    def __str__(self):
        s = '{} [{}:{}] @ ({:5.2f}, {:5.2f}, {:5.2f})'
        s = s.format(self.type, self.id, self.name, *self.pos)
        return s

    def __repr__(self):
        return str(self)


class StaticObject(ScenarioObject):
    def __init__(self, name, pos, scale, shape, rot_quat=None):
        super(StaticObject, self).__init__(name, None, 'TSStatic',
                                           pos, scale, rot_quat=rot_quat,
                                           shapeName=shape)
