"""
.. module:: scenario
    :platform: Windows
    :synopsis: Contains the main :py:class:`.beamngpy.Scenario` class used to
               define scenarios.

.. moduleauthor:: Marc Müller <mmueller@beamng.gmbh>
.. moduleauthor:: Pascale Maul <pmaul@beamng.gmbh>
.. moduleauthor:: Sedonas <https://github.com/Sedonas>
"""

import copy
import json
import logging as log
import os
import os.path

import numpy as np

from jinja2 import Environment
from jinja2.loaders import PackageLoader

from .beamng import Level

from .beamngcommon import BNGValueError, BNGError, angle_to_quat
from .beamngcommon import compute_rotation_matrix, quat_as_rotation_mat_str
from .beamngcommon import raise_rot_deprecation_warning


TEMPLATE_ENV = Environment(loader=PackageLoader('beamngpy'))


class Road:
    """
    This class represents a DecalRoad in the environment. It contains
    information about the road's material, direction-ness of lanes,
    and geometry of the edges that make up the road.


    """

    def __init__(self, material, rid=None, interpolate=True, **options):
        """
        Creates a new road instance using the given material name. The material
        name needs to match a material that is part of the level in the
        simulator this road will be placed in.

        Args:
            material (str): Name of the material this road uses. This affects
                            how the road looks visually and needs to match a
                            material that's part of the level this road is
                            placed in.
            rid (str): Optional string setting this road's name. If specified,
                       needs to be unique with respect to other roads in the
                       level/scenario.
            interpolate (bool): Whether to apply Catmull-Rom spline
                                interpolation to smooth transition between the
                                road's nodes.
        """
        self.material = material

        self.rid = rid

        self.drivability = options.get('drivability', 1)
        self.one_way = options.get('one_way', False)
        self.flip_direction = options.get('flip_direction', False)
        self.over_objects = options.get('over_objects', True)
        self.looped = options.get('looped', False)
        self.smoothness = options.get('smoothness', 0.5)
        self.break_angle = options.get('break_angle', 3)
        self.texture_length = options.get('texture_length', 5)
        self.render_priority = options.get('render_priority', 10)

        self.one_way = '1' if self.one_way else '0'
        self.flip_direction = '1' if self.flip_direction else '0'
        self.over_objects = '1' if self.over_objects else '0'
        self.looped = '1' if self.looped else '0'

        if interpolate:
            self.improved_spline = '1'
        else:
            self.improved_spline = '0'
            self.break_angle = 359.9

        self.nodes = list()


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

        return ScenarioObject(oid, name, otype, pos, rot_quat, scale, **d)

    def __init__(self, oid, name, otype, pos, rot, scale,
                 rot_quat=None, **options):
        self.id = oid
        self.name = name
        self.type = otype
        self.pos = pos
        if rot:
            raise_rot_deprecation_warning()
            rot_quat = angle_to_quat(rot)
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


class DecalRoad(SceneObject):
    def __init__(self, options):
        super(DecalRoad, self).__init__(options)
        self.lines = options.get('lines', [])

        self.annotation = options.get('annotation', None)
        self.detail = options.get('Detail', None)
        self.material = options.get('Material', None)
        self.break_angle = options.get('breakAngle', None)
        self.drivability = options.get('drivability', None)
        self.flip_direction = options.get('flipDirection', False)
        self.improved_spline = options.get('improvedSpline', False)
        self.lanes_left = options.get('lanesLeft', None)
        self.lanes_right = options.get('lanesRight', None)
        self.one_way = options.get('oneWay', False)
        self.over_objects = options.get('overObjects', False)


class StaticObject(ScenarioObject):
    def __init__(self, name, pos, rot, scale, shape, rot_quat=None):
        super(StaticObject, self).__init__(name, None, 'TSStatic',
                                           pos, rot, scale, rot_quat=rot_quat,
                                           shapeName=shape)


class ProceduralMesh(ScenarioObject):
    def __init__(self, pos, rot, name, material, rot_quat=None):
        super(ProceduralMesh, self).__init__(name, name, 'ProceduralMesh',
                                             pos, rot, (1, 1, 1),
                                             rot_quat=rot_quat)
        self.material = material

    def place(self, bng):
        raise NotImplementedError()


class ProceduralCylinder(ProceduralMesh):
    def __init__(self, pos, rot, radius, height, name,
                 rot_quat=None, material=None):
        super(ProceduralCylinder, self).__init__(pos, rot, name, material,
                                                 rot_quat=rot_quat)
        self.radius = radius
        self.height = height

    def place(self, bng):
        bng.create_cylinder(self.name, self.radius, self.height, self.pos,
                            None, material=self.material, rot_quat=self.rot)


class ProceduralBump(ProceduralMesh):
    def __init__(self, pos, rot, width, length, height, upper_length,
                 upper_width, name, rot_quat=None, material=None):
        super(ProceduralBump, self).__init__(pos, rot, name, material,
                                             rot_quat=rot_quat)
        self.width = width
        self.length = length
        self.height = height
        self.upper_length = upper_length
        self.upper_width = upper_width

    def place(self, bng):
        bng.create_bump(self.name, self.width, self.length, self.height,
                        self.upper_length, self.upper_width, self.pos,
                        None, material=self.material, rot_quat=self.rot)


class ProceduralCone(ProceduralMesh):
    def __init__(self, pos, rot, radius, height, name,
                 rot_quat=None, material=None):
        super(ProceduralCone, self).__init__(pos, rot, name, material,
                                             rot_quat=rot_quat)
        self.radius = radius
        self.height = height

    def place(self, bng):
        bng.create_cone(self.name, self.radius, self.height, self.pos,
                        None, material=self.material, rot_quat=self.rot)


class ProceduralCube(ProceduralMesh):
    def __init__(self, pos, rot, size, name, rot_quat=None, material=None):
        super(ProceduralCube, self).__init__(pos, rot, name, material,
                                             rot_quat=rot_quat)
        self.size = size

    def place(self, bng):
        bng.create_cube(self.name, self.size, self.pos, None,
                        material=self.material, rot_quat=self.rot)


class ProceduralRing(ProceduralMesh):
    def __init__(self, pos, rot, radius, thickness, name,
                 rot_quat=None, material=None):
        super(ProceduralRing, self).__init__(pos, rot, name, material,
                                             rot_quat=rot_quat)
        self.radius = radius
        self.thickness = thickness

    def place(self, bng):
        bng.create_ring(self.name, self.radius, self.thickness, self.pos,
                        None, material=self.material, rot_quat=self.rot)


class Scenario:
    """
    The scenario class contains information for setting up and executing
    simulation scenarios along with methods to extract data during their
    execution.
    """

    game_classes = {
        'MissionGroup': lambda d: SceneObject(d),
        'DecalRoad': lambda d: DecalRoad(d),
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

        self.roads = list()
        self.waypoints = list()
        self.checkpoints = list()
        self.proc_meshes = list()
        self.objects = list()

        self.cameras = dict()

        self.scene = None

        self.bng = None

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

        focused = False
        vehicles_dict = dict()
        for vehicle in self.vehicles:
            vehicles_dict[vehicle.vid] = {'playerUsable': True}
            if not focused:
                # Make sure one car has startFocus set
                vehicles_dict[vehicle.vid]['startFocus'] = True
                focused = True

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
        objs = self._get_objects_list()

        return template.render(vehicles=vehicles, roads=roads, objects=objs)

    def _get_level_name(self):
        if isinstance(self.level, Level):
            return self.level.name
        else:
            return self.level

    def add_object(self, obj):
        """
        Adds an extra object to be placed in the prefab. Objects are expected
        to be :class:`.ScenarioObject` instances with additional, type-
        specific properties in that class's opts dictionary.
        """
        self.objects.append(obj)

    def add_vehicle(self, vehicle, pos=(0, 0, 0),
                    rot=None, rot_quat=(0, 0, 0, 1), cling=True):
        """
        Adds a vehicle to this scenario at the given position with the given
        orientation. This method has to be called before a scenario is started.

        Args:
            pos (tuple): (x,y,z) tuple specifying the position of the vehicle.
            rot (tuple): (x,y,z) tuple expressing the rotation of the vehicle
                         in Euler angles around each axis.
            rot_quat (tuple): Optional tuple (x, y, z, w) specifying the
                              rotation as quaternion
        """
        if self.name == vehicle.vid:
            error = 'Cannot have vehicle with the same name as the scenario:' \
                ' Scenario={}, Vehicle={}'.format(self.name, vehicle.vid)
            raise BNGValueError(error)

        if rot:
            raise_rot_deprecation_warning()
            rot_quat = angle_to_quat(rot)
        self.vehicles.add(vehicle)
        self._vehicle_locations[vehicle.vid] = (pos, rot_quat)

        if self.bng:
            self.bng.spawn_vehicle(vehicle, pos, None, rot_quat=rot_quat,
                                   cling=cling)  # todo
            self.transient_vehicles.add(vehicle)
            vehicle.connect(self.bng)

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
                self.transient_vehicles.remove(vehicle)

            if vehicle.vid in self._vehicle_locations:
                del self._vehicle_locations[vehicle.vid]
            self.vehicles.remove(vehicle)

    def get_vehicle(self, needle):
        """
        Retrieves the vehicle with the given ID from this scenario.

        Args:
            needle (str): The ID of the vehicle to find.

        Returns:
            The :class:`.Vehicle` with the given ID. None if it wasn't found.
        """
        for vehicle in self.vehicles:
            if vehicle.vid == needle:
                return vehicle
        return None

    def add_road(self, road):
        """
        Adds a :class:`.Road` to this scenario.
        """
        self.roads.append(road)

    def add_camera(self, camera, name):
        """
        Adds a :class:`beamngpy.sensors.Camera` to this scenario which can be
        used to obtain rendered frames from a location in the world (e.g.
        something like a surveillance camera.)

        Args:
            camera (:class:`beamngpy.sensors.Camera` ): The camera to add.
            name (str): The name the camera should be identified with.
        """
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
        options = dict(rot=None,
                       rot_quat=(0, 0, 0, 1),
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

    def connect(self, bng):
        """
        Connects this scenario to the simulator, hooking up any cameras to
        their counterpart in the simulator.
        """
        self.bng = bng

        for mesh in self.proc_meshes:
            mesh.place(self.bng)

        for _, cam in self.cameras.items():
            cam.connect(self.bng, None)

        for vehicle in self.vehicles:
            vehicle.connect(bng)

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

        Returns:
            The path to the information file of this scenario in the simulator.
        """
        level_name = self._get_level_name()

        prefab = self._get_prefab()
        info = self._get_info_dict()

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
                del self.vehicles[vehicle]

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

    def render_cameras(self):
        """
        Renders images for each of the cameras place in this scenario.

        Returns:
            A dictionary mapping camera names to color, annotation, or depth
            images, depending on how each camera is set up.

        Raises:
            BNGError: If the scenario is currently not loaded.
        """
        if not self.bng:
            raise BNGError('Scenario needs to be loaded into a BeamNGpy '
                           'instance for rendering cameras.')

        return self.bng.render_cameras()
