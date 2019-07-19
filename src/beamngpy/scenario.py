"""
.. module:: scenario
    :platform: Windows
    :synopsis: Contains the main :py:class:`.beamngpy.Scenario` class used to
               define scenarios.
"""

import copy
import json
import logging as log
import math
import os
import os.path

from pathlib import Path

import numpy as np

from jinja2 import Environment
from jinja2.loaders import PackageLoader

from .beamngcommon import BNGValueError, BNGError


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
            interpolat (bool): Whether to apply Catmull-Rom spline
                               interpolation to smooth transition between the
                               road's nodes.
        """
        self.material = material

        self.rid = rid

        self.drivability = options.get('drivability', 1)
        self.one_way = options.get('one_way', False)
        self.flip_direction = options.get('flip_direction', False)
        self.looped = options.get('looped', False)
        self.smoothness = options.get('smoothness', 0.5)
        self.break_angle = options.get('break_angle', 3)
        self.texture_length = options.get('texture_length')

        self.one_way = '1' if self.one_way else '0'
        self.flip_direction = '1' if self.flip_direction else '0'
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

    def __init__(self, oid, name, otype, pos, rot, scale, **options):
        self.id = oid
        self.name = name
        self.type = otype
        self.position = pos
        self.rotation = rot
        self.scale = scale
        self.opts = options

    def __eq__(self, other):
        if isinstance(other, type(self)):
            return self.id == other.id

        return False

    def __hash__(self):
        return hash(self.id)

    def __str__(self):
        s = '{} [{}:{}] @ ({:5.2f}, {:5.2f}, {:5.2f})'
        s = s.format(self.type, self.id, self.name, *self.position)
        return s

    def __repr__(self):
        return str(self)


class StaticObject(ScenarioObject):
    def __init__(self, name, pos, rot, scale, shape):
        rot_mat = compute_rotation_matrix(rot)
        pos_str = '{} {} {}'.format(*pos)
        scale_str = '{} {} {}'.format(*scale)
        super(StaticObject, self).__init__(name, None, 'TSStatic',
                                           pos, rot, scale, shapeName=shape)


class ProceduralMesh:
    def __init__(self, pos, rot, name, material):
        self.name = name
        self.material = material
        self.pos = pos
        self.rot = rot

    def place(self, bng):
        raise NotImplementedError()


class ProceduralCylinder(ProceduralMesh):
    def __init__(self, pos, rot, radius, height, name=None, material=None):
        super(ProceduralCylinder, self).__init__(pos, rot, name, material)
        self.radius = radius
        self.height = height

    def place(self, bng):
        bng.create_cylinder(self.radius, self.height, self.pos, self.rot,
                            self.material, self.name)


class ProceduralBump(ProceduralMesh):
    def __init__(self, pos, rot, width, length, height,
                 upper_length, upper_width, name=None, material=None):
        super(ProceduralBump, self).__init__(pos, rot, name, material)
        self.width = width
        self.length = length
        self.height = height
        self.upper_length = upper_length
        self.upper_width = upper_width

    def place(self, bng):
        bng.create_bump(self.width, self.length, self.height,
                        self.upper_length, self.upper_width, self.pos,
                        self.rot, self.material, self.name)


class ProceduralCone(ProceduralMesh):
    def __init__(self, pos, rot, radius, height, name=None, material=None):
        super(ProceduralCone, self).__init__(pos, rot, name, material)
        self.radius = radius
        self.height = height

    def place(self, bng):
        bng.create_cone(self.radius, self.height, self.pos, self.rot,
                        self.material, self.name)


class ProceduralCube(ProceduralMesh):
    def __init__(self, pos, rot, size, name=None, material=None):
        super(ProceduralCube, self).__init__(pos, rot, name, material)
        self.size = size

    def place(self, bng):
        bng.create_cube(self.size, self.pos, self.rot,
                        self.material, self.name)


class ProceduralRing(ProceduralMesh):
    def __init__(self, pos, rot, radius, thickness, name=None, material=None):
        super(ProceduralRing, self).__init__(pos, rot, name, material)
        self.radius = radius
        self.thickness = thickness

    def place(self, bng):
        bng.create_ring(self.radius, self.thickness, self.pos, self.rot,
                        self.material, self.name)


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
        self.transient_vehicles = set()  # Vehicles added during scenario

        self.roads = list()
        self.waypoints = list()
        self.proc_meshes = list()
        self.objects = list()

        self.cameras = dict()

        self.bng = None

    def _find_path(self, bng):
        """
        Attempts to find the appropriate path for this scenario and creates it
        iff it does not exist.

        Args:
            bng (:class:`.BeamNGpy`): The BeamNGpy instance to find the path
                                      for.
        """
        if bng.user is not None:
            self.path = bng.user / 'levels'
        else:
            self.path = bng.home / 'levels'

        self.path = self.path / self.level
        if not self.path.exists():
            log.warn('Level for scenario does not exist: %s', self.path)
        self.path = self.path / 'scenarios'
        if not self.path.exists():
            self.path.mkdir(parents=True)

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

            pos_str = '{} {} {}'.format(*obj.position)
            rot_mat = compute_rotation_matrix(obj.rotation)
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

    def _get_vehicles_list(self):
        """
        Gets the vehicles contained in this scenario encoded as a dict and
        put into one list, including their position and rotation as a matrix
        ready to be placed in the simulator.

        Returns:
            All vehicles as a dict including position and rotation.
        """
        vehicles = list()
        for vehicle, data in self.vehicles.items():
            pos, rot = data
            vehicle_dict = dict(vid=vehicle.vid)
            vehicle_dict.update(vehicle.options)
            vehicle_dict['position'] = ' '.join([str(p) for p in pos])
            vehicle_dict['rotationMatrix'] = compute_rotation_matrix(rot)
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
        if not self.path:
            return None

        info_path = self.path / '{}.json'.format(self.name)
        return str(info_path)

    def get_prefab_path(self):
        """
        Returns: The path for the prefab file.
        """
        if not self.path:
            return None

        prefab_path = self.path / '{}.prefab'.format(self.name)
        return str(prefab_path)

    def add_object(self, obj):
        """
        Adds an extra object to be placed in the prefab. Objects are expected
        to be :class:`.ScenarioObject` instances with additional, type-
        specific properties in that class's opts dictionary.
        """
        self.objects.append(obj)

    def add_vehicle(self, vehicle, pos=(0, 0, 0), rot=(0, 0, 0), cling=True):
        """
        Adds a vehicle to this scenario at the given position with the given
        orientation. This method has to be called before a scenario is started.

        Args:
            pos (tuple): (x,y,z) tuple specifying the position of the vehicle.
            rot (tuple): (x,y,z) tuple expressing the rotation of the vehicle
                         in Euler angles around each axis.
        """
        self.vehicles[vehicle] = (pos, rot)

        if self.bng:
            self.bng.spawn_vehicle(vehicle, pos, rot, cling=cling)
            self.transient_vehicles.add(vehicle)

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

            del self.vehicles[vehicle]

    def get_vehicle(self, needle):
        """
        Retrieves the vehicle with the given ID from this scenario.

        Args:
            needle (str): The ID of the vehicle to find.

        Returns:
            The :class:`.Vehicle` with the given ID. None if it wasn't found.
        """
        for vehicle in self.vehicles.keys():
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

    def connect(self, bng):
        """
        Connects this scenario to the simulator, hooking up any cameras to
        their counterpart in the simulator.
        """
        self.bng = bng

        for mesh in self.proc_meshes:
            mesh.place(self.bng)

        for name, cam in self.cameras.items():
            cam.connect(self.bng, None)

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
        self._write_info_file()

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
        self._find_path(bng)
        info = self.get_info_path()

    def delete(self, bng):
        """
        Deletes files created by this scenario from the given
        :class:`.BeamNGpy`'s home/user path.
        """
        self._find_path(bng)
        os.remove(self.get_info_path())
        os.remove(self.get_prefab_path())

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
                vehicle.disconnect()
                self.bng.despawn_vehicle(vehicle)
                del self.vehicles[vehicle]

    def close(self):
        """
        Closes open connections and allocations of the scenario.
        """
        if not self.bng:
            raise BNGError('Scenario needs to be loaded into a BeamNGpy '
                           'instance to be stopped.')

        for vehicle in self.vehicles.keys():
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
