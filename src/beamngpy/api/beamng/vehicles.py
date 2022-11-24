from __future__ import annotations

from typing import TYPE_CHECKING, Dict, Iterable, List

from beamngpy.logging import create_warning
from beamngpy.types import Float3, Quat, StrDict
from beamngpy.vehicle import Vehicle

from .base import Api

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy


class VehiclesApi(Api):
    def start_connection(self, vehicle: Vehicle, extensions: List[str] | None) -> StrDict:
        connection_msg: StrDict = {'type': 'StartVehicleConnection'}
        connection_msg['vid'] = vehicle.vid
        if extensions is not None:
            connection_msg['exts'] = extensions
        return self._send(connection_msg).recv('StartVehicleConnection')

    def spawn(self, vehicle: Vehicle, pos: Float3, rot_quat: Quat = (0, 0, 0, 1), cling: bool = True, connect: bool = True) -> bool:
        """
        Spawns the given :class:`.Vehicle` instance in the simulator. This
        method is meant for spawning vehicles *during the simulation*. Vehicles
        that are known to be required before running the simulation should be
        added during scenario creation instead. Cannot spawn two vehicles with
        the same id/name.

        Args:
            vehicle: The vehicle to be spawned.
            pos: Where to spawn the vehicle as a (x, y, z) triplet.
            rot_quat: Vehicle rotation in form of a quaternion
            cling: If set, the z-coordinate of the vehicle's position
                   will be set to the ground level at the given
                   position to avoid spawning the vehicle below ground
                   or in the air.
            connect: Whether to connect the newly spawned vehicle to BeamNGpy.

        Returns:
            bool indicating whether the spawn was successful or not
        """
        data: StrDict = dict(type='SpawnVehicle', cling=cling)
        data['name'] = vehicle.vid
        data['model'] = vehicle.options['model']
        data['pos'] = pos
        data['rot'] = rot_quat
        data.update(vehicle.options)
        resp = self._send(data).recv('VehicleSpawned')
        if resp['success'] and connect:
            vehicle.connect(self._beamng)
        return resp['success']

    def despawn(self, vehicle: Vehicle) -> None:
        """
        Despawns the given :class:`.Vehicle` from the simulation.

        Args:
            vehicle: The vehicle to despawn.
        """
        vehicle.disconnect()
        data = dict(type='DespawnVehicle')
        data['vid'] = vehicle.vid
        self._send(data).ack('VehicleDespawned')

    def replace(self, new_vehicle: Vehicle, old_vehicle: Vehicle | str | None = None, connect: bool = True) -> None:
        """
        Replaces ``old_vehicle`` with ``new_vehicle`` in the scenario. The ``new_vehicle`` keeps
        the position and rotation of ``old_vehicle``. If ``old_vehicle`` is not provided, then
        the current player vehicle is replaced by ``new_vehicle``.

        Args:
            new_vehicle: The vehicle to
            old_vehicle: The vehicle to be replaced, or its ID, or None if the currently focused
                         vehicle should be replaced.
            connect: Whether to connect the replaced vehicle to BeamNGpy.
        """
        if isinstance(old_vehicle, Vehicle) and old_vehicle.is_connected():
            old_vehicle.disconnect()

        data: StrDict = dict(type='SpawnVehicle')
        data['name'] = new_vehicle.vid
        data['model'] = new_vehicle.options['model']
        data['replace'] = True
        data['replace_vid'] = old_vehicle.vid if isinstance(old_vehicle, Vehicle) else old_vehicle
        data.update(new_vehicle.options)

        resp = self._send(data).recv('VehicleSpawned')
        if resp['success'] and connect:
            new_vehicle.connect(self._beamng)
        return resp['success']

    def get_available(self) -> StrDict:
        """
        Retrieves a dictionary of vehicles known to the simulator that map
        to various properties of the vehicle and a list of pre-configured
        vehicle configurations.

        Returns:
            A mapping of model names to vehicle properties & configs.

        Raises:
            BNGError: If the game is not running to accept a request.
        """
        data = dict(type='GetAvailableVehicles')
        return self._send(data).recv('AvailableVehicles')

    def await_spawn(self, vid: str | Vehicle) -> None:
        """
        Waits for the vehicle with the given name to spawn and returns once it
        has.

        Args:
            vid: The name of the vehicle to wait for.
        """
        data: StrDict = dict(type='WaitForSpawn')
        data['name'] = vid
        resp = self._send(data).recv('VehicleSpawned')
        assert resp['name'] == vid

    def switch(self, vehicle: str | Vehicle) -> None:
        """
        Switches to the given :class:`.Vehicle`. This means that the
        simulator's main camera, inputs by the user, and so on will all focus
        on that vehicle from now on.

        Args:
            vehicle: The target vehicle.
        """
        data = dict(type='SwitchVehicle')
        data['vid'] = vehicle.vid if isinstance(vehicle, Vehicle) else vehicle
        self._send(data).ack('VehicleSwitched')

    def teleport(self, vehicle: str | Vehicle, pos: Float3, rot_quat: Quat | None = None, reset: bool = True) -> bool:
        """
        Teleports the given vehicle to the given position with the given
        rotation.

        Args:
            vehicle: The id/name of the vehicle to teleport or the vehicle's object.
            pos: The target position as an (x, y, z) tuple containing world-space coordinates.
            rot_quat: Optional tuple (x, y, z, w) specifying vehicle rotation as quaternion.
            reset: Specifies if the vehicle will be reset to its initial
                   state during teleport (including its velocity).
        """
        vehicle_id = vehicle.vid if isinstance(vehicle, Vehicle) else vehicle

        self._logger.info(f'Teleporting vehicle <{vehicle_id}>.')
        data: StrDict = dict(type='Teleport')
        data['vehicle'] = vehicle_id
        data['pos'] = pos
        data['reset'] = reset
        if rot_quat:
            data['rot'] = rot_quat
        resp = self._send(data).recv('Teleported')
        return resp['success']

    def get_part_annotations(self, vehicle: Vehicle):
        data = dict(type='GetPartAnnotations')
        data['vid'] = vehicle.vid
        resp = self._send(data).recv('PartAnnotations')
        return resp['colors']

    def get_part_annotation(self, part):
        data = dict(type='GetPartAnnotation')
        data['part'] = part
        resp = self._send(data).recv('PartAnnotation')
        if 'color' in resp:
            return resp['color']
        return None

    def get_states(self, vehicles: Iterable[str]) -> Dict[str, Dict[str, Float3]]:
        """
        Gets the states of the vehicles provided as the argument to this function.
        The returned state includes position, direction vectors and the velocities.

        Args:
            vehicles: A list of the vehicle IDs to query state from.

        Returns:
            A mapping of the vehicle IDs to their state stored as a dictionary
            with [``pos``, ``dir``, ``up``, ``vel``] keys.
        """
        data: StrDict = dict(type='UpdateScenario')
        data['vehicles'] = list(vehicles)
        resp = self._send(data).recv('ScenarioUpdate')
        return resp['vehicles']


class BoundVehiclesApi(Api):
    def __init__(self, beamng: BeamNGpy, vehicle: Vehicle):
        super().__init__(beamng)
        self.vehicle = vehicle

    def get_bbox(self) -> Dict[str, Float3]:
        """
        Returns a vehicle's current bounding box as a dictionary containing
        eight points. The bounding box
        corresponds to the vehicle's location/rotation in world space, i.e. if
        the vehicle moves/turns, the bounding box moves acoordingly. Note that
        the bounding box contains the min/max coordinates of the entire
        vehicle. This means that the vehicle losing a part like a mirror will
        cause the bounding box to "expand" while the vehicle moves as the
        mirror is left behind, but still counts as part of the box containing
        the vehicle.

        Returns:
            The vehicle's current bounding box as a dictionary of eight points.
            Points are named following the convention that the cuboid has a
            "near" rectangle towards the rear of the vehicle and "far"
            rectangle towards the front. The points are then named like this:

            * `front_bottom_left`: Bottom left point of the front rectangle as
                                   an (x, y ,z) triplet
            * `front_bottom_right`: Bottom right point of the front rectangle
                                    as an (x, y, z) triplet
            * `front_top_left`: Top left point of the front rectangle as an
                               (x, y, z) triplet
            * `front_top_right`: Top right point of the front rectangle as an
                                (x, y, z) triplet
            * `rear_bottom_left`: Bottom left point of the rear rectangle as an
                                 (x, y, z) triplet
            * `rear_bottom_right`: Bottom right point of the rear rectangle as
                                   an (x, y, z) triplet
            * `rear_top_left`: Top left point of the rear rectangle as an
                              (x, y, z) triplet
            * `rear_top_right`: Top right point of the rear rectangle as an
                               (x, y, z) triplet
        """
        data = dict(type='GetBBoxCorners')
        data['vid'] = self.vehicle.vid
        resp = self._send(data).recv('BBoxCorners')
        points = resp['points']
        bbox = {
            'front_bottom_left': points[3],
            'front_bottom_right': points[0],
            'front_top_left': points[2],
            'front_top_right': points[1],
            'rear_bottom_left': points[7],
            'rear_bottom_right': points[4],
            'rear_top_left': points[6],
            'rear_top_right': points[5],
        }
        return bbox

    def annotate_parts(self) -> None:
        """
        Triggers the process to have individual parts of a vehicle have unique
        annotation colors.
        """
        data = dict(type='AnnotateParts')
        data['vid'] = self.vehicle.vid
        self._send(data).ack('PartsAnnotated')

    def revert_annotations(self) -> None:
        """
        Reverts the given vehicle's annotations back to the object-based mode,
        removing the per-part annotations.
        """
        data = dict(type='RevertAnnotations')
        data['vid'] = self.vehicle.vid
        self._send(data).ack('AnnotationsReverted')

    def switch(self):
        """
        Switches the simulator to this vehicle. This means that the simulator's main camera,
        inputs by the user, and so on will all focus on this vehicle from now on.
        """
        return self._beamng.switch_vehicle(self.vehicle)

    def teleport(self, pos: Float3, rot_quat: Quat | None = None, reset: bool = True) -> bool:
        """
        Teleports the vehicle to the given position with the given
        rotation.

        Args:
            pos: The target position as an (x,y,z) tuple containing world-space coordinates.
            rot_quat: Optional tuple (x, y, z, w) specifying vehicle rotation as quaternion
            reset: Specifies if the vehicle will be reset to its initial state during teleport (including its velocity).

        Notes:
            The ``reset=False`` option is incompatible with setting rotation of
            the vehicle. With the current implementation, it is not possible to
            set the rotation of the vehicle and to keep its velocity during teleport.
        """
        return self._beamng.teleport_vehicle(self.vehicle.vid, pos, rot_quat, reset)

    def get_part_options(self) -> StrDict:
        """
        Retrieves a mapping of part slots for the given vehicle and their
        possible parts.

        Returns:
            A mapping of part configuration options for the given.
        """
        data = dict(type='GetPartOptions')
        data['vid'] = self.vehicle.vid
        resp = self._send(data).recv('PartOptions')
        return resp['options']

    def get_part_config(self) -> StrDict:
        """
        Retrieves the current part configuration of the given vehicle. The
        configuration contains both the current values of adjustable vehicle
        parameters and a mapping of part types to their currently-selected
        part.

        Returns:
            The current vehicle configuration as a dictionary.
        """
        data = dict(type='GetPartConfig')
        data['vid'] = self.vehicle.vid
        resp = self._send(data).recv('PartConfig')
        resp = resp['config']
        if 'parts' not in resp or not resp['parts']:
            resp['parts'] = dict()
        if 'vars' not in resp or not resp['vars']:
            resp['vars'] = dict()
        return resp

    def set_part_config(self, cfg: StrDict) -> None:
        """
        Sets the current part configuration of the given vehicle. The
        configuration is given as a dictionary containing both adjustable
        vehicle parameters and a mapping of part types to their selected parts.

        Args:
            cfg: The new vehicle configuration as a dictionary.

        Notes:
            Changing parts causes the vehicle to respawn, which repairs it as
            a side-effect.
        """
        data: StrDict = dict(type='SetPartConfig')
        data['vid'] = self.vehicle.vid
        data['config'] = cfg
        self._send(data)
        self._beamng.await_vehicle_spawn(self.vehicle.vid)
        self.vehicle.close()
        self.vehicle.connect(self._beamng)
