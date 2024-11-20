from __future__ import annotations

from typing import Dict, Iterable, List

from beamngpy.misc.colors import coerce_color, rgba_to_str
from beamngpy.types import Float3, Quat, StrDict
from beamngpy.vehicle import Vehicle

from .base import Api


class VehiclesApi(Api):
    """
    An API for vehicle manipulation in the simulator.

    Args:
        beamng: An instance of the simulator.
    """

    def start_connection(
        self, vehicle: Vehicle, extensions: List[str] | None
    ) -> StrDict:
        connection_msg: StrDict = {"type": "StartVehicleConnection"}
        connection_msg["vid"] = vehicle.vid
        if extensions is not None:
            connection_msg["exts"] = extensions
        return self._send(connection_msg).recv("StartVehicleConnection")

    def spawn(
        self,
        vehicle: Vehicle,
        pos: Float3,
        rot_quat: Quat = (0, 0, 0, 1),
        cling: bool = True,
        connect: bool = True,
    ) -> bool:
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
        data: StrDict = dict(type="SpawnVehicle", cling=cling)
        data.update(vehicle.options)
        data["name"] = vehicle.vid
        data["model"] = vehicle.options["model"]
        data["pos"] = pos
        data["rot"] = rot_quat
        for color in ("color", "color2", "color3"):
            if data[color] is not None:
                data[color] = rgba_to_str(coerce_color(data[color]))

        resp = self._send(data).recv("VehicleSpawned")
        if resp["success"]:
            if connect:
                vehicle.connect(self._beamng)
        return resp["success"]

    def despawn(self, vehicle: Vehicle) -> None:
        """
        Despawns the given :class:`.Vehicle` from the simulation.

        Args:
            vehicle: The vehicle to despawn.
        """
        vehicle.disconnect()
        data = dict(type="DespawnVehicle")
        data["vid"] = vehicle.vid
        self._send(data).ack("VehicleDespawned")

    def replace(
        self,
        new_vehicle: Vehicle,
        old_vehicle: Vehicle | str | None = None,
        connect: bool = True,
    ) -> None:
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

        data: StrDict = dict(type="SpawnVehicle")
        data.update(new_vehicle.options)
        data["name"] = new_vehicle.vid
        data["model"] = new_vehicle.options["model"]
        data["replace"] = True
        data["replace_vid"] = (
            old_vehicle.vid if isinstance(old_vehicle, Vehicle) else old_vehicle
        )
        for color in ("color", "color2", "color3"):
            if data[color] is not None:
                data[color] = rgba_to_str(coerce_color(data[color]))

        resp = self._send(data).recv("VehicleSpawned")
        if resp["success"] and connect:
            new_vehicle.connect(self._beamng)
        return resp["success"]

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
        data = dict(type="GetAvailableVehicles")
        print("data:", data)
        return self._send(data).recv("AvailableVehicles")

    def await_spawn(self, vid: str | Vehicle) -> None:
        """
        Waits for the vehicle with the given name to spawn and returns once it
        has.

        Args:
            vid: The name of the vehicle to wait for.
        """
        data: StrDict = dict(type="WaitForSpawn")
        data["name"] = vid
        resp = self._send(data).recv("VehicleSpawned")
        assert resp["name"] == vid

    def switch(self, vehicle: str | Vehicle) -> None:
        """
        Switches to the given :class:`.Vehicle`. This means that the
        simulator's main camera, inputs by the user, and so on will all focus
        on that vehicle from now on.

        Args:
            vehicle: The target vehicle.
        """
        data = dict(type="SwitchVehicle")
        data["vid"] = vehicle.vid if isinstance(vehicle, Vehicle) else vehicle
        self._send(data).ack("VehicleSwitched")

    def teleport(
        self,
        vehicle: str | Vehicle,
        pos: Float3,
        rot_quat: Quat | None = None,
        reset: bool = True,
    ) -> bool:
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

        self._logger.info(f"Teleporting vehicle <{vehicle_id}>.")
        data: StrDict = dict(type="Teleport")
        data["vehicle"] = vehicle_id
        data["pos"] = pos
        data["reset"] = reset
        if rot_quat:
            data["rot"] = rot_quat
        resp = self._send(data).recv("Teleported")
        return resp["success"]

    def get_part_annotations(self, vehicle: Vehicle):
        data = dict(type="GetPartAnnotations")
        data["vid"] = vehicle.vid
        resp = self._send(data).recv("PartAnnotations")
        return resp["colors"]

    def get_part_annotation(self, part):
        data = dict(type="GetPartAnnotation")
        data["part"] = part
        resp = self._send(data).recv("PartAnnotation")
        if "color" in resp:
            return resp["color"]
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
        data: StrDict = dict(type="UpdateScenario")
        data["vehicles"] = list(vehicles)
        resp = self._send(data).recv("ScenarioUpdate")
        return resp["vehicles"]

    def get_current_info(self, include_config: bool = True) -> Dict[str, StrDict]:
        """
        Queries the currently active vehicles in the simulator.

        Args:
            include_config: Whether to include info about possible configurations of the vehicles.

        Returns:
            A mapping of vehicle IDs to dictionaries of data needed to represent
            a :class:`.Vehicle`.
        """
        info = self._message("GetCurrentVehicles", include_config=include_config)
        if not info:
            return {}
        for vid, vehicle in info.items():
            vehicle["id"] = int(vehicle["id"])
        return info

    def get_current(self, include_config: bool = True) -> Dict[str, Vehicle]:
        """
        Queries the currently active vehicles in the simulator.

        Args:
            include_config: Whether to include info about possible configurations of the vehicles.

        Returns:
            A mapping of vehicle IDs to instances of the :class:`.Vehicle`
            class for each active vehicle. These vehicles are not connected to
            by this function.
        """
        vehicles = self.get_current_info(include_config=include_config)
        vehicles = {n: Vehicle.from_dict(v) for n, v in vehicles.items()}
        return vehicles

    def get_player_vehicle_id(self) -> StrDict:
        """
        Queries the currently player vehicles in the simulator.

        Returns:
            A dictionary of the active vehicle in simulator from lua.
            {'type': 'getPlayerVehicleID', 'id': 10455.0, 'vid': 'vehicleA'}
            then in python, the return will be only an int value of the 'id' and vehicle's name
            {'id': 10455, 'vid': 'vehicleA'}
            data = bng.vehicles.get_player_vehicle_id()
            for testing you can use the following:
            id_value = data['id']
            vid_value = data['vid']
        """
        data = dict(type="GetPlayerVehicleID")
        resp = self._send(data).recv("getPlayerVehicleID")
        resp = {"id": int(resp["id"]), "vid": resp["vid"]}
        return resp

    def set_license_plate(self, vehicle: str | Vehicle, text: str) -> None:
        """
        Sets the text of a vehicle's license plate.

        Args:
            vehicle: The id/name of the vehicle to teleport or the vehicle's object.
            text: The vehicle plate text to be set.
        """
        data: StrDict = dict(type="SetLicensePlate")
        data["vid"] = vehicle.vid if isinstance(vehicle, Vehicle) else vehicle
        data["text"] = text
        self._send(data).ack("SetLicensePlate")
