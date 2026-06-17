from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any, Dict, List

from beamngpy.api.vehicle import AccApi, AIApi, CouplersApi, LoggingApi, RootApi
from beamngpy.connection import Connection, Response
from beamngpy.logging import LOGGER_ID, BNGError, create_warning
from beamngpy.sensors import State
from beamngpy.types import Color, Float3, Quat, StrDict
from beamngpy.utils.prefab import get_uuid, lua_serialize
from beamngpy.utils.validation import validate_object_name
from beamngpy.vehicle.sensors import Sensors

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy

_beamng_dummy_instance = None

class Vehicle:
    """
    The vehicle class represents a vehicle of the simulation that can be
    interacted with from BeamNGpy. This class offers methods to both control
    the vehicle's state as well as retrieve information about it through
    sensors the user can attach to the vehicle.

    Creates a vehicle with the given vehicle ID. The ID must be unique
    within the scenario.

    Args:
        vid: The vehicle's ID.
        model: Model of the vehicle.
        port: The TCP port on which the vehicle should connect. If None, a
              new port is requested from the simulator.
        license: The license plate's text.
        color: The primary vehicle color.
        color2: The secondary vehicle color.
        color3: The tertiary vehicle color.
        extensions: A list of vehicle Lua extensions to load for the vehicle.
        part_config: The path to the vehicle part configuration (a ``.pc`` file) or a dictionary of the part
                     configuration (contents of a ``.pc`` file or value returned by
                     :meth:`~beamngpy.Vehicle.get_part_config`).
        options: Other possible vehicle options.

    Attributes
    ----------
        sensors: Sensors
            The sensors attached to the vehicle.
        ai: AIApi
            The API module to control the AI behavior of the vehicle.
            See :class:`.AIApi` for details.
        logging: LoggingApi
            Vehicle Signal Logger (``tech/vslSignalLogger``). See :class:`.LoggingApi`.
        start_signal_logging / stop_signal_logging
            Aliases of ``logging.start_logging`` / ``logging.stop_logging`` (same as
            ``start`` / ``stop`` on :class:`.LoggingApi`).
    """

    @staticmethod
    def from_dict(d: StrDict) -> Vehicle:
        if "name" in d:
            vid = d["name"]
            del d["name"]
        else:
            vid = d["id"]

        model = None
        if "model" in d:
            model = d["model"]
            del d["model"]

        port = None
        if "port" in d:
            port = int(d["port"])
            del d["port"]

        options = {}
        if "options" in d:
            options = d["options"]
            del d["options"]

        if not model:
            raise BNGError("The model of the vehicle is not specified!")
        vehicle = Vehicle(vid, model, port=port, **options)
        return vehicle

    @property
    def _uuid(self):
        return get_uuid(f"Vehicle_{self.vid}")

    def __init__(
        self,
        vid: str,
        model: str,
        port: int | None = None,
        license: str | None = None,
        color: Color | None = None,
        color2: Color | None = None,
        color3: Color | None = None,
        extensions: List[str] | None = None,
        part_config: str | StrDict | None = None,
        **options: Any,
    ):
        self.logger = getLogger(f"{LOGGER_ID}.Vehicle")
        self.logger.setLevel(DEBUG)

        self.vid = vid.replace(" ", "_")
        validate_object_name(self.vid)
        self.model = model

        self.port = port
        self.connection = None

        self.sensors = Sensors(self)

        options["model"] = model
        options["licenseText"] = (
            license or options.get("licence") or options.get("licenseText")
        )
        options["color"] = color or options.get("colour")
        options["color2"] = color2 or options.get("colour2")
        options["color3"] = color3 or options.get("colour3")
        part_config = part_config or options.get("partConfig")
        if isinstance(part_config, dict):
            part_config = lua_serialize(part_config)
        options["partConfig"] = part_config
        for key in ("licenseText", "partConfig"):
            if options[key] is None:
                del options[key]
        self.options = options

        self.extensions = extensions
        self.sensors.attach("state", State())

        self._init_api()
        self._init_beamng_api()

    def _init_api(self):
        self.ai = AIApi(self)
        self.ai_set_mode = self.ai.set_mode
        self.ai_set_speed = self.ai.set_speed
        self.ai_set_target = self.ai.set_target
        self.ai_set_waypoint = self.ai.set_waypoint
        self.ai_set_avoid_cars = self.ai.set_avoid_cars
        self.ai_drive_in_lane = self.ai.drive_in_lane
        self.ai_set_line = self.ai.set_line
        self.ai_set_script = self.ai.set_script
        self.ai_set_aggression = self.ai.set_aggression

        self._root = RootApi(
            self
        )  # this API is meant to be at the global level, so it is not meant to be public

        self.logging = LoggingApi(self)
        self.set_in_game_logging_options_from_json = self.logging.set_options_from_json
        self.write_in_game_logging_options_to_json = self.logging.write_options_to_json
        self.start_in_game_logging = self.logging.start
        self.stop_in_game_logging = self.logging.stop
        self.start_signal_logging = self.logging.start_logging
        self.stop_signal_logging = self.logging.stop_logging

        self.attach_sensor = self.sensors.attach
        self.detach_sensor = self.sensors.detach
        self.poll_sensors = self.sensors.poll

        self.acc = AccApi(self)  # this API is for ACC handling
        self.acc_load = self.acc.start
        self.acc_unload = self.acc.stop
        self.acc_change_speed = self.acc.change_speed

        self.couplers = CouplersApi(self)

    def _init_beamng_api(self, beamng: BeamNGpy | None = None):
        from beamngpy.api.beamng import GEVehiclesApi

        # create dummy BeamNGpy object for API hints to work properly (it will be replaced during `connect`)
        if beamng is None:
            from beamngpy.beamng import BeamNGpy
            global _beamng_dummy_instance
            if _beamng_dummy_instance is None:
                _beamng_dummy_instance = BeamNGpy("", -1)
            beamng = _beamng_dummy_instance

        self._ge_api = GEVehiclesApi(beamng, self)
        self.focus = self.switch  # alias

    def __hash__(self) -> int:
        return hash(self.vid)

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, type(self)):
            return self.vid == other.vid
        return False

    def __str__(self) -> str:
        return f"V:{self.vid}"

    def is_connected(self) -> bool:
        """
        Whether the vehicle is connected to the simulator and can be controlled from Python.
        """
        return bool(self.connection and self.connection.skt)

    @property
    def state(self) -> Dict[str, Float3 | Quat]:
        """
        This property contains the vehicle's current state in the running scenario. It is empty if no scenario is running or the state has not
        been retrieved yet. Otherwise, it contains the following key entries:

         * ``pos``: The vehicle's position as an ``(x, y, z)`` triplet
         * ``dir``: The vehicle's direction vector as an ``(x, y, z)`` triplet
         * ``up``: The vehicle's up vector as an ``(x, y, z)`` triplet
         * ``vel``: The vehicle's velocity along each axis in metres per second as an ``(x, y, z)`` triplet
         * ``rotation``: The vehicle's rotation as an ``(x, y, z, w)`` quaternion

        Note that the ``state`` variable represents a *snapshot* of the last state. It has to be updated with a call to :meth:`.Sensors.poll`
        or to :meth:`.Scenario.update`.
        """
        return self.sensors["state"]

    @state.setter
    def state(self, value: StrDict) -> None:
        self.sensors["state"].replace(value)

    @state.deleter
    def state(self) -> None:
        self.sensors["state"].clear()

    def _send(self, data: StrDict) -> Response:
        if not self.connection:
            raise BNGError("Not connected to the vehicle!")
        return self.connection.send(data)

    def connect(self, bng: BeamNGpy, tries: int = 5) -> None:
        """
        Opens socket communication with the corresponding vehicle.

        Args:
            bng: An instance of the simulator.
            tries: The number of connection attempts.

        Raises:
            BNGError: If the connection could not be established.
        """
        if not bng.connection:
            raise BNGError("The simulator is not connected to BeamNGpy!")
        if self.connection is None:
            self.connection = Connection(bng.host, self.port)

            # If we do not have a port (ie because it is the first time we wish to send to the given vehicle), then fetch a new port from the simulator.
            if self.connection.port is None:
                resp = bng.vehicles.start_connection(self, self.extensions)
                vid = resp["vid"]
                assert vid == self.vid
                self.connection.port = int(resp["result"])
                self.logger.debug(
                    f"Created new vehicle connection on port {self.connection.port}"
                )
                self.logger.info(f"Vehicle {vid} connected to simulation.")

        # Now attempt to connect to the given vehicle.
        if not self.connection.connect_to_vehicle(self, tries=tries):
            raise BNGError(f"Error connecting to vehicle {self.vid}.")

        # Connect the vehicle sensors.
        for _, sensor in self.sensors.items():
            sensor.connect(bng, self)
        self.bng = bng
        self._init_beamng_api(bng)

    def disconnect(self) -> None:
        """
        Closes socket communication with the corresponding vehicle.
        """
        for name, sensor in self.sensors.items():
            if name != "state":
                sensor.disconnect(self.bng, self)

        if self.connection is not None:
            self.connection.disconnect()
            self.connection = None

    def close(self) -> None:
        """
        Closes this vehicle's and its sensors' connection and detaches all
        sensors.
        """
        self.disconnect()

        for name, sensor in self.sensors.items():
            sensor.detach(self, name)

        self.sensors = Sensors(self)
        self.logger.info(f"Disconnected from vehicle {self.vid} and detached sensors.")

    def set_shift_mode(self, mode: str) -> None:
        """
        Sets the shifting mode of the vehicle. This changes whether or not and
        how the vehicle shifts gears depending on the RPM. Available modes are:

         * ``realistic_manual``
            Gears have to be shifted manually by the user, including engaging the clutch.
         * ``realistic_manual_auto_clutch``
            Gears have to be shifted manually by the user, without having to use the clutch.
         * ``arcade``
            Gears shift up and down automatically. If the brake is held, the vehicle automatically
            shifts into reverse and accelerates backward until brake is released or throttle is engaged.
         * ``realistic_automatic``
            Gears shift up automatically, but reverse and parking need to be shifted to manually.

        Args:
            mode: The mode to set. Must be a string from the options listed above.

        Raises:
            BNGValueError: If an invalid mode is given.
        """
        return self._root.set_shift_mode(mode)

    def control(
        self,
        steering: float | None = None,
        throttle: float | None = None,
        brake: float | None = None,
        parkingbrake: float | None = None,
        clutch: float | None = None,
        gear: int | None = None,
        is_adas: bool = False,
    ) -> None:
        """
        Sends a control message to the vehicle, setting vehicle inputs
        accordingly.

        Args:
            steering: Rotation of the steering wheel, from -1.0 to 1.0.
            throttle: Intensity of the throttle, from 0.0 to 1.0.
            brake: Intensity of the brake, from 0.0 to 1.0.
            parkingbrake: Intensity of the parkingbrake, from 0.0 to 1.0.
            clutch: Clutch level, from 0.0 to 1.0.
            gear: Gear to shift to, -1 eq backwards, 0 eq neutral, 1 to X eq nth gear
            is_adas: Whether the input source is an ADAS system.
        """
        return self._root.control(steering, throttle, brake, parkingbrake, clutch, gear, is_adas)

    def cycle_esc_mode(self) -> None:
        """
        Cycles the ESC mode if the vehicle has ESC.
        """
        return self._root.cycle_esc_mode()

    def set_esc_mode(self, mode: str) -> None:
        """
        Sets the ESC mode if the vehicle has ESC. This function won't do
        anything if the vehicle uses a "Regular ESC" mode.

        Args:
            mode: The key of the ESC mode to set. The key may vary from the mode's name in the UI.
        """
        return self._root.set_esc_mode(mode)

    def get_esc_mode(self) -> str:
        """
        Gets the current ESC mode's key if the vehicle has ESC. The key may
        vary from the mode's name in the UI. This function will return "none"
        if the vehicle uses a "Regular ESC" mode.
        """
        return self._root.get_esc_mode()

    def set_color(self, rgba: Color = (1.0, 1.0, 1.0, 1.0)) -> None:
        """
        Sets the color of this vehicle. Colour can be adjusted on the RGB
        spectrum and the "shininess" of the paint.

        Args:
            rgba: The new colour given as a tuple of RGBA floats, where
                  the alpha channel encodes the shininess of the paint.
                  Also can be given in any format specified in :class:`~beamngpy.types.Color`.
        """
        return self._root.set_color(rgba)

    def set_velocity(self, velocity: float, dt: float = 1.0) -> None:
        """
        Sets the velocity of this vehicle. The velocity is not achieved instantly,
        it is acquired gradually over the time interval set by the `dt` argument.

        As the method of setting velocity uses physical forces, at high velocities
        it is important to set ``dt`` to an appropriately high value. The default
        ``dt`` value of 1.0 is suitable for velocities up to 30 m/s.

        Args:
            velocity: The target velocity in m/s.
            dt: The time interval over which the vehicle reaches the target velocity.
                Defaults to 1.0.
        """
        return self._root.set_velocity(velocity, dt)

    def set_lights(
        self,
        left_signal: bool | None = None,
        right_signal: bool | None = None,
        hazard_signal: bool | None = None,
        headlights: int | None = None,
        fog_lights: int | None = None,
        lightbar: int | None = None,
    ) -> None:
        """
        Sets the vehicle's lights to given intensity values. The lighting
        system features lights that are simply binary on/off, but also ones
        where the intensity can be varied. Binary lights include:

            * ``left_signal``
            * ``right_signal``
            * ``hazard_signal``

        Non-binary lights vary between 0 for off, 1 for on, 2 for higher
        intensity. For example, headlights can be turned on with 1 and set to
        be more intense with 2. Non-binary lights include:

            * ``headlights``
            * ``fog_lights``
            * ``lightbar``

        Args:
            left_signal: On/off state of the left signal
            right_signal: On/off state of the right signal
            hazard_signal: On/off state of the hazard lights
            headlights: Value from 0 to 2 indicating headlight intensity
            fog_lights: Value from 0 to 2 indicating fog light intensity
            lightbar: Value from 0 to 2 indicating lightbar intensity

        Note:
            Not every vehicle has every type of light. For example, the
            `lightbar` refers to the kind of lights typically found on top of
            police cars. Setting values for non-existent lights will not cause
            an error, but also achieve no effect.

            Note also that lights are not independent. For example, turning on
            the hazard lights will make both signal indicators blink, meaning
            they will be turned on as well. Opposing indicators also turn each
            other off, i.e. turning on the left signal turns off the right one,
            and turning on the left signal during

        Raises:
            BNGValueError: If an invalid light value is given.

        Returns:
            Nothing. To query light states, attach an
            :class:`.sensors.Electrics` sensor and poll it.
        """
        return self._root.set_lights(
            left_signal, right_signal, hazard_signal, headlights, fog_lights, lightbar
        )

    def queue_lua_command(self, chunk: str, response: bool = False) -> StrDict:
        """
        Executes a chunk of Lua code in the vehicle engine VM.

        Args:
            chunk: A chunk of Lua code as a string.
            response: If True, then the response is sent back to BeamNGpy.
        """
        return self._root.queue_lua_command(chunk, response)

    def recover(self) -> None:
        """
        Recovers the vehicle to a drivable position and state and repairs its damage.
        """
        return self._root.recover()

    def get_bbox(self) -> Dict[str, Float3]:
        """
        Returns a vehicle's current bounding box as a dictionary containing
        eight points. The bounding box
        corresponds to the vehicle's location/rotation in world space, i.e. if
        the vehicle moves/turns, the bounding box moves accordingly. Note that
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

            * ``front_bottom_left``
                Bottom left point of the front rectangle as an ``(x, y, z)`` triplet
            * ``front_bottom_right``
                Bottom right point of the front rectangle as an ``(x, y, z)`` triplet
            * ``front_top_left``
                Top left point of the front rectangle as an ``(x, y, z)`` triplet
            * ``front_top_right``
                Top right point of the front rectangle as an ``(x, y, z)`` triplet
            * ``rear_bottom_left``
                Bottom left point of the rear rectangle as an ``(x, y, z)`` triplet
            * ``rear_bottom_right``
                Bottom right point of the rear rectangle as an ``(x, y, z)`` triplet
            * ``rear_top_left``
                Top left point of the rear rectangle as an ``(x, y, z)`` triplet
            * ``rear_top_right``
                Top right point of the rear rectangle as an ``(x, y, z)`` triplet
        """
        return self._ge_api.get_bbox()

    def get_center_of_gravity(self, without_wheels=False) -> Float3:
        """
        Returns the vehicle's current center of gravity in world coordinates.

        Args:
            without_wheels: If True, the center of gravity is calculated without the wheels.
                            Defaults to False.

        Returns:
            The center of gravity as a ``(x, y, z)`` triplet.
        """
        return self._root.get_center_of_gravity(without_wheels)

    def annotate_parts(self) -> None:
        """
        Triggers the process to have individual parts of a vehicle have unique
        annotation colors.
        """
        return self._ge_api.annotate_parts()

    def revert_annotations(self) -> None:
        """
        Reverts the given vehicle's annotations back to the object-based mode,
        removing the per-part annotations.
        """
        return self._ge_api.revert_annotations()

    def switch(self) -> None:
        """
        Switches the simulator to this vehicle. This means that the simulator's main camera,
        inputs by the user, and so on will all focus on this vehicle from now on.
        """
        return self._ge_api.switch()

    def teleport(
        self, pos: Float3,
        rot_quat: Quat | None = None,
        reset: bool = True,
        safe_spawn: bool = False, 
    ) -> bool:
        """
        Teleports the vehicle to the given position with the given
        rotation.

        Args:
            pos: The target position as an (x,y,z) tuple containing world-space coordinates.
            rot_quat: Optional tuple (x, y, z, w) specifying vehicle rotation as quaternion.
            reset: Specifies if the vehicle will be reset to its initial state during teleport (including its velocity).
            safe_spawn: If True, the vehicle will be spawned in the nearest safe position on the ground, avoiding spawning the vehicle below ground or in the air, and collisions with other vehicles or objects. If there is no safe position nearby, the vehicle will be spawned at the given position. This options may modify the spawn position (including x and y coordinates) as well as the rotation of the vehicle. Defaults to False.
        """
        return self._ge_api.teleport(pos, rot_quat, reset, safe_spawn=safe_spawn)

    def get_part_options(self) -> StrDict:
        """
        Deprecated in favor of :func:`get_part_config()`.

        Retrieves a mapping of part slots for the given vehicle and their
        possible parts.

        Returns:
            A mapping of part configuration options for the given.
        """
        create_warning("Function `get_part_options` is deprecated, use the `suitablePartNames` field of the `get_part_config` tree.\n" +
                       "Calling `get_part_config` instead.")
        return self._ge_api.get_part_config()

    def get_part_config(self) -> StrDict:
        """
        Retrieves the current part configuration of the given vehicle.

        The configuration is defined as a part tree with the following fields:

            * ``partPath``
                The unique path to the part from the root (``/``), containing all parent slots.
            * ``path``
                The unique path to the slot from the root (``/``), containing all parent slots and not containing the current part.
            * ``id``
                The ID of the slot.
            * ``chosenPartName``
                Name of the part if it is equipped.
            * ``suitablePartNames``
                List of part names that can be used for this slot.
            * ``unsuitablePartNames``
                List of part names that **cannot** be used for this slot.
            * ``children``
                A dictionary of the ancestor slots of the part equipped.
            * ``decisionMethod``
                Describes the reason why the part is used for the slot. Can be ``user-empty``, ``user``,
                ``default-empty`` or ``default``.

        See also: :meth:`~beamngpy.api.beamng.VehiclesApi.get_part_config_for_config`

        Returns:
            The current vehicle configuration tree as a dictionary.
        """
        return self._ge_api.get_part_config()

    def set_part_config(self, cfg: StrDict) -> None:
        """
        Sets the current part configuration of the given vehicle. The configuration is
        a part tree. You can get a valid part configuration tree by calling the
        :func:`get_part_config()` function.

        Args:
            cfg: The new vehicle configuration tree as a dictionary.

        Notes:
            Changing parts causes the vehicle to respawn, which repairs it as
            a side-effect.
        """
        return self._ge_api.set_part_config(cfg)

    def set_license_plate(self, text: str) -> None:
        """
        Sets the text of the vehicle's license plate.

        Args:
            text: The vehicle plate text to be set.
        """
        return self._ge_api.set_license_plate(text)

    def deflate_tire(self, wheel_id: int) -> None:
        """
        Deflates the tire of this vehicle with the given wheel ID.

        Args:
            wheel_id: The given wheel ID.
        """
        return self._root.deflate_tire(wheel_id)

    def get_mass_properties(self, without_wheels: bool = False) -> StrDict:
        """
        Returns a vehicle's current mass properties including:

        * ``mass``: total vehicle mass in kg (float)
        * ``center_of_gravity``: center of gravity (i.e. center of mass) in world coordinates as an (x, y, z) tuple in m
        * ``inertia``: inertia tensor with respect to the world's coordinate axes about the center of gravity as dict
          (x: float, y: float, z: float, xy: float, xz: float, yz: float) in kg*m^2

        Args:
            without_wheels: If True, the all properties are calculated without the wheels.
                            Defaults to False.

        Returns:
            The vehicle's mass properties as a dictionary.
        """
        return self._root.get_mass_properties(without_wheels=without_wheels)

    def get_ref_nodes(self) -> StrDict:
        """
        Returns the vehicle's reference nodes as a dictionary. The dictionary contains:

        * ``ref``: the node name of the reference node (str)
        * ``back``: the node name of the back reference node (str)
        * ``left``: the node name of the left reference node (str)
        * ``up``: the node name of the up reference node (str)
        * ``leftCorner``: the node name of the front left corner node (str)
        * ``rightCorner``: the node name of the front right corner node (str)

        Returns:
            The vehicle's reference nodes as a dictionary.
        """
        return self._root.get_ref_nodes()

    def get_node_info(self, nodes: List[str | int] | None = None) -> List[StrDict]:
        """
        Returns current mass and position of the given nodes.

        The returned list contains information for each requested node. The node information is a dictionary containing:

        * ``name``: the name of the node (str) or None if the node has no name
        * ``cid``: the numeric ID of the node (int)
        * ``mass``: the current mass of the node in kg (float)
        * ``pos``: the current position of the node in world coordinates as an (x, y, z) tuple in m

        Args:
            nodes: A list of node names to query information for. If None (default), all nodes are provided, otherwise
                   the returned list is in the same order as the given nodes. The list can contain strings (node names)
                   or integers (node CIDs).

        Returns:
            A list with node information for each requested node.
        """
        return self._root.get_node_info(nodes=nodes)
