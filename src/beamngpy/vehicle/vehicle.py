from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any, Dict

from beamngpy.api.vehicle import AIApi, ControlApi, ControllerApi, LoggingApi
from beamngpy.connection import Connection, Response
from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.sensors import State
from beamngpy.types import Float3, Float4, StrDict
from beamngpy.vehicle.sensors import Sensors

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy

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

    Attributes
    ----------
        ai: AIApi
        logging: LoggingApi
    """

    @staticmethod
    def from_dict(d: StrDict) -> Vehicle:
        if 'name' in d:
            vid = d['name']
            del d['name']
        else:
            vid = d['id']

        model = None
        if 'model' in d:
            model = d['model']
            del d['model']

        port = None
        if 'port' in d:
            port = int(d['port'])
            del d['port']

        options = {}
        if 'options' in d:
            options = d['options']
            del d['options']

        if not model:
            raise BNGError('The model of the vehicle is not specified!')
        vehicle = Vehicle(vid, model, port=port, **options)
        return vehicle

    def __init__(self, vid: str, model: str, port: int | None = None, license: str | None = None,
                 color: Any = None, color2: Any = None, color3: Any = None,
                 extensions: Any = None, part_config: str | None = None, **options: Any):
        self.logger = getLogger(f'{LOGGER_ID}.Vehicle')
        self.logger.setLevel(DEBUG)

        self.vid = vid.replace(' ', '_')
        self.model = model

        self.port = port
        self.connection = None

        self.sensors = Sensors(self)

        options['model'] = model
        options['licenseText'] = license or options.get('licence')
        options['color'] = color or options.get('colour')
        options['color2'] = color2 or options.get('colour2')
        options['color3'] = color3 or options.get('colour3')
        options['partConfig'] = part_config or options.get('partConfig')
        self.options = options

        self.extensions = extensions

        self._veh_state_sensor_id = "state"
        state = State()
        self.sensors.attach(self._veh_state_sensor_id, state)

        self._init_api()
        self._init_beamng_api()

    def _init_api(self):
        self.ai = AIApi(self)
        self.ai_set_mode = self.ai.set_mode
        self.ai_set_speed = self.ai.set_speed
        self.ai_set_target = self.ai.set_target
        self.ai_set_waypoint = self.ai.set_waypoint
        self.ai_drive_in_lane = self.ai.drive_in_lane
        self.ai_set_line = self.ai.set_line
        self.ai_set_script = self.ai.set_script
        self.ai_set_aggression = self.ai.set_aggression

        control = ControlApi(self)  # this API is meant to be at the global level, so no self.control
        self._control = control

        self.logging = LoggingApi(self)
        self.set_in_game_logging_options_from_json = self.logging.set_in_game_logging_options_from_json
        self.write_in_game_logging_options_to_json = self.logging.write_in_game_logging_options_to_json
        self.start_in_game_logging = self.logging.start_in_game_logging
        self.stop_in_game_logging = self.logging.stop_in_game_logging

        self.controller = ControllerApi(self)

        self.attach_sensor = self.sensors.attach
        self.detach_sensor = self.sensors.detach
        self.poll_sensors = self.sensors.poll

    def _init_beamng_api(self, beamng: BeamNGpy | None = None):
        from beamngpy.api.beamng import BoundVehiclesApi

        # create dummy BeamNGpy object for API hints to work properly (it will be replaced during `connect`)
        if beamng is None:
            from beamngpy.beamng import BeamNGpy
            beamng = BeamNGpy('', -1)

        api = BoundVehiclesApi(beamng, self)
        self.annotate_parts = api.annotate_parts
        self.revert_annotations = api.revert_annotations
        self.get_bbox = api.get_bbox
        self.get_part_options = api.get_part_options
        self.get_part_config = api.get_part_config
        self.set_part_config = api.set_part_config
        self.teleport = api.teleport
        self.switch = api.switch
        self.focus = api.switch # alias

    def __hash__(self) -> int:
        return hash(self.vid)

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, type(self)):
            return self.vid == other.vid
        return False

    def __str__(self) -> str:
        return 'V:{}'.format(self.vid)

    def is_connected(self) -> bool:
        return bool(self.connection and self.connection.skt)

    @property
    def state(self) -> Dict[str, Float3]:
        """
        This property contains the vehicle's current state in the running scenario. It is empty if no scenario is running or the state has not
        been retrieved yet. Otherwise, it contains the following key entries:

         * ``pos``: The vehicle's position as an (x,y,z) triplet
         * ``dir``: The vehicle's direction vector as an (x,y,z) triplet
         * ``up``: The vehicle's up vector as an (x,y,z) triplet
         * ``vel``: The vehicle's velocity along each axis in metres per second as an (x,y,z) triplet

        Note that the `state` variable represents a *snapshot* of the last state. It has to be updated with a call to :meth:`.Vehicle.poll_sensors`
        or to :meth:`.Scenario.update`.
        """
        return self.sensors[self._veh_state_sensor_id]

    @state.setter
    def state(self, value: StrDict) -> None:
        self.sensors[self._veh_state_sensor_id].replace(value)

    @state.deleter
    def state(self) -> None:
        self.sensors[self._veh_state_sensor_id].clear()

    def _send(self, data: StrDict) -> Response:
        if not self.connection:
            raise BNGError('Not connected to the vehicle!')
        return self.connection.send(data)

    def connect(self, bng: BeamNGpy) -> None:
        """
        Opens socket communication with the corresponding vehicle.
        """
        if not bng.connection:
            raise BNGError('The simulator is not connected to BeamNGpy!')
        if self.connection is None:
            self.connection = Connection(bng.host, self.port)

            # If we do not have a port (ie because it is the first time we wish to send to the given vehicle), then fetch a new port from the simulator.
            if self.connection.port is None:
                resp = bng.vehicles.start_connection(self, self.extensions)
                vid = resp['vid']
                assert vid == self.vid
                self.connection.port = int(resp['result'])
                self.logger.debug(f'Created new vehicle connection on port {self.connection.port}')
                self.logger.info(f'Vehicle {vid} connected to simulation.')

        # Now attempt to connect to the given vehicle.
        self.connection.connect_to_vehicle(self)

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
            if name != self._veh_state_sensor_id:
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
        self.logger.info(f'Disconnected from vehicle {self.vid} and detached sensors.')

    def set_shift_mode(self, mode: str) -> None:
        """
        Sets the shifting mode of the vehicle. This changes whether or not and
        how the vehicle shifts gears depending on the RPM. Available modes are:

         * ``realistic_manual``: Gears have to be shifted manually by the
                                 user, including engaging the clutch.
         * ``realistic_manual_auto_clutch``: Gears have to be shifted manually
                                             by the user, without having to
                                             use the clutch.
         * ``arcade``: Gears shift up and down automatically. If the brake is
                       held, the vehicle automatically shifts into reverse
                       and accelerates backward until brake is released or
                       throttle is engaged.
         * ``realistic_automatic``: Gears shift up automatically, but reverse
                                    and parking need to be shifted to
                                    manually.

        Args:
            mode: The mode to set. Must be a string from the options listed above.

        Raises:
            BNGValueError: If an invalid mode is given.
        """
        return self._control.set_shift_mode(mode)

    def control(self, steering: float | None = None, throttle: float | None = None, brake: float | None = None,
                parkingbrake: float | None = None, clutch: float | None = None, gear: int | None = None) -> None:
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
        """
        return self._control.control(steering, throttle, brake, parkingbrake, clutch, gear)

    def set_color(self, rgba: Float4 = (1., 1., 1., 1.)) -> None:
        """
        Sets the color of this vehicle. Colour can be adjusted on the RGB
        spectrum and the "shininess" of the paint.

        Args:
            rgba: The new colour given as a tuple of RGBA floats, where
                  the alpha channel encodes the shininess of the paint.
        """
        return self._control.set_color(rgba)

    def set_velocity(self, velocity: float, dt: float = 1.0) -> None:
        """
        Sets the velocity of this vehicle. The velocity is not achieved instantly,
        it is acquired gradually over the time interval set by the `dt` argument.

        As the method of setting velocity uses physical forces, at high velocities
        it is important to set `dt` to an appropriately high value. The default
        `dt` value of 1.0 is suitable for velocities up to 30 m/s.

        Args:
            velocity: The target velocity in m/s.
            dt: The time interval over which the vehicle reaches the target velocity.
                Defaults to 1.0.
        """
        return self._control.set_velocity(velocity, dt)

    def set_lights(
            self, left_signal: bool | None = None, right_signal: bool | None = None, hazard_signal:
            bool | None = None, headlights: int | None = None, fog_lights: int | None = None,
            lightbar: int | None = None) -> None:
        """
        Sets the vehicle's lights to given intensity values. The lighting
        system features lights that are simply binary on/off, but also ones
        where the intensity can be varied. Binary lights include:

            * `left_signal`
            * `right_signal`
            * `hazard_signal`

        Non-binary lights vary between 0 for off, 1 for on, 2 for higher
        intensity. For example, headlights can be turned on with 1 and set to
        be more intense with 2. Non-binary lights include:

            * `headlights`
            * `fog_lights`
            * `lightbar`

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
        return self._control.set_lights(left_signal, right_signal, hazard_signal, headlights, fog_lights, lightbar)

    def queue_lua_command(self, chunk: str) -> None:
        """
        Executes lua chunk in the vehicle engine VM.

        Args:
            chunk: lua chunk as a string
        """
        return self._control.queue_lua_command(chunk)

    def recover(self) -> None:
        """
        Recovers the vehicle to a drivable position and state and repairs its damage.
        """
        return self._control.recover()