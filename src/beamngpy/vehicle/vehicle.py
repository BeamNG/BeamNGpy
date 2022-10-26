from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any, Dict

from beamngpy.connection import Connection, Response
from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.sensors import State
from beamngpy.types import Float3, StrDict
from beamngpy.vehicle.api import AIApi, ControlApi, LoggingApi
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
        control: beamngpy.vehicle.api.ControlApi
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
        self.set_shift_mode = control.set_shift_mode
        self.set_color = control.set_color
        self.set_colour = control.set_color
        self.set_velocity = control.set_velocity
        self.set_lights = control.set_lights
        self.queue_lua_command = control.queue_lua_command

        self.logging = LoggingApi(self)
        self.set_in_game_logging_options_from_json = self.logging.set_in_game_logging_options_from_json
        self.write_in_game_logging_options_to_json = self.logging.write_in_game_logging_options_to_json
        self.start_in_game_logging = self.logging.start_in_game_logging
        self.stop_in_game_logging = self.logging.stop_in_game_logging

        self.attach_sensor = self.sensors.attach
        self.detach_sensor = self.sensors.detach
        self.poll_sensors = self.sensors.poll

    def _init_beamng_api(self, beamng: BeamNGpy | None = None):
        from beamngpy.beamng.api import BoundVehiclesApi

        # create dummy BeamNGpy object for API hints to work properly (it will be replaced during `connect`)
        if beamng is None:
            from beamngpy.beamng import BeamNGpy
            beamng = BeamNGpy('', -1, remote=True)

        api = BoundVehiclesApi(beamng, self)
        self.annotate_parts = api.annotate_parts
        self.revert_annotations = api.revert_annotations
        self.get_bbox = api.get_bbox
        self.get_part_options = api.get_part_options
        self.get_part_config = api.get_part_config
        self.set_part_config = api.set_part_config
        self.teleport = api.teleport

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

        Note that the `state` variable represents a *snapshot* of the last state. It has to be updated through :meth:`.Vehicle.update_vehicle`,
        which is made to retrieve the current state. Alternatively, for convenience, a call to :meth:`.Vehicle.poll_sensors` also updates the
        vehicle state along with retrieving sensor data.
        """
        return self.sensors[self._veh_state_sensor_id].data

    @state.setter
    def state(self, value: StrDict) -> None:
        self.sensors[self._veh_state_sensor_id].data = value

    @state.deleter
    def state(self) -> None:
        del self.sensors[self._veh_state_sensor_id].data

    def send(self, data: StrDict) -> Response:
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
