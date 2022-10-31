from __future__ import annotations

import logging
import os
import platform
import signal
import subprocess
from pathlib import Path
from time import sleep
from typing import TYPE_CHECKING, Any, List, Optional, cast

from beamngpy.api.beamng import (CameraApi, ControlApi, DebugApi,
                                 EnvironmentApi, ScenarioApi, SettingsApi,
                                 TrafficApi, VehiclesApi)
from beamngpy.connection import Connection
from beamngpy.logging import LOGGER_ID, BNGError, BNGValueError
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from beamngpy.connection import Response
    from beamngpy.scenario import Scenario

BINARIES = ['Bin64/BeamNG.tech.x64.exe', 'Bin64/BeamNG.drive.x64.exe']
BINARIES_LINUX = ['BinLinux/BeamNG.tech.x64', 'BinLinux/BeamNG.drive.x64']

module_logger = logging.getLogger(f"{LOGGER_ID}.beamng")
module_logger.setLevel(logging.DEBUG)


class BeamNGpy:
    """
    The BeamNGpy class is the backbone of communication with the BeamNG
    simulation and offers methods of starting, stopping, connecting to, and
    controlling the state of the simulator.

    Instantiates a BeamNGpy instance connecting to the simulator on the
    given host and port. The home directory of the simulator can be passed
    to this constructor. If None is given, this class tries to read a
    home path from the ``BNG_HOME`` environment variable.

    Note: If no home path is set, this class will not work properly.

    Args:
        host: The host to connect to
        port: The port to connect to
        home: Path to the simulator's home directory.
        user: Additional optional user path to set. This path can be
                    used to set where custom files created during
                    executions will be placed if the home folder shall not
                    be touched.
        remote: Set to true if using the BeamNGpy library on a
                different system than BeamNG.tech.

    Attributes
    ----------
        camera: CameraApi
        control: beamngpy.api.beamng.ControlApi
        debug: DebugApi
        environment: EnvironmentApi
        scenario: ScenarioApi
        settings: SettingsApi
        traffic: TrafficApi
        vehicles: VehiclesApi
    """

    def __init__(self, host: str, port: int, home: str | None = None, user: str | None = None, remote: bool = False):
        self.logger = logging.getLogger(f'{LOGGER_ID}.BeamNGpy')
        self.logger.setLevel(logging.DEBUG)
        self.host = host
        self.port = port
        self.home = home
        self.remote = remote
        self.process = None
        self._scenario: Scenario | None = None
        self.connection: Connection | None = None

        if not self.remote:
            if not self.home:
                self.home = os.getenv('BNG_HOME')
            if not self.home:
                raise BNGValueError('No BeamNG home folder given. Either specify '
                                    'one in the constructor or define an '
                                    'environment variable "BNG_HOME" that '
                                    'points to where your copy of BeamNG.* is.')

            self.home = Path(self.home).resolve()
            self.binary = self.determine_binary()

            if user:
                self.user = Path(user).resolve()
            else:
                self.user = self.determine_userpath()

        self._setup_api()

    def _setup_api(self):
        self.camera = CameraApi(self)
        self.set_free_camera = self.camera.set_free
        self.set_relative_camera = self.camera.set_relative
        self.set_player_camera_mode = self.camera.set_player_mode
        self.get_player_camera_modes = self.camera.get_player_modes
        self.get_annotations = self.camera.get_annotations
        self.get_annotation_classes = self.camera.get_annotation_classes

        self.control = ControlApi(self)
        self.step = self.control.step
        self.pause = self.control.pause
        self.resume = self.control.resume
        self.get_gamestate = self.control.get_gamestate
        self.queue_lua_command = self.control.queue_lua_command
        self.quit_beamng = self.control.quit_beamng
        self.display_gui_message = self.control.display_gui_message
        self.hide_hud = self.control.hide_hud
        self.show_hud = self.control.show_hud

        self.debug = DebugApi(self)
        self.add_debug_spheres = self.debug.add_spheres
        self.remove_debug_spheres = self.debug.remove_spheres
        self.add_debug_polyline = self.debug.add_polyline
        self.remove_debug_polyline = self.debug.remove_polyline
        self.add_debug_cylinder = self.debug.add_cylinder
        self.remove_debug_cylinder = self.debug.remove_cylinder
        self.add_debug_triangle = self.debug.add_triangle
        self.remove_debug_triangle = self.debug.remove_triangle
        self.add_debug_rectangle = self.debug.add_rectangle
        self.remove_debug_rectangle = self.debug.remove_rectangle
        self.add_debug_text = self.debug.add_text
        self.remove_debug_text = self.debug.remove_text
        self.add_debug_square_prism = self.debug.add_square_prism
        self.remove_debug_square_prism = self.debug.remove_square_prism

        self.env = EnvironmentApi(self)
        self.set_tod = self.env.set_tod
        self.set_weather_preset = self.env.set_weather_preset
        self.set_gravity = self.env.set_gravity

        self.scenario = ScenarioApi(self)
        self.get_levels = self.scenario.get_levels
        self.get_scenarios = self.scenario.get_scenarios
        self.get_level_scenarios = self.scenario.get_level_scenarios
        self.get_levels_and_scenarios = self.scenario.get_levels_and_scenarios
        self.get_current_scenario = self.scenario.get_current
        self.get_scenario_name = self.scenario.get_name
        self.get_current_vehicles_info = self.scenario.get_current_vehicles_info
        self.get_current_vehicles = self.scenario.get_current_vehicles
        self.load_scenario = self.scenario.load
        self.teleport_scenario_object = self.scenario.teleport_scenario_object
        self.start_scenario = self.scenario.start
        self.restart_scenario = self.scenario.restart
        self.stop_scenario = self.scenario.stop
        self.get_roads = self.scenario.get_roads
        self.get_road_edges = self.scenario.get_road_edges
        self.load_trackbuilder_track = self.scenario.load_trackbuilder_track

        self.settings = SettingsApi(self)
        self.change_setting = self.settings.change_setting
        self.apply_graphics_setting = self.settings.apply_graphics_setting
        self.set_deterministic = self.settings.set_deterministic
        self.set_nondeterministic = self.settings.set_nondeterministic
        self.set_steps_per_second = self.settings.set_steps_per_second
        self.remove_step_limit = self.settings.remove_step_limit
        self.set_particles_enabled = self.settings.set_particles_enabled

        self.traffic = TrafficApi(self)
        self.start_traffic = self.traffic.start
        self.stop_traffic = self.traffic.stop

        self.vehicles = VehiclesApi(self)
        self.spawn_vehicle = self.vehicles.spawn
        self.despawn_vehicle = self.vehicles.despawn
        self.get_available_vehicles = self.vehicles.get_available
        self.await_vehicle_spawn = self.vehicles.await_spawn
        self.switch_vehicle = self.vehicles.switch
        self.teleport_vehicle = self.vehicles.teleport
        self.get_part_annotation = self.vehicles.get_part_annotation
        self.get_part_annotations = self.vehicles.get_part_annotations

    def _kill_beamng(self) -> None:
        """
        Kills the running BeamNG.* process.
        """
        self.logger.info('Terminating BeamNG.tech process.')
        if self.connection:
            try:
                self.control.quit_beamng()
            except ConnectionResetError:
                self.connection = None
        if self.remote:
            self.logger.warn('cannot kill remote BeamNG.research process, aborting subroutine')
            return
        if not self.process:
            return
        if os.name == 'nt':
            with open(os.devnull, 'w') as devnull:
                subprocess.call(['taskkill', '/F', '/T', '/PID', str(self.process.pid)], stdout=devnull, stderr=devnull)
        else:
            try:
                os.kill(self.process.pid, signal.SIGTERM)
            except:
                pass
        self.process = None

    def open(self, extensions: Optional[List[str]] = None, *args: str, launch: bool = True, **opts: str) -> BeamNGpy:
        """
        Starts a BeamNG.* process, opens a server socket, and waits for the spawned BeamNG.* process to connect.
        This method blocks until the process started and is ready.

        Args:
            launch: Whether to launch a new process or connect to a running one on the configured host/port. Defaults to True.
        """
        self.connection = Connection(self.host, self.port)

        # try to connect to existing instance
        connected = self.connection.connect_to_beamng(tries=1, log_tries=False)
        if connected:
            self.logger.info('BeamNGpy successfully connected to existing BeamNG instance.')
            return self

        if launch:
            self.logger.info('Opening BeamNGpy instance.')
            self.start_beamng(extensions, *args, **opts)
            sleep(10)
        self.connection.connect_to_beamng()
        return self

    def close(self) -> None:
        """
        Kills the BeamNG.* process.
        """
        self.logger.info('Closing BeamNGpy instance.')
        if self._scenario:
            self._scenario.close()
            self._scenario = None
        self._kill_beamng()

    def send(self, data: StrDict) -> Response:
        if not self.connection:
            raise BNGError('Not connected to the simulator!')
        return self.connection.send(data)

    def message(self, req: str, **kwargs: Any) -> Any:
        if not self.connection:
            raise BNGError('Not connected to the simulator!')
        return self.connection.message(req, **kwargs)

    def determine_userpath(self) -> Path:
        """
        Tries to find the userpath based on the beamng installation if the user
        did not provide a custom userpath.
        """
        user = Path.home() / 'AppData'
        user = user / 'Local'
        if '.research' in self.binary:
            user = user / 'BeamNG.research'
        elif '.tech' in self.binary:
            user = user / 'BeamNG.tech'
        else:
            user = user / 'BeamNG.drive'
        self.logger.debug(f'Userpath is set to {user.as_posix()}')
        return user

    def determine_binary(self) -> str:
        """
        Tries to find one of the common BeamNG-binaries in the specified home
        path and returns the discovered path as a string.

        Returns:
            Path to the binary as a string.

        Raises:
            BNGError: If no binary could be determined.
        """
        self.home = cast(Path, self.home)

        choice = None
        binaries = BINARIES_LINUX if platform.system() == 'Linux' else BINARIES
        for option in binaries:
            binary = self.home / option
            if binary.exists():
                choice = binary
                break

        if not choice:
            raise BNGError('No BeamNG binary found in BeamNG home. Make '
                           'sure any of these exist in the BeamNG home '
                           f'folder: {", ".join(binaries)}')

        self.logger.debug(f'Determined BeamNG.* binary to be: {choice}')
        return str(choice)

    def prepare_call(self, extensions: Optional[List[str]], *args: str, **usr_opts: str) -> List[str]:
        """
        Prepares the command line call to execute to start BeamNG.*.
        according to this class' and the global configuration.

        Returns:
            List of shell components ready to be called in the
            :mod:`subprocess` module.
        """
        if extensions is None:
            extensions = []

        extensions.insert(0, 'tech/techCore')
        lua = ("registerCoreModule('{}');" * len(extensions))[:-1]
        lua = lua.format(*extensions)
        call = [self.binary, '-rport', str(self.port), '-nosteam']
        if platform.system() != 'Linux':  # console is not supported for Linux hosts yet
            call.append('-console')

        for arg in args:
            call.append(arg)

        call_opts = {'physicsfps': '4000', 'lua': lua}
        if 'lua' in usr_opts.keys():
            call_opts['lua'] = usr_opts['lua']

        if 'physicsfps' in usr_opts.keys():
            call_opts['physicsfps'] = usr_opts['physicsfps']

        for key, val in call_opts.items():
            call.extend(['-' + key, val])

        if self.user:
            call.append('-userpath')
            call.append(str(self.user))
            if ' ' in str(self.user):
                msg = 'Your configured userpath contains a space. ' \
                      'Unfortunately, this is known to cause issues in ' \
                      'launching BeamNG.tech. If you require a path with a ' \
                      'space in it, you can alternatively set it manually in' \
                      'the file "startup.ini" contained in the directory of ' \
                      'your BeamNG.tech installtion. This would not be ' \
                      'automatically updated if you change the `user` ' \
                      'parameter to `BeamNGpy`, but serves as a workaround ' \
                      'until the issue is fixed in BeamNG.tech.'
                self.logger.error(msg)

        call_str = ' '.join(call)
        self.logger.debug('Created system call for starting '
                          f'BeamNG process: `{call_str}`')
        return call

    def start_beamng(self, extensions: Optional[List[str]], *args: str, **opts: str) -> None:
        """
        Spawns a BeamNG.* process and retains a reference to it for later
        termination.
        """
        call = self.prepare_call(extensions, *args, **opts)

        if platform.system() == 'Linux':  # keep the same behaviour as on Windows - do not print game logs to the Python stdout
            self.process = subprocess.Popen(call, stdout=subprocess.DEVNULL)
        else:
            self.process = subprocess.Popen(call)
        self.logger.info('Started BeamNG.')

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
