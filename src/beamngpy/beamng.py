from __future__ import annotations

import logging
import os
import platform
import signal
import subprocess
from pathlib import Path
from time import sleep
from typing import TYPE_CHECKING, Dict, List, Optional

from .beamngcommon import (ENV, LOGGER_ID, BNGError, BNGValueError, ack,
                           create_warning)
from .connection import Connection
from .scenario import ScenarioObject
from .vehicle import Vehicle

if TYPE_CHECKING:
    from .level import Level
    from .scenario import Scenario
    from .types import ConnData, Float3, Quat

BINARIES = ['Bin64/BeamNG.tech.x64.exe', 'Bin64/BeamNG.drive.x64.exe']
BINARIES_LINUX = ['BinLinux/BeamNG.tech.x64', 'BinLinux/BeamNG.drive.x64']

module_logger = logging.getLogger(f"{LOGGER_ID}.beamng")
module_logger.setLevel(logging.DEBUG)


def log_exception(extype, value, trace):
    """
    Hook to log uncaught exceptions to the logging framework. Register this as
    the excepthook with `sys.excepthook = log_exception`.
    """
    module_logger.exception("Uncaught exception: ", exc_info=(extype, value, trace))


class BeamNGpy:
    """
    The BeamNGpy class is the backbone of communication with the BeamNG
    simulation and offers methods of starting, stopping, connecting to, and
    controlling the state of the simulator.
    """

    def __init__(
            self, host: str, port: int, home: Optional[str] = None, user: Optional[str] = None, remote: bool = False):
        """
        Instantiates a BeamNGpy instance connecting to the simulator on the
        given host and port. The home directory of the simulator can be passed
        to this constructor. If None is given, this class tries to read a
        home path from the ``BNG_HOME`` environment variable.

        Note:
            If no home path is set, this class will not work properly.

        Args:
            host (str): The host to connect to
            port (int): The port to connect to
            home (str): Path to the simulator's home directory.
            user (str): Additional optional user path to set. This path can be
                        used to set where custom files created during
                        executions will be placed if the home folder shall not
                        be touched.
            remote (bool): Set to true if using the BeamNGpy library on a
                           different system than BeamNG.tech.
        """
        self.logger = logging.getLogger(f'{LOGGER_ID}.BeamNGpy')
        self.logger.setLevel(logging.DEBUG)
        self.host = host
        self.port = port
        self.home = home
        self.remote = remote
        self.process = None
        self.scenario: Optional[Scenario] = None
        self.connection: Optional[Connection] = None

        if not self.remote:
            if not self.home:
                self.home = ENV['BNG_HOME']
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

    def kill_beamng(self):
        """
        Kills the running BeamNG.* process.
        """
        self.logger.info('Terminating BeamNG.tech process.')
        if self.connection:
            try:
                self.quit_beamng()
            except ConnectionResetError:
                self.connection = None
        if self.remote:
            self.logger.warn('cannot kill remote BeamNG.research process, aborting subroutine')
            return
        if not self.process:
            return
        if os.name == "nt":
            with open(os.devnull, 'w') as devnull:
                subprocess.call(['taskkill', '/F', '/T', '/PID', str(self.process.pid)], stdout=devnull, stderr=devnull)
        else:
            try:
                os.kill(self.process.pid, signal.SIGTERM)
            except:
                pass
        self.process = None

    def open(self, extensions=None, *args, launch=True, **opts):
        """
        Starts a BeamNG.* process, opens a server socket, and waits for the spawned BeamNG.* process to connect.
        This method blocks until the process started and is ready.

        Args:
            launch (bool): Whether to launch a new process or connect to a running one on the configured host/port. Defaults to True.
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

    def close(self):
        """
        Kills the BeamNG.* process.
        """
        self.logger.info('Closing BeamNGpy instance.')
        if self.scenario:
            self.scenario.close()
            self.scenario = None
        self.kill_beamng()

    def send(self, data: ConnData):
        if not self.connection:
            raise BNGError('Not connected to the simulator!')
        return self.connection.send(data)

    def message(self, req: str, **kwargs):
        if not self.connection:
            raise BNGError('Not connected to the simulator!')
        return self.connection.message(req, **kwargs)

    def determine_userpath(self):
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

    def determine_binary(self):
        """
        Tries to find one of the common BeamNG-binaries in the specified home
        path and returns the discovered path as a string.

        Returns:
            Path to the binary as a string.

        Raises:
            BNGError: If no binary could be determined.
        """
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

    def prepare_call(self, extensions, *args, **usr_opts):
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

    def start_beamng(self, extensions, *args, **opts):
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

    def get_levels(self):
        """
        Queries the available levels in the simulator and returns them as a
        mapping of level name to :class:`.Level` instances.

        Returns:
            A dictionary of available level names to a corresponding instance
            of the :class:`.Level` class.
        """
        levels = self.message('GetLevels')
        levels = [Level.from_dict(l) for l in levels]
        levels = {l.name: l for l in levels}
        return levels

    def get_scenarios(self, levels=None):
        """
        Queries the available scenarios and returns them as a mapping of
        paths to :class:`.Scenario` instances. The scenarios are constructed
        to point to their parent levels, so to avoid extra queries to the
        simulator about existing levels, a cache of available levels can be
        passed to this method.

        Args:
            levels (dict): A dictionary of level names to :class:`.Level`
                           instances to fill in the parent level of returned
                           scenarios.

        Returns:
            A mapping of scenario paths to their corresponding
            :class:`.Scenario` instance.
        """
        if levels is None:
            levels = self.get_levels()

        scenarios = self.message('GetScenarios')
        scenarios = [Scenario.from_dict(s) for s in scenarios]
        scenarios = {s.path: s for s in scenarios if s.level in levels.keys()}
        for _, scenario in scenarios.items():
            assert isinstance(scenario.level, str)
            scenario.level = levels[scenario.level]

        return scenarios

    def get_level_scenarios(self, level):
        """
        Queries the simulator for all scenarios available in the  given level.

        Args:
            level: The level to get scenarios for. Can either be the name of
                   the level as a string or an instance of :class:`.Level`

        Returns:
            A mapping of scenario paths to their corresponding
            :class:`.Scenario` instance.
        """
        level_name = None
        if isinstance(level, Level):
            level_name = level.name
        else:
            level_name = level

        scenarios = self.message('GetScenarios')
        scenarios = [Scenario.from_dict(s) for s in scenarios]
        scenarios = {s.path: s for s in scenarios if s.level == level_name}

        for scenario in scenarios.values():
            scenario.level = level

        return scenarios

    def get_levels_and_scenarios(self):
        """
        Utility method that retrieves all levels and scenarios and returns
        them as a tuple of (levels, scenarios).

        Returns:
            (:meth:`~BeamNGpy.get_levels`, :meth:`~BeamNGpy.get_scenarios`)
        """
        levels = self.get_levels()
        scenarios = self.get_scenarios(levels=levels)

        for scenario in scenarios.values():
            assert isinstance(scenario.level, Level)
            level_scenarios = scenario.level.scenarios
            level_scenarios[scenario.path] = scenario

        return levels, scenarios

    def get_current_scenario(self, levels=None):
        """
        Queries the currently loaded scenario from the simulator.

        Args:
            levels (dict): A mapping of level names to :class:`.Level` instances

        Returns:
            A :class:`.Scenario` instance of the currently-loaded scenario. If
            the ``levels`` parameter contains the level the scenario is taking
            place in, the scenario's parent level field will be filled in
            accordingly.
        """
        scenario = self.message('GetCurrentScenario')
        if not scenario:
            raise BNGError('The current scenario could not be retrieved.')
        scenario = Scenario.from_dict(scenario)

        if levels is not None:
            if scenario.level in levels:
                scenario.level = levels[scenario.level]

        return scenario

    def get_current_vehicles_info(self) -> Dict[str, Dict]:
        """
        Queries the currently active vehicles in the simulator.

        Returns:
            A mapping of vehicle IDs to instances of the :class:`.Vehicle`
            class for each active vehicle. These vehicles are not connected to
            by this function.
        """
        return self.message('GetCurrentVehicles')

    def get_current_vehicles(self):
        vehicles = self.get_current_vehicles_info()
        vehicles = {n: Vehicle.from_dict(v) for n, v in vehicles.items()}
        return vehicles

    def hide_hud(self):
        """
        Hides the HUD in the simulator.
        """
        data = dict(type='HideHUD')
        return self.send(data)

    def show_hud(self):
        """
        Shows the HUD in the simulator.
        """
        data = dict(type='ShowHUD')
        return self.send(data)

    def load_scenario(self, scenario: Scenario):
        """
        Loads the given scenario in the simulation and returns once loading
        is finished.

        Args:
            scenario (:class:`.Scenario`): The scenario to load.
        """
        # clean up the vehicle connections if the `scenario` object is reused multiple times
        for vehicle in scenario.vehicles:
            if vehicle.connection:
                vehicle.disconnect()

        data = {'type': 'LoadScenario', 'path': scenario.path}
        self.send(data).ack('MapLoaded')
        self.logger.info('Loaded map.')
        self.scenario = scenario
        self.scenario.connect(self)

    @ack('SetEngineFlags')
    def set_engine_flags(self, flags):
        """
        Sets flags in the simulation engine. Flags are given as key/value pairs
        of strings and booleans, where each string specifies a flag and the
        boolean the state to set. Possible flags are:

         * ``annotations``: Whether pixel-wise annotation should be enabled.

        """
        flags = dict(type='EngineFlags', flags=flags)
        self.logger.debug(f'set following engine flags: {flags}')
        return self.send(flags)

    def teleport_vehicle(self, vehicle_id: str, pos: Float3, rot_quat: Optional[Quat] = None, reset=True):
        """
        Teleports the given vehicle to the given position with the given
        rotation.

        Args:
            vehicle_id (string): The id/name of the vehicle to teleport.
            pos (tuple): The target position as an (x,y,z) tuple containing
                         world-space coordinates.
            rot_quat (tuple): Optional tuple (x, y, z, w) specifying vehicle
                              rotation as quaternion
            reset (bool): Specifies if the vehicle will be reset to its initial
                          state during teleport (including its velocity).

        Notes:
            The ``reset=False`` option is incompatible with setting rotation of
            the vehicle. With the current implementation, it is not possible to
            set the rotation of the vehicle and to keep its velocity during teleport.
        """
        self.logger.info(f'Teleporting vehicle <{vehicle_id}>.')
        data: ConnData = dict(type='Teleport')
        data['vehicle'] = vehicle_id
        data['pos'] = pos
        data['reset'] = reset
        if rot_quat:
            data['rot'] = rot_quat
        if not reset and rot_quat:
            create_warning('the usage of `reset=False` is incompatible with '
                           'the usage of `rot_quat` in `beamng.teleport_vehicle`; '
                           'rotation will not be applied to the vehicle',
                           RuntimeWarning)
        resp = self.send(data).recv('Teleported')
        return resp['success']

    @ack('ScenarioObjectTeleported')
    def teleport_scenario_object(self, scenario_object: ScenarioObject, pos: Float3, rot_quat: Optional[Quat] = None):
        """
        Teleports the given scenario object to the given position with the
        given rotation.

        Args:
            scenario_object (:class:`.ScenarioObject`): The vehicle to
                                                        teleport.
            pos (tuple): The target position as an (x,y,z) tuple containing
                         world-space coordinates.
            rot_quat (tuple): Optional tuple specifying object rotation as a
                              quaternion
        """
        data: ConnData = dict(type='TeleportScenarioObject')
        data['id'] = scenario_object.id
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        return self.send(data)

    @ack('ScenarioStarted')
    def start_scenario(self, restrict_actions=False):
        """
        Starts the scenario; equivalent to clicking the "Start" button in the
        game after loading a scenario. This method blocks until the countdown
        to the scenario's start has finished.

        Args:
            restrict_actions (bool): Whether to keep scenario restrictions,
                                     such as limited menu options and controls.
                                     Defaults to False.
        """
        data: ConnData = dict(type='StartScenario')
        data['restrict_actions'] = restrict_actions
        resp = self.send(data)
        self.logger.info('Starting scenario.')
        return resp

    def restart_scenario(self):
        """
        Restarts a running scenario.
        """
        if not self.scenario:
            raise BNGError('Need to have a scenario loaded to restart it.')

        vehicles_to_reconnect = [v.vid for v in self.scenario.vehicles if v.is_connected()]
        self.scenario.restart()

        self.logger.info('Restarting scenario.')
        data = dict(type='RestartScenario')
        self.send(data).ack('ScenarioRestarted')

        self.scenario._load_existing_vehicles()
        for vehicle in self.scenario.vehicles:
            if vehicle.vid in vehicles_to_reconnect and not vehicle.is_connected():
                vehicle.connect(self)

    @ack('ScenarioStopped')
    def stop_scenario(self):
        """
        Stops a running scenario and returns to the main menu.
        """
        if not self.scenario:
            raise BNGError('Need to have a scenario loaded to stop it.')

        self.scenario.close()
        self.scenario = None

        data = dict(type='StopScenario')
        resp = self.send(data)
        self.logger.info('Stopping scenario.')
        return resp

    @ack('SetPhysicsDeterministic')
    def set_deterministic(self):
        """
        Sets the simulator to run in deterministic mode. For this to function
        properly, an amount of steps per second needs to have been specified
        in the simulator's settings, or through
        :meth:`~.BeamnGpy.set_steps_per_second`.
        """
        data = dict(type='SetPhysicsDeterministic')
        return self.send(data)

    @ack('SetPhysicsNonDeterministic')
    def set_nondeterministic(self):
        """
        Disables the deterministic mode of the simulator. Any steps per second
        setting is retained.
        """
        data = dict(type='SetPhysicsNonDeterministic')
        return self.send(data)

    @ack('SetFPSLimit')
    def set_steps_per_second(self, sps):
        """
        Specifies the temporal resolution of the simulation. The setting can be
        understood to determine into how many steps the simulation divides one
        second of simulation. A setting of two, for example, would mean one
        second is simulated in two steps. Conversely, to simulate one second,
        one needs to advance the simulation two steps.

        Args:
            sps (int): The steps per second to set.
        """
        data = dict(type='FPSLimit', fps=sps)
        return self.send(data)

    @ack('RemovedFPSLimit')
    def remove_step_limit(self):
        """
        Removes the steps-per-second setting, making the simulation run at
        undefined time slices.
        """
        data = dict(type='RemoveFPSLimit')
        return self.send(data)

    def step(self, count, wait=True):
        """
        Advances the simulation the given amount of steps, assuming it is
        currently paused. If the wait flag is set, this method blocks until
        the simulator has finished simulating the desired amount of steps. If
        not, this method resumes immediately. This can be used to queue
        commands that should be executed right after the steps have been
        simulated.

        Args:
            count (int): The amount of steps to simulate.
            wait (bool): Optional. Whether to wait for the steps to be
                         simulated. Defaults to True.

        Raises:
            BNGError: If the wait flag is set but the simulator doesn't respond
                      appropriately.
        """
        data: ConnData = dict(type='Step', count=count)
        data['ack'] = wait
        resp = self.send(data)
        if wait:
            resp.ack('Stepped')
        self.logger.info(f'Advancing the simulation by {count} steps.')

    @ack('Paused')
    def pause(self):
        """
        Sends a pause request to BeamNG.*, blocking until the simulation is
        paused.
        """
        data = dict(type='Pause')
        resp = self.send(data)
        self.logger.info('Pausing the simulation.')
        return resp

    @ack('Resumed')
    def resume(self):
        """
        Sends a resume request to BeamNG.*, blocking until the simulation
        is resumed.
        """
        data = dict(type='Resume')
        resp = self.send(data)
        self.logger.info('Resuming the simulation.')
        return resp

    def get_roads(self):
        """
        Retrieves the vertex data of all DecalRoads in the current scenario.
        The vertex data of a DecalRoad is formatted as point triples, where
        each triplet represents the left, centre, and right points of the edges
        that make up a DecalRoad.

        Returns:
            A dict mapping DecalRoad IDs to lists of point triples.
        """
        if not self.scenario:
            raise BNGError('Need to be in a started scenario to get its '
                           'DecalRoad data.')

        data = dict(type='GetDecalRoadData')
        resp = self.send(data).recv('DecalRoadData')
        return resp['data']

    def get_road_edges(self, road):
        """
        Retrieves the edges of the road with the given name and returns them
        as a list of point triplets. Roads are defined by a series of lines
        that specify the leftmost, center, and rightmost point in the road.
        These lines go horizontally across the road and the series of leftmost
        points make up the left edge of the road, the series of rightmost
        points make up the right edge of the road, and the series of center
        points the middle line of the road.

        Args:
            road (str): Name of the road to get edges from.

        Returns:
            The road edges as a list of (left, center, right) point triplets.
            Each point is an (X, Y, Z) coordinate triplet.
        """
        data = dict(type='GetDecalRoadEdges')
        data['road'] = road
        resp = self.send(data).recv('DecalRoadEdges')
        return resp['edges']

    def get_gamestate(self):
        """
        Retrieves the current game state of the simulator. The game state is
        returned as a dictionary containing a ``state`` entry that is either:

            * ``scenario`` when a scenario is loaded
            * ``menu`` otherwise

        If a scenario is loaded, the resulting dictionary also contains a
        ``scenario_state`` entry whose value is ``pre-running`` if the scenario
        is currently at the start screen or ``running`` otherwise.

        Returns:
            The game state as a dictionary as described above.
        """
        data = dict(type='GameStateRequest')
        resp = self.send(data).recv('GameState')
        return resp

    @ack('TimeOfDayChanged')
    def set_tod(self, tod):
        """
        Sets the current time of day. The time of day value is given as a float
        between 0 and 1. How this value affects the lighting of the scene is
        dependant on the map's TimeOfDay object.

        Args:
            tod (float): Time of day beteen 0 and 1.
        """
        data = dict(type='TimeOfDayChange')
        data['tod'] = tod
        return self.send(data)

    @ack('WeatherPresetChanged')
    def set_weather_preset(self, preset, time=1):
        """
        Triggers a change to a different weather preset. Weather presets affect
        multiple settings at once (time of day, wind speed, cloud coverage,
        etc.) and need to have been defined first. Example json objects
        defining weather presets can be found in BeamNG.tech's
        ``art/weather/defaults.json`` file.

        Args:
            preset (str): The name of the preset to switch to. Needs to be
                          defined already within the simulation.
            time (float): Time in seconds the transition from the current
                          settings to the preset's should take.
        """
        data: ConnData = dict(type='SetWeatherPreset')
        data['preset'] = preset
        data['time'] = time
        return self.send(data)

    def await_vehicle_spawn(self, vid):
        """
        Waits for the vehicle with the given name to spawn and returns once it
        has.

        Args:
            vid (str): The name of the  vehicle to wait for.
        """
        data = dict(type='WaitForSpawn')
        data['name'] = vid
        resp = self.send(data).recv('VehicleSpawned')
        assert resp['name'] == vid

    def update_scenario(self):
        """
        Updates the :attr:`.Vehicle.state` field of each vehicle in the
        currently running scenario.
        """
        if not self.scenario:
            raise BNGError('Need to have a senario loaded to update it.')

        data: ConnData = dict(type='UpdateScenario')
        data['vehicles'] = list()
        for vehicle in self.scenario.vehicles:
            data['vehicles'].append(vehicle.vid)
        resp = self.send(data).recv('ScenarioUpdate')

        scenario_vehicles = {vehicle.vid: vehicle for vehicle in self.scenario.vehicles}
        for name, vehicle_state in resp['vehicles'].items():
            vehicle = scenario_vehicles.get(name, None)
            if vehicle:
                vehicle.state = vehicle_state

    @ack('GuiMessageDisplayed')
    def display_gui_message(self, msg):
        """
        Displays a toast message in the user interface of the simulator.

        Args:
            msg (str): The message to display.
        """
        data = dict(type='DisplayGuiMessage')
        data['message'] = msg
        return self.send(data)

    @ack('VehicleSwitched')
    def switch_vehicle(self, vehicle):
        """
        Switches to the given :class:`.Vehicle`. This means that the
        simulator's main camera, inputs by the user, and so on will all focus
        on that vehicle from now on.

        Args:
            vehicle (:class:`.Vehicle`): The target vehicle.
        """
        data = dict(type='SwitchVehicle')
        data['vid'] = vehicle.vid
        return self.send(data)

    @ack('FreeCameraSet')
    def set_free_camera(self, pos, direction):
        """
        Sets the position and direction of the free camera. The free camera is
        one that does not follow any particular vehicle, but can instead be
        put at any spot and any position on the map.

        Args:
            pos (tuple): The position of the camera as a (x, y, z) triplet.
            direction (tuple): The directional vector of the camera as a
                               (x, y, z) triplet.
        """
        data = dict(type='SetFreeCamera')
        data['pos'] = pos
        data['dir'] = direction
        return self.send(data)

    @ack('ParticlesSet')
    def set_particles_enabled(self, enabled):
        """
        En-/disabled visual particle emmission.

        Args:
            enabled (bool): Whether or not to en- or disabled effects.
        """
        data = dict(type='ParticlesEnabled')
        data['enabled'] = enabled
        return self.send(data)

    def get_part_annotations(self, vehicle):
        data = dict(type='GetPartAnnotations')
        data['vid'] = vehicle.vid
        resp = self.send(data).recv('PartAnnotations')
        return resp['colors']

    def get_part_annotation(self, part):
        data = dict(type='GetPartAnnotation')
        data['part'] = part
        resp = self.send(data).recv('PartAnnotation')
        if 'color' in resp:
            return resp['color']
        return None

    def get_scenario_name(self):
        """
        Retrieves the name of the currently-loaded scenario in the simulator.

        Returns:
            The name of the loaded scenario as a string.
        """
        data = dict(type='GetScenarioName')
        resp = self.send(data).recv('ScenarioName')
        return resp['name']

    def get_scenetree(self):
        """
        Queries the scene tree of the current scenario/level. The scene tree
        is returned as a tree of :class:`.SceneObject` instances the root of
        which is of the ``SimGroup`` class.

        Returns:
            A tree of :class:`.SceneObject` objects that contains all objects
            in the current scene. The respective objects do not contain
            type-specific information. This information can be obtained on a
            per-object basis using :meth:`~BeamNGpy.get_scene_object_data`.
        """
        return self.message('GetSceneTree')

    def get_scene_object_data(self, obj_id: str):
        """
        Retrieves all available key/value pairs the simulation offers for the
        given object as a dictionary.

        Returns:
            A dictionary of key/values the simulator offers for the object of
            the given ID.
        """
        return self.message('GetObject', id=obj_id)

    def spawn_vehicle(self, vehicle: Vehicle, pos: Float3, rot_quat: Quat = (0, 0, 0, 1), cling=True):
        """
        Spawns the given :class:`.Vehicle` instance in the simulator. This
        method is meant for spawning vehicles *during the simulation*. Vehicles
        that are known to be required before running the simulation should be
        added during scenario creation instead. Cannot spawn two vehicles with
        the same id/name.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to be spawned.
            pos (tuple): Where to spawn the vehicle as a (x, y, z) triplet.
            rot_quat (tuple): Vehicle rotation in form of a quaternion
            cling (bool): If set, the z-coordinate of the vehicle's position
                          will be set to the ground level at the given
                          position to avoid spawning the vehicle below ground
                          or in the air.

        Returns:
            bool indicating whether the spawn was successful or not

        """
        data: ConnData = dict(type='SpawnVehicle', cling=cling)
        data['name'] = vehicle.vid
        data['model'] = vehicle.options['model']
        data['pos'] = pos
        data['rot'] = rot_quat
        data.update(vehicle.options)
        resp = self.send(data).recv('VehicleSpawned')
        if resp['success']:
            vehicle.connect(self)
        return resp['success']

    @ack('VehicleDespawned')
    def despawn_vehicle(self, vehicle: Vehicle):
        """
        Despawns the given :class:`.Vehicle` from the simulation.

        Args:
            vehicle (:class:`.Vehicle`): The vehicle to despawn.
        """
        vehicle.disconnect()
        data = dict(type='DespawnVehicle')
        data['vid'] = vehicle.vid
        return self.send(data)

    def find_objects_class(self, clazz: str):
        """
        Scans the current environment in the simulator for objects of a
        certain class and returns them as a list of :class:`.ScenarioObject`.

        What kind of classes correspond to what kind of objects is described
        in the BeamNG.drive documentation.

        Args:
            clazz (str): The class name of objects to find.

        Returns:
            Found objects as a list.
        """
        data = dict(type='FindObjectsClass')
        data['class'] = clazz
        resp = self.send(data).recv()
        ret: List[ScenarioObject] = list()
        for obj in resp['objects']:
            sobj = ScenarioObject(obj['id'], obj['name'], obj['type'],
                                  tuple(obj['position']),
                                  tuple(obj['scale']),
                                  rot_quat=tuple(obj['rotation']),
                                  **obj['options'])
            ret.append(sobj)
        return ret

    @ack('GravitySet')
    def set_gravity(self, gravity=-9.807):
        """
        Sets the strength of gravity in the simulator.

        Args:
            gravity (float): The gravity value to set. The default one is
                             that of earth (-9.807)
        """
        data: ConnData = dict(type='SetGravity')
        data['gravity'] = gravity
        return self.send(data)

    def get_available_vehicles(self):
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
        return self.send(data).recv('AvailableVehicles')

    @ack('TrafficStarted')
    def start_traffic(self, participants: List[Vehicle]):
        """
        Enables traffic simulation for the given list of vehicles.

        Args:
            participants (list): List of vehicles that will be part of the
                                 simulation. These vehicles need to be spawned
                                 beforehand and the simulation will take
                                 control of them.
        """
        data: ConnData = dict(type='StartTraffic')
        data['participants'] = [p.vid for p in participants]
        return self.send(data)

    @ack('TrafficStopped')
    def stop_traffic(self, stop=False):
        """
        Stops the traffic simulation.

        Args:
            stop (bool): Whether or not to stop the vehicles participating in
                         traffic. If True, vehicles will come to a halt, if
                         False, the AI will simply stop controlling the
                         vehicle.
        """
        data: ConnData = dict(type='StopTraffic')
        data['stop'] = stop
        return self.send(data)

    @ack('SettingsChanged')
    def change_setting(self, key, value):
        """
        Changes a setting in the game. Examples of the key and value pairs
        given to this method can be found in your game's settings ini files.
        These are usually in <userpath>/settings/game-settings.ini or
        <userpath>/settings/cloud/game-settings-cloud.ini.

        Args:
            key (str): The key of the setting that is to be changed
            value (str): The desired value.
        """
        data = dict(type='ChangeSetting')
        data['key'] = key
        data['value'] = value
        return self.send(data)

    @ack('GraphicsSettingApplied')
    def apply_graphics_setting(self):
        """
        Makes the game apply a graphics setting that has been changed since
        startup or the last time settings were applied. A call to this is
        required after changing settings like whether or not the game is
        in fullscreen or the resolution, otherwise those settings will only
        take effect after the next launch.
        """
        data = dict(type='ApplyGraphicsSetting')
        return self.send(data)

    @ack('ExecutedLuaChunkGE')
    def queue_lua_command(self, chunk: str):
        """
        Executes one lua chunk in the game engine VM.

        Args:
            chunk(str): lua chunk as a string
        """
        data = dict(type='QueueLuaCommandGE')
        data['chunk'] = chunk
        return self.send(data)

    @ack('RelativeCamSet')
    def set_relative_camera(self, pos: Float3, rot_quat: Optional[Quat] = None):
        """
        Switches the camera mode for the currently-entered vehicle to the
        'relative' mode in which the camera can be placed at an arbitrary point
        relative to the vehicle, moving along with it as it drives around.

        Args:
            pos (tuple): (x, y, z) tuple of the camera's position relative to
                         the vehicle.
            rot (tuple): Euler angles expressing the rotation of the camera.
            rot_quat (tuple): The camera's rotation but written as a quat.
        """
        data: ConnData = dict(type='SetRelativeCam')
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        return self.send(data)

    def add_debug_spheres(self, coordinates, radii, rgba_colors,
                                cling=False, offset=0):
        data: ConnData = dict(type="AddDebugSpheres")
        assert len(coordinates) == len(radii) == len(rgba_colors)
        data['coordinates'] = coordinates
        data['radii'] = radii
        data['colors'] = rgba_colors
        data['cling'] = cling
        data['offset'] = offset
        resp = self.send(data).recv('DebugSphereAdded')
        return resp['sphereIDs']

    @ack('DebugObjectsRemoved')
    def remove_debug_spheres(self, sphere_ids: List[str]):
        data: ConnData = dict(type='RemoveDebugObjects')
        data['objType'] = 'spheres'
        data['objIDs'] = sphere_ids
        return self.send(data)

    def add_debug_polyline(self, coordinates, rgba_color,
                                 cling=False, offset=0):
        data: ConnData = dict(type='AddDebugPolyline')
        data['coordinates'] = coordinates
        data['color'] = rgba_color
        data['cling'] = cling
        data['offset'] = offset
        resp = self.send(data).recv('DebugPolylineAdded')
        return resp['lineID']

    @ack('DebugObjectsRemoved')
    def remove_debug_polyline(self, line_id: str):
        data: ConnData = dict(type='RemoveDebugObjects')
        data['objType'] = 'polylines'
        data['objIDs'] = [line_id]
        return self.send(data)

    def add_debug_cylinder(self, circle_positions, radius, rgba_color):
        data: ConnData = dict(type='AddDebugCylinder')
        data['circlePositions'] = circle_positions
        data['radius'] = radius
        data['color'] = rgba_color
        resp = self.send(data).recv('DebugCylinderAdded')
        return resp['cylinderID']

    @ack('DebugObjectsRemoved')
    def remove_debug_cylinder(self, cylinder_id):
        data: ConnData = dict(type='RemoveDebugObjects')
        data['objType'] = 'cylinders'
        data['objIDs'] = [cylinder_id]
        return self.send(data)

    def add_debug_triangle(self, vertices, rgba_color, cling=False, offset=0):
        data: ConnData = dict(type='AddDebugTriangle')
        data['vertices'] = vertices
        data['color'] = rgba_color
        data['cling'] = cling
        data['offset'] = offset
        resp = self.send(data).recv('DebugTriangleAdded')
        return resp['triangleID']

    @ack('DebugObjectsRemoved')
    def remove_debug_triangle(self, triangle_id):
        data: ConnData = dict(type='RemoveDebugObjects')
        data['objType'] = 'triangles'
        data['objIDs'] = [triangle_id]
        return self.send(data)

    def add_debug_rectangle(self, vertices, rgba_color, cling=False, offset=0):
        data: ConnData = dict(type='AddDebugRectangle')
        data['vertices'] = vertices
        data['color'] = rgba_color
        data['cling'] = cling
        data['offset'] = offset
        resp = self.send(data).recv('DebugRectangleAdded')
        return resp['rectangleID']

    @ack('DebugObjectsRemoved')
    def remove_debug_rectangle(self, rectangle_id: str):
        data: ConnData = dict(type='RemoveDebugObjects')
        data['objType'] = 'rectangles'
        data['objIDs'] = [rectangle_id]
        return self.send(data)

    def add_debug_text(self, origin, content, rgba_color, cling=False, offset=0):
        data: ConnData = dict(type='AddDebugText')
        data['origin'] = origin
        data['content'] = content
        data['color'] = rgba_color
        data['cling'] = cling
        data['offset'] = offset
        resp = self.send(data).recv('DebugTextAdded')
        return resp['textID']

    @ack('DebugObjectsRemoved')
    def remove_debug_text(self, text_id: str):
        data: ConnData = dict(type='RemoveDebugObjects')
        data['objType'] = 'text'
        data['objIDs'] = [text_id]
        return self.send(data)

    def add_debug_square_prism(self, end_points, end_point_dims, rgba_color):
        data = dict(type='AddDebugSquarePrism')
        data['endPoints'] = end_points
        data['dims'] = end_point_dims
        data['color'] = rgba_color
        resp = self.send(data).recv('DebugSquarePrismAdded')
        return resp['prismID']

    @ack('DebugObjectsRemoved')
    def remove_debug_square_prism(self, prism_id: str):
        data: ConnData = dict(type='RemoveDebugObjects')
        data['objType'] = 'squarePrisms'
        data['objIDs'] = [prism_id]
        return self.send(data)

    def get_annotations(self):
        """
        Method to obtain the annotation configuration of the simulator.

        Returns:
            A mapping of object classes to lists containing the [R, G, B]
            values of the colors objects of that class are rendered with.
        """
        data = dict(type='GetAnnotations')
        resp = self.send(data).recv('Annotations')
        return resp['annotations']

    def get_annotation_classes(self, annotations):
        """
        Method to convert the annotation configuration of the simulator into
        a mapping of colors to the corresponding object classes.

        Args:
            annotations (dict): The annotation configuration of the simulator.
                                Expected to be in the format
                                `get_annotations()` returns.

        Returns:
            A mapping of colors encoded as 24bit integers to object classes
            according to the simulator.
        """
        classes = {}
        for k, v in annotations.items():
            key = v[0] * 256 * 256 + v[1] * 256 + v[2]
            classes[key] = k
        return classes

    def create_scenario(self, level, name, prefab, info):
        """
        Prompts the simulator to create files required for a scenario. Namely,
        the "info.json" containing scenario information and the contents of the
        scenario prefab containing objects in the scene.

        Args:
            level (str): The name of the level the new scenario is in
            name (str): The name of the scenario
            prefab (str): Contents of the scenario's prefab file
            info (dict): Contents of the scenario's info.json
        """
        return self.message('CreateScenario', level=level, name=name, prefab=prefab, info=info)

    def delete_scenario(self, path):
        """
        Prompts the simulator to delete files of the scenario at the given
        path.

        Args:
            path (str): The path to the scenario relative to the
            user directory.
        """
        self.message('DeleteScenario', path=path)

    @ack('Quit')
    def quit_beamng(self):
        data = dict(type='Quit')
        return self.send(data)

    @ack('PlayerCameraModeSet')
    def set_player_camera_mode(self, vid, mode, config, custom_data=None):
        """
        Sets the camera mode of the vehicle identified by the given vehicle ID.
        The mode is given as a string that identifies one of the valid modes
        offered by the simulator. These modes can be queried using the
        (:meth:`~BeamNGpy.get_player_camera_mode`) method.

        The camera can be further configured with some common parameters,
        but it is not guaranteed the camera mode will respect all of them.
        These parameters include:

         * rotation: The rotation of the camera as a triplet of Euler angles
         * fov: The field of view angle
         * offset: The (x, y, z) vector to offset the camera's position by
         * distance: The distance of the camera to the vehicle

        Since each camera mode is implemented as a custom Lua extension, it is
        not possible to automatically query the exact features of the mode.
        Further information can be found in the
        lua/ge/extensions/core/cameraModes files which contain the
        implementations of each camera mode.

        Args:
            vid (str): Vehicle ID of the vehice to change the mode of.
            mode (str): Camera mode to set.
            config (dict): Dictionary of further properties to set in the mode.
            custom_data (dict): Custom data used by the specific camera mode. Defaults to None.
        """
        data: ConnData = dict(type='SetPlayerCameraMode')
        data['vid'] = vid
        data['mode'] = mode
        data['config'] = config
        data['customData'] = custom_data
        return self.send(data)

    def get_player_camera_modes(self, vid: str):
        """
        Retrieves information about the camera modes configured for the vehicle
        identified by the given ID.

        Args:
            vid (str): Vehicle ID of the vehicle to get camera mode information
                       of.

        Returns:
            A dictionary mapping camera mode names to configuration options.
        """
        data = dict(type='GetPlayerCameraMode')
        data['vid'] = vid
        resp = self.send(data).recv('PlayerCameraMode')
        return resp['cameraData']

    @ack('TrackBuilderTrackLoaded')
    def load_trackbuilder_track(self, path: str):
        """
        Spawns a TrackBuilder track provided by the given path to a TrackBuilder
        ``.json`` file.

        Args:
            path (str): Path to a ``.json`` file created by TrackBuilder.
        """
        data = dict(type='LoadTrackBuilderTrack')
        data['path'] = path
        return self.send(data)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()