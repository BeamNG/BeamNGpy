from __future__ import annotations

import logging
import os
import platform
import signal
import subprocess
from pathlib import Path
from time import sleep
from typing import TYPE_CHECKING, Any, List

from beamngpy.api.beamng import (CameraApi, ControlApi, DebugApi,
                                 EnvironmentApi, ScenarioApi, SettingsApi,
                                 SystemApi, TrafficApi, UiApi, VehiclesApi)
from beamngpy.beamng import filesystem
from beamngpy.connection import Connection
from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from beamngpy.connection import Response
    from beamngpy.scenario import Scenario


module_logger = logging.getLogger(f"{LOGGER_ID}.beamng")
module_logger.setLevel(logging.DEBUG)


class BeamNGpy:
    """
    The :class:`BeamNGpy` class is the backbone of communication with the BeamNG
    simulation and offers methods of starting, stopping, connecting to, and
    controlling the state of the simulator.

    Instantiates a BeamNGpy instance connecting to the simulator on the
    given host and port. The home directory of the simulator can be passed
    to this constructor. If ``None`` is given, this class tries to read a
    home path from the ``BNG_HOME`` environment variable.

    Args:
        host: The host to connect to.
        port: The port to connect to.
        home: Path to the simulator's home directory.
        binary: Optional custom path to the binary, relative to the simulator's
                home directory. Default is ``Bin64/BeamNG.{tech/drive}.x64.exe``
                for Windows hosts, ``BinLinux/BeamNG.{tech/drive}.x64`` for Linux
                hosts.
        user: Additional optional user path to set. This path can be
              used to set where custom files created during executions
              will be placed if the home folder shall not be touched.
        quit_on_close: Whether the simulator should be closed when :func:`close()` is called.
                       Defaults to True.
        debug: If True, then sets BeamNG.tech communication to debug mode. That means:

                1. BeamNG will not respond to BeamNGpy requests when a Lua error
                    happens and prints the stacktrace instead.
                2. The ``techCapture.*.log`` files are created automatically in the userfolder,
                    they log every protocol call and can be replayed using the ``tech/capturePlayer``
                    Lua extension.

               This option is applicable only when the process is launched by this instance
               of BeamNGpy, as it sets a launch argument of the process. Defaults to False.

    Attributes
    ----------
        camera: CameraApi
            The API module to control the camera in the simulator.
            See :class:`.CameraApi` for details.
        control: ControlApi
            The API module to control the flow of the simulation.
            See :class:`.ControlApi` for details.
        debug: DebugApi
            The API module to control debug objects.
            See :class:`.DebugApi` for details.
        env: EnvironmentApi
            The API module to control the simulation's environment.
            See :class:`.EnvironmentApi` for details.
        scenario: ScenarioApi
            The API module to control the scenarios.
            See :class:`.ScenarioApi` for details.
        settings: SettingsApi
            The API module to control the settings of the simulator.
            See :class:`.SettingsApi` for details.
        system: SystemApi
            The API module for getting information about the host system.
            See :class:`.SystemApi` for details.
        traffic: TrafficApi
            The API module to control the traffic.
            See :class:`.TrafficApi` for details.
        vehicles: VehiclesApi
            The API module to control the vehicles in the scenario.
            See :class:`.VehiclesApi` for details.
    """

    def __init__(
        self,
        host: str,
        port: int,
        home: str | None = None,
        binary: str | None = None,
        user: str | None = None,
        quit_on_close: bool = True,
        debug: bool | None = None,
    ):
        self.logger = logging.getLogger(f"{LOGGER_ID}.BeamNGpy")
        self.logger.setLevel(logging.DEBUG)
        self.host = host
        self.port = port
        self.home = home
        self.binary = binary
        self.user = user
        self.process = None
        self.quit_on_close = quit_on_close
        self._debug = debug
        self.connection: Connection | None = None
        self._scenario: Scenario | None = None
        self._host_os: str | None = None
        self._tech_enabled: bool | None = None

        self._setup_api()

    def host_os(self) -> str | None:
        """
        The operating system of the host the simulator is running on.
        """
        return self._host_os

    def tech_enabled(self) -> bool | None:
        """
        A flag that specifies whether a BeamNG.tech features are enabled or not.
        """
        return self._tech_enabled

    def open(
        self,
        extensions: List[str] | None = None,
        *args: str,
        launch: bool = True,
        debug: bool | None = None,
        listen_ip: str = "127.0.0.1",
        **opts: str,
    ) -> BeamNGpy:
        """
        Starts a BeamNG.* process, opens a server socket, and waits for the spawned BeamNG.* process to connect.
        This method blocks until the process started and is ready.

        Args:
            extensions: A list of non-default BeamNG Lua extensions to be loaded on start.
            launch: Whether to launch a new process or connect to a running one on the configured host/port.
                    Defaults to True.
            debug: If True, then sets BeamNG.tech communication to debug mode. That means:

                    1. BeamNG will not respond to BeamNGpy requests when a Lua error
                    happens and prints the stacktrace instead.
                    2. The ``techCapture.*.log`` files are created automatically in the userfolder,
                    they log every protocol call and can be replayed using the ``tech/capturePlayer``
                    Lua extension.

                This option is applicable only when the process is launched by this instance
                of BeamNGpy, as it sets a launch argument of the process. Defaults to False.
            listen_ip: The IP address that the BeamNG process will be listening on. Only relevant when ``launch`` is True.
                     Set to ``*`` if you want BeamNG to listen on ALL network interfaces.
        """
        self.connection = Connection(self.host, self.port)

        # try to connect to existing instance
        connected = self.connection.connect_to_beamng(tries=1, log_tries=False)
        if connected:
            self.logger.info(
                "BeamNGpy successfully connected to existing BeamNG instance."
            )
            if extensions:
                cmd = ';'.join((f'extensions.load(\'{extension}\')' for extension in extensions))
                self.control.queue_lua_command(cmd)
        elif launch:
            self.logger.info("Opening BeamNGpy instance.")
            arg_list = list(args)

            if debug is None:
                debug = self._debug
            if debug == True:
                arg_list.append("-tcom-debug")
            elif debug == False:
                arg_list.append("-no-tcom-debug")
            arg_list.extend(("-tcom-listen-ip", listen_ip))

            self._start_beamng(extensions, *arg_list, **opts)
            sleep(10)
            self.connection.connect_to_beamng()
        self._load_system_info()
        return self

    def disconnect(self) -> None:
        """
        Disconnects from the BeamNG simulator.
        """
        if self._scenario:
            for vehicle in self._scenario.vehicles.values():
                try:
                    if vehicle.is_connected():
                        vehicle.disconnect()
                except Exception as e:
                    module_logger.debug(f"Cannot disconnect vehicle: {e}")
        if self.connection:
            self.connection.disconnect()
            self.connection = None

    def close(self) -> None:
        """
        Disconnects from the simulator and kills the BeamNG.* process.
        """
        self.logger.info("Closing BeamNGpy instance.")
        if not self.quit_on_close:
            self.disconnect()
            return
        if self._scenario:
            self._scenario.close()
            self._scenario = None
        self._kill_beamng()

    def _load_system_info(self) -> None:
        info = self.system.get_info()
        self._host_os = info["os"]["type"]
        self._tech_enabled = info["tech"]

    def _setup_api(self) -> None:
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

        self.ui = UiApi(self)
        self.display_gui_message = self.ui.display_message
        self.hide_hud = self.ui.hide_hud
        self.show_hud = self.ui.show_hud

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
        self.load_scenario = self.scenario.load
        self.teleport_scenario_object = self.scenario.teleport_object
        self.start_scenario = self.scenario.start
        self.restart_scenario = self.scenario.restart
        self.stop_scenario = self.scenario.stop
        self.get_roads = self.scenario.get_roads
        self.get_road_edges = self.scenario.get_road_edges
        self.load_trackbuilder_track = self.scenario.load_trackbuilder_track

        self.settings = SettingsApi(self)
        self.change_setting = self.settings.change
        self.apply_graphics_setting = self.settings.apply_graphics
        self.set_deterministic = self.settings.set_deterministic
        self.set_nondeterministic = self.settings.set_nondeterministic
        self.set_steps_per_second = self.settings.set_steps_per_second
        self.remove_step_limit = self.settings.remove_step_limit
        self.set_particles_enabled = self.settings.set_particles_enabled

        self.system = SystemApi(self)

        self.traffic = TrafficApi(self)
        self.spawn_traffic = self.traffic.spawn
        self.start_traffic = self.traffic.start
        self.reset_traffic = self.traffic.reset
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
        self.get_current_vehicles_info = self.vehicles.get_current_info
        self.get_current_vehicles = self.vehicles.get_current

    def _kill_beamng(self) -> None:
        """
        Kills the running BeamNG.* process.
        """
        self.logger.info("Terminating BeamNG.tech process.")
        if self.connection:
            try:
                self.control.quit_beamng()
                self.connection.disconnect()
                self.connection = None
            except (ConnectionResetError, ConnectionAbortedError, ConnectionRefusedError):
                self.connection = None
        if not self.process:
            self.logger.info(
                "cannot kill BeamNG.tech process not spawned by this instance of BeamNGpy, aborting subroutine"
            )
            return
        if self.process.stdin:
            self.process.stdin.close()
        if os.name == "nt":
            with open(os.devnull, "w") as devnull:
                subprocess.call(
                    ["taskkill", "/F", "/T", "/PID", str(self.process.pid)],
                    stdout=devnull,
                    stderr=devnull,
                )
                self.process.wait()
        else:
            try:
                os.kill(self.process.pid, signal.SIGTERM)
                self.process.wait()
            except:
                pass
        self.process = None

    def _send(self, data: StrDict) -> Response:
        if not self.connection:
            raise BNGError("Not connected to the simulator!")
        return self.connection.send(data)

    def _message(self, req: str, **kwargs: Any) -> Any:
        if not self.connection:
            raise BNGError("Not connected to the simulator!")
        return self.connection.message(req, **kwargs)

    def _prepare_call(
        self,
        binary: str,
        user: Path | None,
        extensions: List[str] | None,
        *args: str,
        **usr_opts: str,
    ) -> List[str]:
        """
        Prepares the command line call to execute to start BeamNG.*.
        according to this class' and the global configuration.

        Returns:
            List of shell components ready to be called in the
            :mod:`subprocess` module.
        """
        if extensions is None:
            extensions = []

        extensions.insert(0, "tech/techCore")
        lua = "extensions.load('{}');" * len(extensions)
        lua = lua.format(*extensions) + f"tech_techCore.openServer({self.port})"
        call = [binary, "-nosteam"]
        if platform.system() != "Linux":  # console is not supported for Linux hosts yet
            call.append("-console")

        for arg in args:
            call.append(arg)

        call_opts = {"lua": lua}
        if "lua" in usr_opts.keys():
            call_opts["lua"] = usr_opts["lua"]

        for key, val in call_opts.items():
            call.extend(["-" + key, val])

        if user:
            call.append("-userpath")
            call.append(str(user))
            if " " in str(user):
                msg = (
                    "Your configured userpath contains a space. "
                    "Unfortunately, this is known to cause issues in "
                    "launching BeamNG.tech. If you require a path with a "
                    "space in it, you can alternatively set it manually in"
                    'the file "startup.ini" contained in the directory of '
                    "your BeamNG.tech installtion. This would not be "
                    "automatically updated if you change the `user` "
                    "parameter to `BeamNGpy`, but serves as a workaround "
                    "until the issue is fixed in BeamNG.tech."
                )
                self.logger.error(msg)

        call_str = " ".join(call)
        self.logger.debug(
            "Created system call for starting " f"BeamNG process: `{call_str}`"
        )
        return call

    def _start_beamng(
        self, extensions: List[str] | None, *args: str, **opts: str
    ) -> None:
        """
        Spawns a BeamNG.* process and retains a reference to it for later
        termination.
        """
        home = filesystem.determine_home(self.home)
        if self.binary:
            binary = home / self.binary
            if not binary.is_file():
                raise BNGError(
                    f"The BeamNG binary {binary} was not found in BeamNG home."
                )
        else:
            binary = filesystem.determine_binary(home)
        userpath = (
            Path(self.user) if self.user else filesystem.determine_userpath(binary)
        )
        call = self._prepare_call(str(binary), userpath, extensions, *args, **opts)

        if platform.system() == "Linux":
            # keep the same behaviour as on Windows - do not print game logs to the Python stdout
            self.process = subprocess.Popen(
                call, stdout=subprocess.DEVNULL, stdin=subprocess.PIPE
            )
        else:
            self.process = subprocess.Popen(call, stdin=subprocess.PIPE)
        self.logger.info("Started BeamNG.")

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
