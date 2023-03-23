from __future__ import annotations

from typing import TYPE_CHECKING, Dict, List, Tuple

from beamngpy.logging import BNGError, BNGValueError
from beamngpy.scenario import Scenario, ScenarioObject
from beamngpy.scenario.level import Level
from beamngpy.types import Float3, Quat, StrDict

from .base import Api

if TYPE_CHECKING:
    from beamngpy.vehicle import Vehicle


class ScenarioApi(Api):
    """
    An API gathering function for working with scenarios, levels and scenario objects.

    Args:
        beamng: An instance of the simulator.
    """

    def get_levels(self) -> Dict[str, Level]:
        """
        Queries the available levels in the simulator and returns them as a
        mapping of level name to :class:`.Level` instances.

        Returns:
            A dictionary of available level names to a corresponding instance
            of the :class:`.Level` class.
        """
        levels = self._message('GetLevels')
        levels = [Level.from_dict(l) for l in levels]
        levels = {l.name: l for l in levels}
        return levels

    def get_scenarios(self, levels: Dict[str, Level] | None = None) -> Dict[str, Scenario]:
        """
        Queries the available scenarios and returns them as a mapping of
        paths to :class:`.Scenario` instances. The scenarios are constructed
        to point to their parent levels, so to avoid extra queries to the
        simulator about existing levels, a cache of available levels can be
        passed to this method.

        Args:
            levels: A dictionary of level names to :class:`.Level`
                    instances to fill in the parent level of returned
                    scenarios.

        Returns:
            A mapping of scenario paths to their corresponding
            :class:`.Scenario` instance.
        """
        if levels is None:
            levels = self.get_levels()

        scenarios = self._message('GetScenarios')
        scenarios = [Scenario.from_dict(s) for s in scenarios]
        scenarios = {str(s.path): s for s in scenarios if s.level in levels.keys()}
        for _, scenario in scenarios.items():
            assert isinstance(scenario.level, str)
            scenario.level = levels[scenario.level]

        return scenarios

    def get_level_scenarios(self, level: str | Level) -> Dict[str, Scenario]:
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

        scenarios = self._message('GetScenarios')
        scenarios = [Scenario.from_dict(s) for s in scenarios]
        scenarios = {str(s.path): s for s in scenarios if s.level == level_name}

        for scenario in scenarios.values():
            scenario.level = level

        return scenarios

    def get_levels_and_scenarios(self) -> Tuple[Dict[str, Level], Dict[str, Scenario]]:
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

    def get_current(self, levels: Dict[str, Level] | None = None) -> Scenario:
        """
        Queries the currently loaded scenario from the simulator.

        Args:
            levels: A mapping of level names to :class:`.Level` instances

        Returns:
            A :class:`.Scenario` instance of the currently-loaded scenario. If
            the ``levels`` parameter contains the level the scenario is taking
            place in, the scenario's parent level field will be filled in
            accordingly.
        """
        scenario = self._message('GetCurrentScenario')
        if not scenario:
            raise BNGValueError('The current scenario could not be retrieved.')
        scenario = Scenario.from_dict(scenario)

        if levels is not None:
            if scenario.level in levels:
                scenario.level = levels[scenario.level]

        return scenario

    def get_name(self) -> str:
        """
        Retrieves the name of the currently-loaded scenario in the simulator.

        Returns:
            The name of the loaded scenario as a string.
        """
        data = dict(type='GetScenarioName')
        resp = self._send(data).recv('ScenarioName')
        return resp['name']

    def load(self, scenario: Scenario, precompile_shaders: bool = True) -> None:
        """
        Loads the given scenario in the simulation and returns once loading
        is finished.

        Args:
            scenario: The scenario to load.
            precompile_shaders: Whether the shaders should be compiled before the start of the scenario.
                                If False, the first load of a map will take a longer time, but disabling
                                the precompilation can lead to issues with the :class:`Camera` sensor.
                                Defaults to True.
        """
        # clean up the vehicle connections if the `scenario` object is reused multiple times
        for vehicle in scenario.vehicles.values():
            if vehicle.connection:
                vehicle.disconnect()

        data = {'type': 'LoadScenario', 'path': scenario.path, 'precompileShaders': precompile_shaders}
        self._send(data).ack('MapLoaded')
        self._logger.info('Loaded map.')
        self._beamng._scenario = scenario
        self._beamng._scenario.connect(self._beamng)

    def teleport_object(self, scenario_object: ScenarioObject, pos: Float3, rot_quat: Quat | None = None) -> None:
        """
        Teleports the given scenario object to the given position with the
        given rotation.

        Args:
            scenario_object: The vehicle to teleport.
            pos: The target position as an (x,y,z) tuple containing world-space coordinates.
            rot_quat: Optional tuple specifying object rotation as a quaternion.
        """
        data: StrDict = dict(type='TeleportScenarioObject')
        data['id'] = scenario_object.id
        data['pos'] = pos
        if rot_quat:
            data['rot'] = rot_quat
        self._send(data).ack('ScenarioObjectTeleported')

    def start(self, restrict_actions: bool = False) -> None:
        """
        Starts the scenario; equivalent to clicking the "Start" button in the
        game after loading a scenario. This method blocks until the countdown
        to the scenario's start has finished.

        Args:
            restrict_actions: Whether to keep scenario restrictions,
                            such as limited menu options and controls.
                            Defaults to False.
        """
        data: StrDict = dict(type='StartScenario')
        data['restrict_actions'] = restrict_actions
        self._send(data).ack('ScenarioStarted')
        self._logger.info('Starting scenario.')

    def restart(self) -> None:
        """
        Restarts a running scenario.
        """
        if not self._beamng._scenario:
            raise BNGError('Need to have a scenario loaded to restart it.')

        vehicles_to_reconnect = [v.vid for v in self._beamng._scenario.vehicles.values() if v.is_connected()]
        self._beamng._scenario.restart()

        self._logger.info('Restarting scenario.')
        data = dict(type='RestartScenario')
        self._send(data).ack('ScenarioRestarted')

        self._beamng._scenario._load_existing_vehicles()
        for vehicle in self._beamng._scenario.vehicles.values():
            if vehicle.vid in vehicles_to_reconnect and not vehicle.is_connected():
                vehicle.connect(self._beamng)

    def stop(self) -> None:
        """
        Stops a running scenario and returns to the main menu.
        """
        if not self._beamng._scenario:
            raise BNGError('Need to have a scenario loaded to stop it.')

        self._beamng._scenario.close()
        self._beamng._scenario = None

        data = dict(type='StopScenario')
        self._send(data).ack('ScenarioStopped')
        self._logger.info('Stopping scenario.')

    def get_roads(self) -> StrDict:
        """
        Retrieves the vertex data of all DecalRoads in the current scenario.
        The vertex data of a DecalRoad is formatted as point triples, where
        each triplet represents the left, centre, and right points of the edges
        that make up a DecalRoad.

        Returns:
            A dict mapping DecalRoad IDs to lists of point triples.
        """
        if not self._beamng._scenario:
            raise BNGError('Need to be in a started scenario to get its '
                           'DecalRoad data.')

        data = dict(type='GetDecalRoadData')
        resp = self._send(data).recv('DecalRoadData')
        return resp['data']

    def get_road_edges(self, road: str) -> List[Dict[str, Float3]]:
        """
        Retrieves the edges of the road with the given name and returns them
        as a list of point triplets. Roads are defined by a series of lines
        that specify the leftmost, center, and rightmost point in the road.
        These lines go horizontally across the road and the series of leftmost
        points make up the left edge of the road, the series of rightmost
        points make up the right edge of the road, and the series of center
        points the middle line of the road.

        Args:
            road: Name of the road to get edges from.

        Returns:
            The road edges as a list of (left, center, right) point triplets.
            Each point is an (X, Y, Z) coordinate triplet.
        """
        data = dict(type='GetDecalRoadEdges')
        data['road'] = road
        resp = self._send(data).recv('DecalRoadEdges')
        return resp['edges']

    def load_trackbuilder_track(self, path: str):
        """
        Spawns a TrackBuilder track provided by the given path to a TrackBuilder
        ``.json`` file.

        Args:
            path: Path to a ``.json`` file created by TrackBuilder.
        """
        data = dict(type='LoadTrackBuilderTrack')
        data['path'] = path
        return self._send(data).ack('TrackBuilderTrackLoaded')

    def find_objects_class(self, clazz: str) -> List[ScenarioObject]:
        """
        Scans the current environment in the simulator for objects of a
        certain class and returns them as a list of :class:`.ScenarioObject`.

        What kind of classes correspond to what kind of objects is described
        in the BeamNG.drive documentation.

        Args:
            clazz: The class name of objects to find.

        Returns:
            Found objects as a list.
        """
        data = dict(type='FindObjectsClass')
        data['class'] = clazz
        resp = self._send(data).recv()
        ret: List[ScenarioObject] = list()
        for obj in resp['objects']:
            sobj = ScenarioObject(obj['id'], obj['name'], obj['type'],
                                  tuple(obj['position']),
                                  tuple(obj['scale']),
                                  rot_quat=tuple(obj['rotation']),
                                  **obj['options'])
            ret.append(sobj)
        return ret

    def get_vehicle(self, vehicle_id: str) -> Vehicle | None:
        """
        Retrieves the vehicle with the given ID from the currently loaded scenario.

        Args:
            vehicle_id: The ID of the vehicle to find.

        Returns:
            The :class:`.Vehicle` with the given ID. None if it wasn't found.
        """
        scenario = self._beamng._scenario
        if not scenario:
            scenario = self.get_current()
            scenario._load_existing_vehicles()

        return scenario.get_vehicle(vehicle_id)
