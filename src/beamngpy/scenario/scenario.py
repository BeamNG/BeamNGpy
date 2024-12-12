from __future__ import annotations

import copy
from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any, Callable, Dict, Iterable, List, Tuple

from jinja2 import Environment
from jinja2.loaders import PackageLoader

from beamngpy.logging import LOGGER_ID, BNGError, BNGValueError
from beamngpy.misc.colors import coerce_color
from beamngpy.misc.quat import quat_as_rotation_mat_str
from beamngpy.scenario.road import DecalRoad
from beamngpy.scenario.scenario_object import ScenarioObject, SceneObject
from beamngpy.types import Float3, Quat, StrDict
from beamngpy.utils.prefab import bool_to_str, get_uuid
from beamngpy.vehicle import Vehicle

from .level import Level

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.scenario.procedural import ProceduralMesh
    from beamngpy.scenario.road import MeshRoad, Road

TEMPLATE_ENV = Environment(loader=PackageLoader("beamngpy"))
TEMPLATE_ENV.filters["bool"] = bool_to_str

module_logger = getLogger(f"{LOGGER_ID}.scenario")
module_logger.setLevel(DEBUG)


def _list_to_str(list: Iterable[Any]) -> str:
    return "[" + ", ".join([str(p) for p in list]) + "]"


class Scenario:
    """
    The scenario class contains information for setting up and executing
    simulation scenarios along with methods to extract data during their
    execution.

    Instantiates a scenario instance with the given name taking place in
    the given level.

    Args:
        level: Either the name of the level this scenario takes place in
                as a string or as an instance of :class:`.Level`
        name: The name of this scenario. Should be unique for the
                    level it's taking place in to avoid file collisions.
        path: The path to an already existing scenario file (relative to
              the home folder / user folder). If set, then :func:`Scenario.make`
              should not be called, as the scenario is already made.
        human_name: The human-readable name of the scenario. If None, it
                    will be set to ``name``.
        description: The description of the scenario displayed in the simulator.
        difficulty: The difficulty of the scenario displayed in the simulator.
        authors: Names of the authors. Defaults to ``BeamNGpy``.
        restrict_actions: Whether to keep scenario restrictions, such as limited menu options and controls.
                          Defaults to False.
        options: Other options of the scenario object, not used at the moment.
    """

    scenetree_classes: Dict[str, Callable[[StrDict], SceneObject]] = {
        "MissionGroup": lambda d: SceneObject(d),
        "DecalRoad": lambda d: DecalRoad(d),
    }

    @staticmethod
    def from_dict(d: StrDict) -> Scenario:
        if "sourceFile" in d:
            path = d["sourceFile"]
            del d["sourceFile"]
        elif "scenarioName" in d:  # mission file
            path = "/gameplay/missions/" + d["scenarioName"] + "/info.json"
            del d["scenarioName"]
        else:
            path = None

        if "level" in d:
            level = Level.from_dict(d["level"])
            del d["level"]
        elif "levelName" in d:
            level = d["levelName"]
            del d["levelName"]
        else:
            level = "unknown"

        if "name" in d:
            name = d["name"]
            del d["name"]
        else:
            name = "unknown"

        scenario = Scenario(level, name, path, **d)

        return scenario

    @property
    def _uuid(self):
        return get_uuid(f"Scenario_{self.name}")

    def __init__(
        self,
        level: str | Level,
        name: str,
        path: str | None = None,
        human_name: str | None = None,
        description: str | None = None,
        difficulty: int = 0,
        authors: str = "BeamNGpy",
        restrict_actions: bool = False,
        **options: Any,
    ):
        self.level = level
        self.name = name
        self.path = path
        self.human_name = human_name if human_name is not None else self.name
        self.description = description
        self.difficulty = difficulty
        self.authors = authors
        self.restrict_actions = restrict_actions
        self.options = options

        self.vehicles: Dict[str, Vehicle] = {}
        self.transient_vehicles: Dict[str, Vehicle] = (
            {}
        )  # Vehicles added during scenario
        self._vehicle_locations: Dict[str, Tuple[Float3, Quat]] = {}
        self._focus_vehicle: str | None = None

        self.roads: List[Road] = list()
        self.mesh_roads: List[MeshRoad] = list()
        self.checkpoints: List[str] = list()
        self.proc_meshes: Dict[str, ProceduralMesh] = {}
        self.objects: List[ScenarioObject] = list()

        self.scene = None

        self.bng: BeamNGpy | None = None

        self.logger = getLogger(f"{LOGGER_ID}.Scenario")
        self.logger.setLevel(DEBUG)

    def __repr__(self):
        return f"<Scenario(level='{self.level}', name='{self.human_name}', path='{self.path}')>"

    def _get_objects_list(self) -> List[StrDict]:
        """
        Encodes extra objects to be placed in the scene as dictionaries for the
        prefab template.

        Returns:
            A list of dictionaries representing :class:`.ScenarioObject`
            instances to be placed in the prefab.
        """
        objs: List[StrDict] = list()
        for obj in self.objects:
            obj_dict: StrDict = dict(type=obj.type, id=obj.id)
            obj_dict["_uuid"] = obj._uuid
            obj_dict["options"] = copy.deepcopy(obj.opts)

            assert isinstance(obj.rot, tuple)
            for option in obj_dict["options"]:
                if isinstance(obj_dict["options"][option], str):
                    obj_dict["options"][option] = (
                        '"' + obj_dict["options"][option] + '"'
                    )
            obj_dict["options"]["position"] = _list_to_str(obj.pos)
            obj_dict["options"]["rotationMatrix"] = (
                "[" + quat_as_rotation_mat_str(obj.rot, ", ") + "]"
            )
            obj_dict["options"]["scale"] = _list_to_str(obj.scale)

            objs.append(obj_dict)
        self.logger.debug(
            f"The scenario {self.name} has {len(objs)} "
            "objects of type `beamngpy.ScenarioObject`"
        )
        return objs

    def _get_info_dict(self) -> StrDict:
        """
        Generates a dictionary of information to be written to the scenario's
        files in the simulation diretory and returns it.

        Returns:
            Dictionary of information to write into the scenario files of the
            simulator.
        """
        info: StrDict = dict(
            name=self.human_name,
            description=self.description,
            difficulty=self.difficulty,
            authors=self.authors,
            lapConfig=self.checkpoints,
            forceNoCountDown=True
        )

        vehicles_dict = dict()
        for vid in self.vehicles:
            vehicles_dict[vid] = {"playerUsable": True}

        if self.vehicles:
            if self._focus_vehicle is None:
                self._focus_vehicle = next(iter(self.vehicles))

            vehicles_dict[self._focus_vehicle]["startFocus"] = True

        info["vehicles"] = vehicles_dict
        info["prefabs"] = [f"levels/{self.level}/scenarios/{self.name}.prefab.json"]

        return info

    def _get_vehicles_list(self) -> List[StrDict]:
        """
        Gets the vehicles contained in this scenario encoded as a dict and
        put into one list, including their position and rotation as a matrix
        ready to be placed in the simulator.

        Returns:
            All vehicles as a dict including position and rotation.
        """
        vehicles: List[StrDict] = list()
        for vid, vehicle in self.vehicles.items():
            pos, rot = self._vehicle_locations[vid]
            vehicle_dict = dict(vid=vid)
            vehicle_dict.update(vehicle.options)
            vehicle_dict["position"] = _list_to_str(pos)
            vehicle_dict["rotationMatrix"] = (
                "[" + quat_as_rotation_mat_str(rot, ", ") + "]"
            )
            vehicle_dict["_uuid"] = vehicle._uuid
            if vehicle_dict["color"] is None:
                del vehicle_dict["color"]
            else:
                vehicle_dict["color"] = _list_to_str(
                    coerce_color(vehicle_dict["color"])
                )
            vehicles.append(vehicle_dict)
        vehicles = sorted(vehicles, key=lambda v: v["vid"])
        self.logger.debug(
            f"The scenario {self.name} has {len(vehicles)} "
            f'vehicles: {", ".join(self.vehicles.keys())}'
        )
        return vehicles

    def _get_roads_list(self) -> List[StrDict]:
        """
        Gets the roads defined in this scenario encoded as a dict and put into
        one list ready to be placed in the simulator.

        Returns:
            All roads encoded as a dict in one list.
        """
        ret: List[StrDict] = list()
        for idx, road in enumerate(self.roads):
            road_dict = dict(**road.__dict__)
            road_dict["road_id"] = (
                f"beamngpy_road_{self.name}_{idx:03}" if road.rid is None else road.rid
            )
            road.rid = road_dict["road_id"]
            road_dict["_uuid"] = road._uuid
            road_dict["render_priority"] = idx

            ret.append(road_dict)
        self.logger.debug(
            f"The scenario {self.name} has {len(ret)} " "scenario-specific roads."
        )
        return ret

    def _get_mesh_roads_list(self) -> List[StrDict]:
        """
        Gets the mesh roads defined in this scenario encoded as a dict and put into
        one list ready to be placed in the simulator.

        Returns:
            All mesh roads encoded as a dict in one list.
        """
        ret: List[StrDict] = list()
        for idx, road in enumerate(self.mesh_roads):
            road_dict = dict(**road.__dict__)
            road_dict["road_id"] = (
                f"beamngpy_mesh_road_{self.name}_{idx:03}"
                if road.rid is None
                else road.rid
            )
            road.rid = road_dict["road_id"]
            road_dict["_uuid"] = road._uuid
            road_dict["render_priority"] = idx

            ret.append(road_dict)
        self.logger.debug(
            f"The scenario {self.name} has {len(ret)} " "scenario-specific mesh roads."
        )
        return ret

    def _get_prefab(self) -> str:
        """
        Generates prefab code to describe this scenario to the simulation
        engine and returns it as a string.

        Returns:
            Prefab code for the simulator.
        """
        template = TEMPLATE_ENV.get_template("prefab.json")

        vehicles = self._get_vehicles_list()
        roads = self._get_roads_list()
        mesh_roads = self._get_mesh_roads_list()
        objs = self._get_objects_list()

        prefab = template.render(
            scenario=self,
            vehicles=vehicles,
            roads=roads,
            mesh_roads=mesh_roads,
            objects=objs,
        )
        prefab = prefab.replace("\n", "").replace("|---|", "\n")
        return prefab

    def _get_level_name(self) -> str:
        if isinstance(self.level, Level):
            return self.level.name
        else:
            return self.level

    def _load_existing_vehicles(self) -> None:
        assert self.bng

        current_vehicles = self.bng.vehicles.get_current(include_config=False)
        self.logger.debug(f"Got {len(current_vehicles)} vehicles from scenario.")
        self.transient_vehicles = current_vehicles.copy()

        for vid, vehicle in self.vehicles.items():
            self.transient_vehicles.pop(vid, None)
            current_vehicles[vid] = vehicle

        self.vehicles = current_vehicles

    def add_object(self, obj: ScenarioObject) -> None:
        """
        Adds an extra object to be placed in the prefab. Objects are expected
        to be :class:`.ScenarioObject` instances with additional, type-
        specific properties in that class's opts dictionary.
        """
        self.objects.append(obj)

    def add_vehicle(
        self,
        vehicle: Vehicle,
        pos: Float3 = (0, 0, 0),
        rot_quat: Quat = (0, 0, 0, 1),
        cling: bool = True,
    ) -> None:
        """
        Adds a :class:`.Vehicle`: to this scenario at the given position with the given
        orientation.

        Args:
            vehicle: The vehicle to spawn.
            pos: ``(x, y, z)`` tuple specifying the position of the vehicle.
            rot_quat: ``(x, y, z, w)`` tuple specifying the rotation as quaternion.
            cling: If True, the z-coordinate of the vehicle's position will be set to the ground level at the given
                   position to avoid spawning the vehicle below ground or in the air.
        """
        if self.name == vehicle.vid:
            error = (
                "Cannot have vehicle with the same name as the scenario:"
                f" Scenario={self.name}, Vehicle={vehicle.vid}"
            )
            raise BNGValueError(error)

        if vehicle.vid in self.vehicles:
            error = f"The vehicle '{vehicle.vid}' is already in the scenario."
            raise BNGValueError(error)

        if vehicle.connection:
            vehicle.disconnect()

        self.vehicles[vehicle.vid] = vehicle
        self._vehicle_locations[vehicle.vid] = (pos, rot_quat)
        self.logger.debug(f"Added vehicle with id '{vehicle.vid}'.")

        if self.bng:
            self.bng.vehicles.spawn(vehicle, pos, rot_quat=rot_quat, cling=cling)
            self.transient_vehicles[vehicle.vid] = vehicle
            vehicle.connect(self.bng)
        else:
            self.logger.debug(
                "No BeamNGpy instance available. "
                f"Did not spawn vehicle with id '{vehicle.vid}'."
            )

    def remove_vehicle(self, vehicle: Vehicle) -> None:
        """
        Removes the given :class:`.Vehicle`: from this scenario. If the
        scenario is currently loaded, the vehicle will be despawned.

        Args:
            vehicle: The vehicle to remove.
        """
        if vehicle.vid in self.vehicles:
            if self.bng:
                self.bng.vehicles.despawn(vehicle)
                self.transient_vehicles.pop(vehicle.vid, None)
            else:
                self.logger.debug(
                    "No beamngpy instance available, cannot "
                    f"despawn vehicle with id '{vehicle.vid}'"
                )

            if vehicle.vid in self._vehicle_locations:
                del self._vehicle_locations[vehicle.vid]
            del self.vehicles[vehicle.vid]
        else:
            self.logger.debug(f"No vehicle with id {vehicle.vid} found.")

    def get_vehicle(self, vehicle_id: str) -> Vehicle | None:
        """
        Retrieves the vehicle with the given ID from this scenario.

        Args:
            vehicle_id: The ID of the vehicle to find.

        Returns:
            The :class:`.Vehicle` with the given ID. ``None`` if it wasn't found.
        """
        if vehicle_id in self.vehicles:
            return self.vehicles[vehicle_id]
        self.logger.debug(f"Could not find vehicle with id {vehicle_id}")
        return None

    def set_initial_focus(self, vehicle_id: str) -> None:
        """
        Defines which vehicle has the initial focus.

        Args:
            vehicle_id: Vehicle id of focused vehicle
        """
        self._focus_vehicle = vehicle_id

    def add_road(self, road: Road) -> None:
        """
        Adds a :class:`.Road` to this scenario.

        Args:
            road: Road to be added to the scenario.
        """
        self.roads.append(road)

    def add_mesh_road(self, road: MeshRoad) -> None:
        """
        Adds a :class:`.MeshRoad` to this scenario.

        Args:
            road: Mesh road to be added to the scenario.
        """
        self.mesh_roads.append(road)

    def add_procedural_mesh(self, mesh: ProceduralMesh) -> None:
        """
        Adds a :class:`.ProceduralMesh` to be placed in world to the scenario.

        Args:
            mesh: The mesh to place.
        """
        self.proc_meshes[mesh.name] = mesh
        if self.bng:
            mesh.place(self.bng)

    def remove_procedural_mesh(self, mesh: ProceduralMesh) -> None:
        """
        Removes a :class:`.ProceduralMesh` that was placed in the world.

        Args:
            mesh: The mesh to remove.

        Raises:
            BNGError: If the mesh to remove was not found.
        """
        deleted = False
        if mesh.name in self.proc_meshes:
            del self.proc_meshes[mesh.name]
            deleted = True
        if self.bng:
            mesh.remove(self.bng)
            deleted = True
        if not deleted:
            raise BNGError(f"The mesh '{mesh.name}' was not found in the scenario.")

    def add_checkpoints(
        self,
        positions: List[Float3],
        scales: List[Float3],
        ids: List[str] | None = None,
    ) -> None:
        """
        Adds checkpoints to the scenario.

        Args:
            positions: Positions (tuple of length 3) of the individual points.
            scales: Scales (tuple of length 3) of the individual points
            ids: Optional, names of the individual points.
        """
        if ids is None:
            ids = [f"wp{i}" for i in range(len(positions))]
        assert len(positions) == len(scales) == len(ids)
        options = dict(
            rot_quat=(0, 0, 0, 1),
            drawDebug="0",
            directionalWaypoint="0",
            mode="Ignore",
            canSave="1",
            canSaveDynamicFields="1",
        )
        for oid, p, s in zip(ids, positions, scales):
            cp = ScenarioObject(
                oid=oid, name=oid, otype="BeamNGWaypoint", pos=p, scale=s, **options
            )
            self.add_object(cp)
        self.checkpoints.extend(ids)

    def _convert_scene_object(self, obj: StrDict) -> SceneObject:
        assert self.bng
        data = self.bng._message("GetObject", id=obj["id"])
        clazz = data["class"]
        if clazz in Scenario.scenetree_classes:
            converted = Scenario.scenetree_classes[clazz](data)
        else:
            converted = SceneObject(data)

        if "children" in obj:
            for child in obj["children"]:
                child = self._convert_scene_object(child)
                converted.children.append(child)

        return converted

    def sync_scene(self) -> None:
        """
        Retrieves the current scene tree of the scenario from the simulator,
        converting them into the most appropriate known (sub)class of
        :class:`.SceneObject`. The result is not returned but rather stored
        in the ``scene`` field of this class.
        """
        assert self.bng
        scenetree = self.bng._message("GetSceneTree")
        assert scenetree["class"] == "SimGroup"
        self.scene = self._convert_scene_object(scenetree)

    def connect(
        self, bng: BeamNGpy, connect_player: bool = True, connect_existing: bool = True
    ) -> None:
        """
        Connects this scenario to the simulator.

        Args:
            bng: The BeamNGpy instance to generate the scenario for.
            connect_player: Whether the player vehicle should be connected
                            to this (:class:``.Scenario``) instance. Defaults to True.
            connect_existing: Whether ALL vehicles spawned already in the scenario should be connected
                              to this (:class:``.Scenario``) instance. Defaults to True.
        """
        self.bng = bng

        self.logger.debug(f"{len(self.proc_meshes)} procedural meshes.")
        for mesh in self.proc_meshes.values():
            mesh.place(self.bng)

        if connect_existing or connect_player:
            self._load_existing_vehicles()
        try:
            player_vid = bng.vehicles.get_player_vehicle_id()["vid"]
        except BNGError:
            player_vid = None

        self.logger.debug(f"Connecting to {len(self.vehicles)} vehicles.")
        for vehicle in self.vehicles.values():
            if connect_existing or (connect_player and vehicle.vid == player_vid):
                vehicle.connect(bng)

        self.logger.info(f"Connected to scenario: {self.name}")

    def make(self, bng: BeamNGpy) -> None:
        """
        Generates necessary files to describe the scenario in the simulation
        and outputs them to the simulator.

        Args:
            bng: The BeamNGpy instance to generate the scenario for.

        Raises:
            BNGError: If the scenario already has set its info .json file included.
        """
        if self.path is not None:
            raise BNGError("This scenario already has an info file.")

        level_name = self._get_level_name()

        prefab = self._get_prefab()
        info = self._get_info_dict()
        self.logger.debug(f"Generated prefab:\n{prefab}\n")
        self.logger.debug(f"Generated scenarios info dict:\n{info}\n")

        self.path = bng._message(
            "CreateScenario",
            level=level_name,
            name=self.name,
            prefab=prefab,
            info=info,
            json=True,
        )

    def find(self, bng: BeamNGpy) -> str | None:
        """
        Looks for the files of an existing scenario and returns the path to the
        info file of this scenario, iff one is found.

        Args:
            bng: The BeamNGpy instance to look for the scenario in.

        Returns:
            The path to the information file of his scenario found in the
            simulator as a string. None if it could not be found.
        """
        scenarios = bng.scenario.get_level_scenarios(self.level)
        for scenario in scenarios:
            if scenario.name == self.name and scenario.level == self.level:
                self.path = scenario.path
                return self.path
        return None

    def delete(self, bng: BeamNGpy) -> None:
        """
        Deletes files created by this scenario from the given
        :class:`.BeamNGpy`'s home/user path.
        """
        if self.path is None:
            self.find(bng)
        bng._message("DeleteScenario", path=self.path)
        self.logger.info(f'Deleted scenario from simulation: "{self.name}".')

    def restart(self) -> None:
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
            raise BNGError(
                "Scenario needs to be loaded into a BeamNGpy "
                "instance to be restarted."
            )

        while self.transient_vehicles:
            vid, vehicle = self.transient_vehicles.popitem()
            if vid in self.vehicles:
                self.bng.vehicles.despawn(vehicle)
                del self.vehicles[vid]
        self.logger.info(f'Restarted scenario: "{self.name}"')

    def close(self) -> None:
        """
        Closes open connections and allocations of the scenario.

        Raises:
            BNGError: If the scenario has not been loaded.
        """
        if not self.bng:
            raise BNGError(
                "Scenario needs to be loaded into a BeamNGpy " "instance to be stopped."
            )

        for vehicle in self.vehicles.values():
            vehicle.close()

        self.bng = None
        self.logger.debug("Removed beamngpy instance from scenario class.")

    def _find_objects_class(self, clazz: str) -> List[ScenarioObject]:
        if not self.bng:
            raise BNGError(
                "Scenario needs to be loaded into a BeamNGpy "
                "instance to find objects."
            )

        return self.bng.scenario.find_objects_class(clazz)

    def find_waypoints(self) -> List[ScenarioObject]:
        """
        Finds waypoints placed in the world right now.

        Returns:
            A list of :class:`.ScenarioObject` containing waypoints found in
            the world.

        Raises:
            BNGError: If the scenario is not currently loaded.
        """
        waypoints = self._find_objects_class("BeamNGWaypoint")
        return waypoints

    def find_procedural_meshes(self) -> List[ScenarioObject]:
        """
        Finds procedural meshes placed in the world right now.

        Returns:
            A list of :class:`.ScenarioObject` containing procedural meshes
            found in the world.

        Raises:
            BNGError: If the scenario is not currently loaded.
        """
        meshes = self._find_objects_class("ProceduralMesh")
        return meshes

    def find_static_objects(self) -> List[ScenarioObject]:
        """
        Finds static objects placed in the world right now.

        Returns:
            A list of :class:`.ScenarioObject` containing statically placed
            objects found in the world.

        Raises:
            BNGError: If the scenario is not currently loaded.
        """
        return self._find_objects_class("TSStatic")

    def update(self) -> None:
        """
        Synchronizes object states of this scenario with the simulator.
        This is used to update the :attr:`.Vehicle.state` fields of
        each vehicle in the scenario.

        Raises:
            BNGError: If the scenario is currently not loaded.
        """
        if not self.bng:
            raise BNGError(
                "Scenario needs to be loaded into a BeamNGpy "
                "instance to update its state."
            )

        for vehicle in self.vehicles.values():
            vehicle.sensors.poll("state")
