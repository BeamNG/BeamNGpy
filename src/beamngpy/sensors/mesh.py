from __future__ import annotations

import math
from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib import collections as mc

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID
from beamngpy.types import StrDict

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

__all__ = ["Mesh"]


class Mesh(CommBase):
    """
    An automated 'sensor' to retrieve mesh data in real time.

    Args:
        name: A unique name for this mesh sensor.
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this sensor should be attached. Note: a vehicle must be provided for the mesh sensor.
        gfx_update_time: The gfx-step time which should pass between sensor reading updates to the user, in seconds.
        groups_list: A list of mesh groups which are to be considered. Optional.  If empty, we include all mesh nodes/beams.
        is_track_beams: A flag which indicates if we should keep updating the beam to node maps. This will track broken beams over time, but is slower.
    """

    def __init__(
        self,
        name: str,
        bng: BeamNGpy,
        vehicle: Vehicle,
        gfx_update_time: float = 0.0,
        physics_update_time: float = 0.015,
        groups_list=[],
        is_track_beams=True,
    ):
        super().__init__(bng, vehicle)

        self.logger = getLogger(f"{LOGGER_ID}.Mesh")
        self.logger.setLevel(DEBUG)

        # BeamNG properties.
        self.name = name
        self.vehicle = vehicle
        self.vid = vehicle.vid

        self.is_track_beams = is_track_beams

        # Mesh group properties (which parts of mesh to include, if used).
        # We populate a Boolean dictionary of all groups to be included, for fast lookup.
        self.groups = {}
        for g in groups_list:
            self.groups[g] = True
        self.is_full_mesh = len(groups_list) == 0

        # Create and initialise this sensor in the simulation.
        self._open_mesh(name, vehicle, gfx_update_time, physics_update_time)

        # Fetch the unique Id number (in the simulator) for this mesh sensor. We will need this later.
        self.sensorId = self._get_mesh_id()

        # Node/Beam relevance mapping properties. These maps are computed once for efficiency in all later iterations of polling this sensor.
        self.is_node_relevance_map_computed = False
        self.node_relevance_map = {}
        self.is_beam_relevance_map_computed = False
        self.beam_relevance_map = {}

        # Mesh related properties.
        self.raw_data = {}
        self.node_positions = {}

        # Populate the list of beams for this sensor.
        self.beams = self._get_active_beams()

        self.logger.debug("Mesh - sensor created: " f"{self.name}")

    def _is_node_relevant(self, v):
        """
        Determines whether a given node is in one of the selected mesh groups, or not.

        Args:
            v (dict): A node instance (from the simulator).

        Returns:
            True if the node is in one of the selected mesh groups, otherwise false
        """
        if "partOrigin" not in v:
            return False

        return v["partOrigin"] in self.groups

    def _is_beam_relevant(self, b0, b1):
        """
        Determines whether a given beam is connected to one of the selected mesh groups, or not.
        If the node at only one end of the beam is within a mesh group, we do not include this as being relevant.

        Args:
            b0 (int): The index of the beams first node (in the node list).
            b1 (int): The index of the beams second node (in the node list).

        Returns:
            True if the beam is in one of the selected mesh groups, otherwise false
        """
        return self._is_node_relevant(
            self.raw_data["nodes"][b0]
        ) and self._is_node_relevant(self.raw_data["nodes"][b1])

    def _get_active_beams(self):
        """
        Gets the latest collection beams and updates the class state with them.
        """

        # If we are not tracking beams, then we only need to compute the beam structures once, so leave immediately if it has been done in that case.
        if self.is_track_beams == False and self.is_beam_relevance_map_computed == True:
            return

        beam_data = self.send_recv_ge("GetBeamData", vid=self.vid)["data"]
        self.beams = {}

        if self.is_beam_relevance_map_computed == True:
            for key, value in beam_data.items():
                # We do not include any irrelevent beams or broken beams.
                if key not in self.beam_relevance_map or int(value[3]) == 5:
                    continue
                b0, b1, b2 = int(value[0]), int(value[1]), int(value[2])
                self.beams[int(key)] = [b0, b1, b2]
            return

        if "nodes" not in self.raw_data:
            return

        # We have not yet computed the beam relevance map, so compute it now. We only need to do this once.
        self.beam_relevance_map = {}
        for key, value in beam_data.items():
            # We do not include any broken beams.
            if int(value[3]) == 5:
                continue
            b0, b1, b2 = int(value[0]), int(value[1]), int(value[2])
            # We do not include beams which are not requested.
            if self.is_full_mesh == True or self._is_beam_relevant(b0, b1) == True:
                self.beams[int(key)] = [b0, b1, b2]
                self.beam_relevance_map[key] = True
        self.is_beam_relevance_map_computed = True

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        self._close_mesh()
        self.logger.debug("Mesh - sensor removed: " f"{self.name}")

    def poll(self) -> StrDict:
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary containing the sensor readings data.
        """

        # Send and receive a request for readings data from this sensor.
        self.raw_data = self._poll_mesh_VE()

        # Convert dict indices to int.
        self.node_positions = {}

        if self.is_node_relevance_map_computed == True:
            for k in self.node_relevance_map.keys():
                self.node_positions[k] = self.raw_data["nodes"][k]
        else:
            # The node relevance map has not been computed, so compute it now.  This only needs to be computed once.
            self.node_relevance_map = {}
            if self.is_full_mesh == True:
                for k, v in self.raw_data["nodes"].items():
                    key = int(k)
                    self.node_positions[key] = v
                    self.node_relevance_map[key] = True
            else:
                for k, v in self.raw_data["nodes"].items():
                    # Only include beams which are requested.
                    if self._is_node_relevant(v) == True:
                        key = int(k)
                        self.node_positions[key] = v
                        self.node_relevance_map[key] = True
            self.is_node_relevance_map_computed = True

        # Get the latest beams from the simulator.
        self._get_active_beams()

        self.logger.debug(
            "Mesh - sensor readings received from simulation: " f"{self.name}"
        )
        return self.raw_data

    def send_ad_hoc_poll_request(self) -> int:
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            A unique Id number for the ad-hoc request.
        """
        self.logger.debug("Mesh - ad-hoc polling request sent: " f"{self.name}")
        return int(
            self.send_recv_ge(
                "SendAdHocRequestMesh", name=self.name, vid=self.vehicle.vid
            )["data"]
        )

    def is_ad_hoc_poll_request_ready(self, request_id: int) -> bool:
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug(
            "Mesh - ad-hoc polling request checked for completion: " f"{self.name}"
        )
        return self.send_recv_ge("IsAdHocPollRequestReadyMesh", requestId=request_id)[
            "data"
        ]

    def collect_ad_hoc_poll_request(self, request_id: int) -> StrDict:
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            The readings data.
        """
        readings = self.send_recv_ge(
            "CollectAdHocPollRequestMesh", requestId=request_id
        )["data"]
        self.logger.debug(
            "Mesh - ad-hoc polling request returned and processed: " f"{self.name}"
        )
        return readings

    def set_requested_update_time(self, requested_update_time: float) -> None:
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time: The new requested update time.
        """
        self.send_ack_ge(
            "SetMeshRequestedUpdateTime",
            ack="CompletedSetMeshRequestedUpdateTime",
            name=self.name,
            vid=self.vehicle.vid,
            GFXUpdateTime=requested_update_time,
        )

    def _get_mesh_id(self) -> int:
        return int(self.send_recv_ge("GetMeshId", name=self.name)["data"])

    def _open_mesh(
        self,
        name: str,
        vehicle: Vehicle,
        gfx_update_time: float,
        physics_update_time: float,
    ) -> None:
        self.send_ack_ge(
            type="OpenMesh",
            ack="OpenedMesh",
            name=name,
            vid=vehicle.vid,
            GFXUpdateTime=gfx_update_time,
            physicsUpdateTime=physics_update_time,
        )
        self.logger.info(f'Opened Mesh sensor: "{name}"')

    def _close_mesh(self) -> None:
        self.send_ack_ge(
            type="CloseMesh", ack="ClosedMesh", name=self.name, vid=self.vehicle.vid
        )
        self.logger.info(f'Closed Mesh sensor: "{self.name}"')

    def _poll_mesh_VE(self) -> StrDict:
        return self.send_recv_veh("PollMeshVE", name=self.name, sensorId=self.sensorId)[
            "data"
        ]

    def get_node_positions(self):
        return self.node_positions

    def compute_beam_line_segments(self):
        lines1 = []
        lines2 = []
        lines3 = []
        c = []
        for _, v in self.beams.items():
            p1 = self.node_positions[v[0]]["pos"]
            p2 = self.node_positions[v[1]]["pos"]
            lines1.append([(p1["x"], p1["y"]), (p2["x"], p2["y"])])
            lines2.append([(p1["x"], p1["z"]), (p2["x"], p2["z"])])
            lines3.append([(p1["y"], p1["z"]), (p2["y"], p2["z"])])
            c.append((0.3, 0.3, 0.3, 0.1))
        lns1 = mc.LineCollection(lines1, colors=c, linewidths=0.5)
        lns2 = mc.LineCollection(lines2, colors=c, linewidths=0.5)
        lns3 = mc.LineCollection(lines3, colors=c, linewidths=0.5)
        return lns1, lns2, lns3

    def mesh_plot(self):
        sns.set_theme()  # Let seaborn apply better styling to all matplotlib graphs
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect("equal", adjustable="box")
        ax[1, 0].set_aspect("equal", adjustable="box")
        ax[1, 1].set_aspect("equal", adjustable="box")
        ax[0, 0].set_xlim([-3, 3])
        ax[1, 0].set_xlim([-3, 3])
        ax[1, 1].set_xlim([-3, 3])
        ax[0, 0].set_ylim([-3, 3])
        ax[1, 0].set_ylim([-3, 3])
        ax[1, 1].set_ylim([-3, 3])
        ax[0, 0].set_title("Plan")
        ax[0, 0].set_xlabel("x")
        ax[0, 0].set_ylabel("y")
        ax[1, 0].set_title("Front Elevation")
        ax[1, 0].set_xlabel("x")
        ax[1, 0].set_ylabel("z")
        ax[1, 1].set_title("End Elevation")
        ax[1, 1].set_xlabel("y")
        ax[1, 1].set_ylabel("z")
        ax[0, 1].axis("off")
        for i in range(len(self.node_positions)):
            node = self.node_positions[i]["pos"]
            ax[0, 0].plot(node["x"], node["y"], "ro")
            ax[1, 0].plot(node["x"], node["z"], "ro")
            ax[1, 1].plot(node["y"], node["z"], "ro")

        lns1, lns2, lns3 = self.compute_beam_line_segments()
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()

    def mass_distribution_plot(self, data):
        sns.set_theme()  # Let seaborn apply better styling to all matplotlib graphs
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect("equal", adjustable="box")
        ax[1, 0].set_aspect("equal", adjustable="box")
        ax[1, 1].set_aspect("equal", adjustable="box")
        ax[0, 0].set_xlim([-3, 3])
        ax[1, 0].set_xlim([-3, 3])
        ax[1, 1].set_xlim([-3, 3])
        ax[0, 0].set_ylim([-3, 3])
        ax[1, 0].set_ylim([-3, 3])
        ax[1, 1].set_ylim([-3, 3])
        ax[0, 0].set_title("Plan")
        ax[0, 0].set_xlabel("x")
        ax[0, 0].set_ylabel("y")
        ax[1, 0].set_title("Front Elevation")
        ax[1, 0].set_xlabel("x")
        ax[1, 0].set_ylabel("z")
        ax[1, 1].set_title("End Elevation")
        ax[1, 1].set_xlabel("y")
        ax[1, 1].set_ylabel("z")
        ax[0, 1].axis("off")

        x = []
        y = []
        z = []
        colors = []
        circle_size = 3.0
        for i in range(len(data)):
            node = data[i]["pos"]
            x.append(node["x"])
            y.append(node["y"])
            z.append(node["z"])
            colors.append(data[i]["mass"])

        cmap = matplotlib.cm.viridis
        s1 = ax[0, 0].scatter(x, y, s=circle_size, c=colors, cmap=cmap)
        s2 = ax[1, 0].scatter(x, z, s=circle_size, c=colors, cmap=cmap)
        s3 = ax[1, 1].scatter(y, z, s=circle_size, c=colors, cmap=cmap)
        fig.colorbar(s1)

        lns1, lns2, lns3 = self.compute_beam_line_segments()
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()

    def force_distribution_plot(self, data):
        sns.set_theme()  # Let seaborn apply better styling to all matplotlib graphs
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect("equal", adjustable="box")
        ax[1, 0].set_aspect("equal", adjustable="box")
        ax[1, 1].set_aspect("equal", adjustable="box")
        ax[0, 0].set_xlim([-3, 3])
        ax[1, 0].set_xlim([-3, 3])
        ax[1, 1].set_xlim([-3, 3])
        ax[0, 0].set_ylim([-3, 3])
        ax[1, 0].set_ylim([-3, 3])
        ax[1, 1].set_ylim([-3, 3])
        ax[0, 0].set_title("Plan")
        ax[0, 0].set_xlabel("x")
        ax[0, 0].set_ylabel("y")
        ax[1, 0].set_title("Front Elevation")
        ax[1, 0].set_xlabel("x")
        ax[1, 0].set_ylabel("z")
        ax[1, 1].set_title("End Elevation")
        ax[1, 1].set_xlabel("y")
        ax[1, 1].set_ylabel("z")
        ax[0, 1].axis("off")

        x = []
        y = []
        z = []
        colors = []
        circle_size = 3.0
        for i in range(len(data)):
            node = data[i]["pos"]
            x.append(node["x"])
            y.append(node["y"])
            z.append(node["z"])
            force = data[i]["force"]
            fx = force["x"]
            fy = force["y"]
            fz = force["z"]
            colors.append(math.sqrt(fx * fx + fy * fy + fz * fz))

        cmap = matplotlib.cm.viridis
        s1 = ax[0, 0].scatter(x, y, s=circle_size, c=colors, cmap=cmap)
        s2 = ax[1, 0].scatter(x, z, s=circle_size, c=colors, cmap=cmap)
        s3 = ax[1, 1].scatter(y, z, s=circle_size, c=colors, cmap=cmap)
        fig.colorbar(s1)

        lns1, lns2, lns3 = self.compute_beam_line_segments()
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()

    def force_direction_plot(self, data):
        sns.set_theme()  # Let seaborn apply better styling to all matplotlib graphs
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect("equal", adjustable="box")
        ax[1, 0].set_aspect("equal", adjustable="box")
        ax[1, 1].set_aspect("equal", adjustable="box")
        ax[0, 0].set_xlim([-3, 3])
        ax[1, 0].set_xlim([-3, 3])
        ax[1, 1].set_xlim([-3, 3])
        ax[0, 0].set_ylim([-3, 3])
        ax[1, 0].set_ylim([-3, 3])
        ax[1, 1].set_ylim([-3, 3])
        ax[0, 0].set_title("Plan")
        ax[0, 0].set_xlabel("x")
        ax[0, 0].set_ylabel("y")
        ax[1, 0].set_title("Front Elevation")
        ax[1, 0].set_xlabel("x")
        ax[1, 0].set_ylabel("z")
        ax[1, 1].set_title("End Elevation")
        ax[1, 1].set_xlabel("y")
        ax[1, 1].set_ylabel("z")
        ax[0, 1].axis("off")

        x = []
        y = []
        z = []
        colors = []
        circle_size = 3.0
        for i in range(len(data)):
            node = data[i]["pos"]
            x.append(node["x"])
            y.append(node["y"])
            z.append(node["z"])
            force = data[i]["force"]
            fx = force["x"]
            fy = force["y"]
            fz = force["z"]
            mag = math.sqrt(fx * fx + fy * fy + fz * fz)
            colors.append(mag)
            fac = 1 / max(1, mag)
            ax[0, 0].arrow(x[-1], y[-1], fx * fac, fy * fac, width=0.05, ec="red")
            ax[1, 0].arrow(x[-1], z[-1], fx * fac, fz * fac, width=0.05, ec="red")
            ax[1, 1].arrow(y[-1], z[-1], fy * fac, fz * fac, width=0.05, ec="red")

        cmap = matplotlib.cm.viridis
        s1 = ax[0, 0].scatter(x, y, s=circle_size, c=colors, cmap=cmap)
        s2 = ax[1, 0].scatter(x, z, s=circle_size, c=colors, cmap=cmap)
        s3 = ax[1, 1].scatter(y, z, s=circle_size, c=colors, cmap=cmap)
        fig.colorbar(s1)

        lns1, lns2, lns3 = self.compute_beam_line_segments()
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()

    def velocity_distribution_plot(self, data):
        sns.set_theme()  # Let seaborn apply better styling to all matplotlib graphs
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect("equal", adjustable="box")
        ax[1, 0].set_aspect("equal", adjustable="box")
        ax[1, 1].set_aspect("equal", adjustable="box")
        ax[0, 0].set_xlim([-3, 3])
        ax[1, 0].set_xlim([-3, 3])
        ax[1, 1].set_xlim([-3, 3])
        ax[0, 0].set_ylim([-3, 3])
        ax[1, 0].set_ylim([-3, 3])
        ax[1, 1].set_ylim([-3, 3])
        ax[0, 0].set_title("Plan")
        ax[0, 0].set_xlabel("x")
        ax[0, 0].set_ylabel("y")
        ax[1, 0].set_title("Front Elevation")
        ax[1, 0].set_xlabel("x")
        ax[1, 0].set_ylabel("z")
        ax[1, 1].set_title("End Elevation")
        ax[1, 1].set_xlabel("y")
        ax[1, 1].set_ylabel("z")
        ax[0, 1].axis("off")

        x = []
        y = []
        z = []
        colors = []
        circle_size = 3.0
        for i in range(len(data)):
            node = data[i]["pos"]
            x.append(node["x"])
            y.append(node["y"])
            z.append(node["z"])
            vel = data[i]["vel"]
            vx = vel["x"]
            vy = vel["y"]
            vz = vel["z"]
            colors.append(math.sqrt(vx * vx + vy * vy + vz * vz))

        cmap = matplotlib.cm.viridis
        s1 = ax[0, 0].scatter(x, y, s=circle_size, c=colors, cmap=cmap)
        s2 = ax[1, 0].scatter(x, z, s=circle_size, c=colors, cmap=cmap)
        s3 = ax[1, 1].scatter(y, z, s=circle_size, c=colors, cmap=cmap)
        fig.colorbar(s1)

        lns1, lns2, lns3 = self.compute_beam_line_segments()
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()

    def velocity_direction_plot(self, data):
        sns.set_theme()  # Let seaborn apply better styling to all matplotlib graphs
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect("equal", adjustable="box")
        ax[1, 0].set_aspect("equal", adjustable="box")
        ax[1, 1].set_aspect("equal", adjustable="box")
        ax[0, 0].set_xlim([-3, 3])
        ax[1, 0].set_xlim([-3, 3])
        ax[1, 1].set_xlim([-3, 3])
        ax[0, 0].set_ylim([-3, 3])
        ax[1, 0].set_ylim([-3, 3])
        ax[1, 1].set_ylim([-3, 3])
        ax[0, 0].set_title("Plan")
        ax[0, 0].set_xlabel("x")
        ax[0, 0].set_ylabel("y")
        ax[1, 0].set_title("Front Elevation")
        ax[1, 0].set_xlabel("x")
        ax[1, 0].set_ylabel("z")
        ax[1, 1].set_title("End Elevation")
        ax[1, 1].set_xlabel("y")
        ax[1, 1].set_ylabel("z")
        ax[0, 1].axis("off")

        x = []
        y = []
        z = []
        colors = []
        circle_size = 3.0
        for i in range(len(data)):
            node = data[i]["pos"]
            x.append(node["x"])
            y.append(node["y"])
            z.append(node["z"])
            vel = data[i]["vel"]
            vx = vel["x"]
            vy = vel["y"]
            vz = vel["z"]
            mag = math.sqrt(vx * vx + vy * vy + vz * vz)
            colors.append(mag)
            fac = 1 / max(1, mag)
            ax[0, 0].arrow(x[-1], y[-1], vx * fac, vy * fac, width=0.05, ec="red")
            ax[1, 0].arrow(x[-1], z[-1], vx * fac, vz * fac, width=0.05, ec="red")
            ax[1, 1].arrow(y[-1], z[-1], vy * fac, vz * fac, width=0.05, ec="red")

        cmap = matplotlib.cm.viridis
        s1 = ax[0, 0].scatter(x, y, s=circle_size, c=colors, cmap=cmap)
        s2 = ax[1, 0].scatter(x, z, s=circle_size, c=colors, cmap=cmap)
        s3 = ax[1, 1].scatter(y, z, s=circle_size, c=colors, cmap=cmap)
        fig.colorbar(s1)

        lns1, lns2, lns3 = self.compute_beam_line_segments()
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()
