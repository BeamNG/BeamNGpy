from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING, Any

from beamngpy.logging import LOGGER_ID, BNGError
from beamngpy.types import Float2, Float3, Int2, StrDict

from .communication_utils import send_sensor_request, set_sensor

import math

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import collections  as mc

import numpy as np
import seaborn as sns

sns.set()  # Let seaborn apply better styling to all matplotlib graphs

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

__all__ = ['Mesh']


class Mesh:

    def __init__(self, name: str, bng: BeamNGpy, vehicle: Vehicle, gfx_update_time: float = 0.0, physics_update_time: float = 0.01, is_send_immediately: bool = False):
        self.logger = getLogger(f'{LOGGER_ID}.Mesh')
        self.logger.setLevel(DEBUG)

        self.name = name
        self.vehicle = vehicle
        self.vid = vehicle.vid
        self.bng = bng
        self.is_send_immediately = is_send_immediately

        # Create and initialise this sensor in the simulation.
        self._open_mesh(name, vehicle, gfx_update_time, physics_update_time, is_send_immediately)

        # Fetch the unique Id number (in the simulator) for this mesh sensor.  We will need this later.
        self.sensorId = self._get_mesh_id()

        # Populate the list of triangles for this sensor.
        d = self._send_sensor_request(
            'GetFullTriangleData',
            ack='CompletedGetFullTriangleData',
            vid=self.vid)['data']
        self.triangles = {}
        for key, value in d.items():
            self.triangles[int(key)] = [int(value[0]), int(value[1]), int(value[2])]

        self.num_nodes = self.get_num_nodes()

        self.logger.debug('Mesh - sensor created: 'f'{self.name}')

    def _send_sensor_request(self, type: str, ack: str | None = None, **kwargs: Any) -> StrDict:
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        return send_sensor_request(self.bng.connection, type, ack, **kwargs)

    def _set_sensor(self, type: str, **kwargs: Any) -> None:
        if not self.bng.connection:
            raise BNGError('The simulator is not connected!')
        set_sensor(self.bng.connection, type, **kwargs)

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_mesh()
        self.logger.debug('Mesh - sensor removed: 'f'{self.name}')

    def poll(self) -> StrDict:
        """
        Gets the most-recent readings for this sensor.
        Note: if this sensor was created with a negative update rate, then there may have been no readings taken.

        Returns:
            A dictionary containing the sensor readings data.
        """
        # Send and receive a request for readings data from this sensor.
        readings_data = []
        if self.is_send_immediately:
            # Get the most-recent single reading from vlua.
            readings_data = self._poll_mesh_VE()
        else:
            # Get the bulk data from ge lua.
            readings_data = self._poll_mesh_GE()

        self.logger.debug('Mesh - sensor readings received from simulation: 'f'{self.name}')
        return readings_data

    def send_ad_hoc_poll_request(self) -> int:
        """
        Sends an ad-hoc polling request to the simulator. This will be executed by the simulator immediately, but will take time to process, so the
        result can be queried after some time has passed. To check if it has been processed, we first call the is_ad_hoc_poll_request_ready() function,
        then call the collect_ad_hoc_poll_request() function to retrieve the sensor reading.

        Returns:
            A unique Id number for the ad-hoc request.
        """
        self.logger.debug('Mesh - ad-hoc polling request sent: 'f'{self.name}')
        return int(self._send_sensor_request(
            'SendAdHocRequestMesh', ack='CompletedSendAdHocRequestMesh', name=self.name,
            vid=self.vehicle.vid)['data'])

    def is_ad_hoc_poll_request_ready(self, request_id: int) -> bool:
        """
        Checks if a previously-issued ad-hoc polling request has been processed and is ready to collect.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            A flag which indicates if the ad-hoc polling request is complete.
        """
        self.logger.debug('Mesh - ad-hoc polling request checked for completion: 'f'{self.name}')
        return self._send_sensor_request('IsAdHocPollRequestReadyMesh',
                                         ack='CompletedIsAdHocPollRequestReadyMesh', requestId=request_id)['data']

    def collect_ad_hoc_poll_request(self, request_id: int) -> StrDict:
        """
        Collects a previously-issued ad-hoc polling request, if it has been processed.

        Args:
            request_id: The unique Id number of the ad-hoc request. This was returned from the simulator upon sending the ad-hoc polling request.

        Returns:
            The readings data.
        """
        readings = self._send_sensor_request('CollectAdHocPollRequestMesh',
                                             ack='CompletedCollectAdHocPollRequestMesh', requestId=request_id)['data']
        self.logger.debug('Mesh - ad-hoc polling request returned and processed: 'f'{self.name}')
        return readings

    def set_requested_update_time(self, requested_update_time: float) -> None:
        """
        Sets the current 'requested update time' value for this sensor.

        Args:
            requested_update_time: The new requested update time.
        """
        self._set_sensor(
            'SetMeshRequestedUpdateTime', ack='CompletedSetMeshRequestedUpdateTime', name=self.name, vid=self.vehicle.vid,
            GFXUpdateTime=requested_update_time)

    def _get_mesh_id(self) -> int:
        return int(self._send_sensor_request('GetMeshId', ack='CompletedGetMeshId', name=self.name)['data'])

    def _open_mesh(self, name: str, vehicle: Vehicle, gfx_update_time: float, physics_update_time: float, is_send_immediately: bool) -> None:
        data: StrDict = dict(type='OpenMesh')
        data['name'] = name
        data['vid'] = vehicle.vid
        data['GFXUpdateTime'] = gfx_update_time
        data['physicsUpdateTime'] = physics_update_time
        data['isSendImmediately'] = is_send_immediately
        self.bng._send(data).ack('OpenedMesh')
        self.logger.info(f'Opened Mesh sensor: "{name}"')

    def _close_mesh(self) -> None:
        data = dict(type='CloseMesh')
        data['name'] = self.name
        data['vid'] = self.vehicle.vid
        self.bng._send(data).ack('ClosedMesh')
        self.logger.info(f'Closed Mesh sensor: "{self.name}"')

    def _poll_mesh_GE(self) -> StrDict:
        return self._send_sensor_request('PollMeshGE', ack='PolledMeshGECompleted', name=self.name)['data']

    def _poll_mesh_VE(self) -> StrDict:
        if not self.vehicle.connection:
            raise BNGError('The vehicle is not connected!')
        return send_sensor_request(self.vehicle.connection, 'PollMeshVE', name=self.name, sensorId=self.sensorId)['data']

    def get_triangle_data(self):
        return self.triangles

    def get_wheel_mesh(self, wheel_id):
        d = self._send_sensor_request(
            'GetWheelTriangleData',
            ack='CompletedGetWheelTriangleData',
            vid=self.vid,
            wheelIndex=wheel_id)['data']
        mesh = {}
        for key, value in d.items():
            mesh[int(key)] = [int(value[0]), int(value[1]), int(value[2])]
        return mesh

    def get_node_positions(self):
        d = self._send_sensor_request(
            'GetNodePositions',
            ack='CompletedGetNodePositions',
            vid=self.vid)['data']
        nodes = {}
        ctr = 0
        for i in range(0, len(d), 3):
            nodes[ctr] = [d[i], d[i + 1], d[i + 2]]
            ctr = ctr + 1
        return nodes

    def get_closest_mesh_point_to_point(self, point):
        return self._send_sensor_request(
            'GetClosestMeshPointToGivenPoint',
            ack='CompletedGetClosestMeshPointToGivenPoint',
            vid=self.vid,
            point = point)['data']

    def get_closest_vehicle_triangle_to_point(self, point, is_include_wheels):
        d = self._send_sensor_request(
            'GetClosestTriangle',
            ack='CompletedGetClosestTriangle',
            vid=self.vid,
            point = point,
            includeWheelNodes=is_include_wheels)['data']
        return [int(d['nodeIndex1']), int(d['nodeIndex2']), int(d['nodeIndex3'])]

    def get_num_nodes(self):
        max_idx = -1
        for _, v in self.triangles.items():
            if v[0] > max_idx:
                max_idx = v[0]
            if v[1] > max_idx:
                max_idx = v[1]
            if v[2] > max_idx:
                max_idx = v[2]
        return max_idx

    def get_nodes_to_triangles_map(self):
        map = {}
        for i in range(self.num_nodes + 1):
            map[i] = []
        for k, v in self.triangles.items():
            map[v[0]].append(k)
            map[v[1]].append(k)
            map[v[2]].append(k)
        return map

    def get_neighbor_nodes(self, node_id):
        neighbors = []
        for _, v in self.triangles.items():
            if node_id == v[0]:
                neighbors.append(v[1])
                neighbors.append(v[2])
            if node_id == v[1]:
                neighbors.append(v[0])
                neighbors.append(v[2])
            if node_id == v[2]:
                neighbors.append(v[0])
                neighbors.append(v[1])
        return neighbors

    def get_neighbor_triangles(self, triangle_id):
        t = self.triangles[triangle_id]
        neighbors = []
        for k, v in self.triangles.items():
            match0 = t[0] == v[0] or t[0] == v[1] or t[0] == v[2]
            match1 = t[1] == v[0] or t[1] == v[1] or t[1] == v[2]
            match2 = t[2] == v[0] or t[2] == v[1] or t[2] == v[2]
            if match0 or match1 or match2:
                neighbors.append(k)
        return neighbors

    def mesh_plot(self):
        nodes = self.get_node_positions()
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect('equal', adjustable='box')
        ax[1, 0].set_aspect('equal', adjustable='box')
        ax[1, 1].set_aspect('equal', adjustable='box')
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
        ax[0, 1].axis('off')
        for i in range(len(nodes)):
            ax[0, 0].plot(nodes[i][0], nodes[i][1],'ro')
            ax[1, 0].plot(nodes[i][0], nodes[i][2],'ro')
            ax[1, 1].plot(nodes[i][1], nodes[i][2],'ro')

        lines1 = []
        lines2 = []
        lines3 = []
        c = []
        for _, v in self.triangles.items():
            p1 = nodes[v[0]]
            p2 = nodes[v[1]]
            p3 = nodes[v[2]]
            lines1.append([(p1[0], p1[1]), (p2[0], p2[1])])
            lines1.append([(p2[0], p2[1]), (p3[0], p3[1])])
            lines1.append([(p1[0], p1[1]), (p3[0], p3[1])])
            lines2.append([(p1[0], p1[2]), (p2[0], p2[2])])
            lines2.append([(p2[0], p2[2]), (p3[0], p3[2])])
            lines2.append([(p1[0], p1[2]), (p3[0], p3[2])])
            lines3.append([(p1[1], p1[2]), (p2[1], p2[2])])
            lines3.append([(p2[1], p2[2]), (p3[1], p3[2])])
            lines3.append([(p1[1], p1[2]), (p3[1], p3[2])])
            c.append((0, 0, 1, 1))
            c.append((0, 0, 1, 1))
            c.append((0, 0, 1, 1))
        lns1 = mc.LineCollection(lines1, colors=c, linewidths=0.5)
        lns2 = mc.LineCollection(lines2, colors=c, linewidths=0.5)
        lns3 = mc.LineCollection(lines3, colors=c, linewidths=0.5)
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()

    def mass_distribution_plot(self, data):
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect('equal', adjustable='box')
        ax[1, 0].set_aspect('equal', adjustable='box')
        ax[1, 1].set_aspect('equal', adjustable='box')
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
        ax[0, 1].axis('off')

        x = []
        y = []
        z = []
        colors = []
        circle_size = 3.0
        for i in range(len(data)):
            x.append(data[i]['pos'][0])
            y.append(data[i]['pos'][1])
            z.append(data[i]['pos'][2])
            colors.append(data[i]['mass'])

        cmap = matplotlib.cm.viridis
        s1 = ax[0, 0].scatter(x, y, s=circle_size, c=colors, cmap=cmap)
        s2 = ax[1, 0].scatter(x, z, s=circle_size, c=colors, cmap=cmap)
        s3 = ax[1, 1].scatter(y, z, s=circle_size, c=colors, cmap=cmap)
        fig.colorbar(s1)

        nodes = self.get_node_positions()
        lines1 = []
        lines2 = []
        lines3 = []
        c = []
        for _, v in self.triangles.items():
            p1 = nodes[v[0]]
            p2 = nodes[v[1]]
            p3 = nodes[v[2]]
            lines1.append([(p1[0], p1[1]), (p2[0], p2[1])])
            lines1.append([(p2[0], p2[1]), (p3[0], p3[1])])
            lines1.append([(p1[0], p1[1]), (p3[0], p3[1])])
            lines2.append([(p1[0], p1[2]), (p2[0], p2[2])])
            lines2.append([(p2[0], p2[2]), (p3[0], p3[2])])
            lines2.append([(p1[0], p1[2]), (p3[0], p3[2])])
            lines3.append([(p1[1], p1[2]), (p2[1], p2[2])])
            lines3.append([(p2[1], p2[2]), (p3[1], p3[2])])
            lines3.append([(p1[1], p1[2]), (p3[1], p3[2])])
            c.append((0.3, 0.3, 0.3, 0.1))
            c.append((0.3, 0.3, 0.3, 0.1))
            c.append((0.3, 0.3, 0.3, 0.1))
        lns1 = mc.LineCollection(lines1, colors=c, linewidths=0.5)
        lns2 = mc.LineCollection(lines2, colors=c, linewidths=0.5)
        lns3 = mc.LineCollection(lines3, colors=c, linewidths=0.5)
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()

    def force_distribution_plot(self, data):
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect('equal', adjustable='box')
        ax[1, 0].set_aspect('equal', adjustable='box')
        ax[1, 1].set_aspect('equal', adjustable='box')
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
        ax[0, 1].axis('off')

        x = []
        y = []
        z = []
        colors = []
        circle_size = 3.0
        for i in range(len(data)):
            x.append(data[i]['pos'][0])
            y.append(data[i]['pos'][1])
            z.append(data[i]['pos'][2])
            fx = data[i]['force'][0]
            fy = data[i]['force'][1]
            fz = data[i]['force'][2]
            colors.append(math.sqrt(fx * fx + fy * fy + fz * fz))

        cmap = matplotlib.cm.viridis
        s1 = ax[0, 0].scatter(x, y, s=circle_size, c=colors, cmap=cmap)
        s2 = ax[1, 0].scatter(x, z, s=circle_size, c=colors, cmap=cmap)
        s3 = ax[1, 1].scatter(y, z, s=circle_size, c=colors, cmap=cmap)
        fig.colorbar(s1)

        nodes = self.get_node_positions()
        lines1 = []
        lines2 = []
        lines3 = []
        c = []
        for _, v in self.triangles.items():
            p1 = nodes[v[0]]
            p2 = nodes[v[1]]
            p3 = nodes[v[2]]
            lines1.append([(p1[0], p1[1]), (p2[0], p2[1])])
            lines1.append([(p2[0], p2[1]), (p3[0], p3[1])])
            lines1.append([(p1[0], p1[1]), (p3[0], p3[1])])
            lines2.append([(p1[0], p1[2]), (p2[0], p2[2])])
            lines2.append([(p2[0], p2[2]), (p3[0], p3[2])])
            lines2.append([(p1[0], p1[2]), (p3[0], p3[2])])
            lines3.append([(p1[1], p1[2]), (p2[1], p2[2])])
            lines3.append([(p2[1], p2[2]), (p3[1], p3[2])])
            lines3.append([(p1[1], p1[2]), (p3[1], p3[2])])
            c.append((0.3, 0.3, 0.3, 0.1))
            c.append((0.3, 0.3, 0.3, 0.1))
            c.append((0.3, 0.3, 0.3, 0.1))
        lns1 = mc.LineCollection(lines1, colors=c, linewidths=0.5)
        lns2 = mc.LineCollection(lines2, colors=c, linewidths=0.5)
        lns3 = mc.LineCollection(lines3, colors=c, linewidths=0.5)
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()

    def force_direction_plot(self, data):
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect('equal', adjustable='box')
        ax[1, 0].set_aspect('equal', adjustable='box')
        ax[1, 1].set_aspect('equal', adjustable='box')
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
        ax[0, 1].axis('off')

        x = []
        y = []
        z = []
        colors = []
        circle_size = 3.0
        for i in range(len(data)):
            x.append(data[i]['pos'][0])
            y.append(data[i]['pos'][1])
            z.append(data[i]['pos'][2])
            fx = data[i]['force'][0]
            fy = data[i]['force'][1]
            fz = data[i]['force'][2]
            mag = math.sqrt(fx * fx + fy * fy + fz * fz)
            colors.append(mag)
            fac = 1/max(1, mag)
            ax[0, 0].arrow(x[-1], y[-1], fx*fac, fy*fac, width = 0.05, ec = 'red')
            ax[1, 0].arrow(x[-1], z[-1], fx*fac, fz*fac, width = 0.05, ec = 'red')
            ax[1, 1].arrow(y[-1], z[-1], fy*fac, fz*fac, width = 0.05, ec = 'red')

        cmap = matplotlib.cm.viridis
        s1 = ax[0, 0].scatter(x, y, s=circle_size, c=colors, cmap=cmap)
        s2 = ax[1, 0].scatter(x, z, s=circle_size, c=colors, cmap=cmap)
        s3 = ax[1, 1].scatter(y, z, s=circle_size, c=colors, cmap=cmap)
        fig.colorbar(s1)

        nodes = self.get_node_positions()
        lines1 = []
        lines2 = []
        lines3 = []
        c = []
        for _, v in self.triangles.items():
            p1 = nodes[v[0]]
            p2 = nodes[v[1]]
            p3 = nodes[v[2]]
            lines1.append([(p1[0], p1[1]), (p2[0], p2[1])])
            lines1.append([(p2[0], p2[1]), (p3[0], p3[1])])
            lines1.append([(p1[0], p1[1]), (p3[0], p3[1])])
            lines2.append([(p1[0], p1[2]), (p2[0], p2[2])])
            lines2.append([(p2[0], p2[2]), (p3[0], p3[2])])
            lines2.append([(p1[0], p1[2]), (p3[0], p3[2])])
            lines3.append([(p1[1], p1[2]), (p2[1], p2[2])])
            lines3.append([(p2[1], p2[2]), (p3[1], p3[2])])
            lines3.append([(p1[1], p1[2]), (p3[1], p3[2])])
            c.append((0.3, 0.3, 0.3, 0.1))
            c.append((0.3, 0.3, 0.3, 0.1))
            c.append((0.3, 0.3, 0.3, 0.1))
        lns1 = mc.LineCollection(lines1, colors=c, linewidths=0.5)
        lns2 = mc.LineCollection(lines2, colors=c, linewidths=0.5)
        lns3 = mc.LineCollection(lines3, colors=c, linewidths=0.5)
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()

    def velocity_distribution_plot(self, data):
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect('equal', adjustable='box')
        ax[1, 0].set_aspect('equal', adjustable='box')
        ax[1, 1].set_aspect('equal', adjustable='box')
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
        ax[0, 1].axis('off')

        x = []
        y = []
        z = []
        colors = []
        circle_size = 3.0
        for i in range(len(data)):
            x.append(data[i]['pos'][0])
            y.append(data[i]['pos'][1])
            z.append(data[i]['pos'][2])
            vx = data[i]['vel'][0]
            vy = data[i]['vel'][1]
            vz = data[i]['vel'][2]
            colors.append(math.sqrt(vx * vx + vy * vy + vz * vz))

        cmap = matplotlib.cm.viridis
        s1 = ax[0, 0].scatter(x, y, s=circle_size, c=colors, cmap=cmap)
        s2 = ax[1, 0].scatter(x, z, s=circle_size, c=colors, cmap=cmap)
        s3 = ax[1, 1].scatter(y, z, s=circle_size, c=colors, cmap=cmap)
        fig.colorbar(s1)

        nodes = self.get_node_positions()
        lines1 = []
        lines2 = []
        lines3 = []
        c = []
        for _, v in self.triangles.items():
            p1 = nodes[v[0]]
            p2 = nodes[v[1]]
            p3 = nodes[v[2]]
            lines1.append([(p1[0], p1[1]), (p2[0], p2[1])])
            lines1.append([(p2[0], p2[1]), (p3[0], p3[1])])
            lines1.append([(p1[0], p1[1]), (p3[0], p3[1])])
            lines2.append([(p1[0], p1[2]), (p2[0], p2[2])])
            lines2.append([(p2[0], p2[2]), (p3[0], p3[2])])
            lines2.append([(p1[0], p1[2]), (p3[0], p3[2])])
            lines3.append([(p1[1], p1[2]), (p2[1], p2[2])])
            lines3.append([(p2[1], p2[2]), (p3[1], p3[2])])
            lines3.append([(p1[1], p1[2]), (p3[1], p3[2])])
            c.append((0.3, 0.3, 0.3, 0.1))
            c.append((0.3, 0.3, 0.3, 0.1))
            c.append((0.3, 0.3, 0.3, 0.1))
        lns1 = mc.LineCollection(lines1, colors=c, linewidths=0.5)
        lns2 = mc.LineCollection(lines2, colors=c, linewidths=0.5)
        lns3 = mc.LineCollection(lines3, colors=c, linewidths=0.5)
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()

    def velocity_direction_plot(self, data):
        fig, ax = plt.subplots(2, 2)
        ax[0, 0].set_aspect('equal', adjustable='box')
        ax[1, 0].set_aspect('equal', adjustable='box')
        ax[1, 1].set_aspect('equal', adjustable='box')
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
        ax[0, 1].axis('off')

        x = []
        y = []
        z = []
        colors = []
        circle_size = 3.0
        for i in range(len(data)):
            x.append(data[i]['pos'][0])
            y.append(data[i]['pos'][1])
            z.append(data[i]['pos'][2])
            vx = data[i]['vel'][0]
            vy = data[i]['vel'][1]
            vz = data[i]['vel'][2]
            mag = math.sqrt(vx * vx + vy * vy + vz * vz)
            colors.append(mag)
            fac = 1/max(1, mag)
            ax[0, 0].arrow(x[-1], y[-1], vx*fac, vy*fac, width = 0.05, ec = 'red')
            ax[1, 0].arrow(x[-1], z[-1], vx*fac, vz*fac, width = 0.05, ec = 'red')
            ax[1, 1].arrow(y[-1], z[-1], vy*fac, vz*fac, width = 0.05, ec = 'red')

        cmap = matplotlib.cm.viridis
        s1 = ax[0, 0].scatter(x, y, s=circle_size, c=colors, cmap=cmap)
        s2 = ax[1, 0].scatter(x, z, s=circle_size, c=colors, cmap=cmap)
        s3 = ax[1, 1].scatter(y, z, s=circle_size, c=colors, cmap=cmap)
        fig.colorbar(s1)

        nodes = self.get_node_positions()
        lines1 = []
        lines2 = []
        lines3 = []
        c = []
        for _, v in self.triangles.items():
            p1 = nodes[v[0]]
            p2 = nodes[v[1]]
            p3 = nodes[v[2]]
            lines1.append([(p1[0], p1[1]), (p2[0], p2[1])])
            lines1.append([(p2[0], p2[1]), (p3[0], p3[1])])
            lines1.append([(p1[0], p1[1]), (p3[0], p3[1])])
            lines2.append([(p1[0], p1[2]), (p2[0], p2[2])])
            lines2.append([(p2[0], p2[2]), (p3[0], p3[2])])
            lines2.append([(p1[0], p1[2]), (p3[0], p3[2])])
            lines3.append([(p1[1], p1[2]), (p2[1], p2[2])])
            lines3.append([(p2[1], p2[2]), (p3[1], p3[2])])
            lines3.append([(p1[1], p1[2]), (p3[1], p3[2])])
            c.append((0.3, 0.3, 0.3, 0.1))
            c.append((0.3, 0.3, 0.3, 0.1))
            c.append((0.3, 0.3, 0.3, 0.1))
        lns1 = mc.LineCollection(lines1, colors=c, linewidths=0.5)
        lns2 = mc.LineCollection(lines2, colors=c, linewidths=0.5)
        lns3 = mc.LineCollection(lines3, colors=c, linewidths=0.5)
        ax[0, 0].add_collection(lns1)
        ax[1, 0].add_collection(lns2)
        ax[1, 1].add_collection(lns3)

        plt.show()