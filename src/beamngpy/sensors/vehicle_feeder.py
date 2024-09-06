from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID, create_warning
from beamngpy.misc.vec3 import vec3

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

import csv
import math
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

pypdf_installed = False
try:
    from PyPDF2 import PdfMerger, PdfReader

    pypdf_installed = True
except ImportError:
    pypdf_installed = False

__all__ = ["VehicleFeeder"]


class TemporalSmoother:

    def __init__(self, stVal):
        self.state = stVal

    def smooth(self, sample, dt, rate):
        st = self.state
        ratedt = rate * dt
        st = st + (sample - st) * ratedt / (1 + ratedt)
        self.state = st
        return st


class VehicleFeeder(CommBase):

    def __init__(self, name: str, bng: BeamNGpy, vehicle: Vehicle, test_id: int):
        super().__init__(bng, vehicle)
        self.logger = getLogger(f"{LOGGER_ID}.Advanced IMU")
        self.logger.setLevel(DEBUG)

        # Cache some properties we will need later.
        self.name = name
        self.vehicle = vehicle

        # Create and initialise this sensor in the simulation.
        self._open_vehicle_feeder(name, vehicle, test_id)

        self.logger.debug("Vehicle Feeder created: " f"{self.name}")

    def remove(self) -> None:
        """
        Removes this sensor from the simulation.
        """
        # Remove this sensor from the simulation.
        self._close_vehicle_feeder()
        self.logger.debug("Vehicle Feeder removed: " f"{self.name}")

    def is_time_evolution_complete(self) -> bool:
        return bool(
            self.send_recv_ge("IsTimeEvolutionComplete", vid=self.vehicle.vid)["data"]
        )

    def _open_vehicle_feeder(self, name: str, vehicle: Vehicle, test_id: int) -> None:
        self.send_ack_ge(
            type="OpenVehicleFeeder",
            ack="OpenedVehicleFeeder",
            name=name,
            vid=vehicle.vid,
            testId=test_id,
        )
        self.logger.info(f'Opened vehicle feeder: "{name}"')

    def _close_vehicle_feeder(self) -> None:
        self.send_ack_ge(
            type="CloseVehicleFeeder",
            ack="ClosedVehicleFeeder",
            name=self.name,
            vid=self.vehicle.vid,
        )
        self.logger.info(f'Closed vehicle feeder: "{self.name}"')

    def smoothExp(self, d, rate):
        s = []
        sm = TemporalSmoother(d[0])
        s.append(d[0])
        for i in range(1, len(d)):
            smDat = sm.smooth(d[i], 1, rate)
            s.append(smDat)
        return s

    def smoothExpBothWays(self, d, rate):
        fwd = self.smoothExp(d, rate)
        return self.smoothExp(fwd[::-1], rate)[::-1]

    def smoothTemporal(self, d, t, rate):
        s = []
        sm = TemporalSmoother(d[0])
        s.append(d[0])
        for i in range(1, len(d)):
            smDat = sm.smooth(d[i], abs(t[i] - t[i - 1]), rate)
            s.append(smDat)
        return s

    def smoothTemporalBothWays(self, d, t, rate):
        fwd = self.smoothTemporal(d, t, rate)
        return self.smoothTemporal(fwd[::-1], t[::-1], rate)[::-1]

    def getPos(self, d):
        dx = d["y_signal_preprocess_gps_front_pos_0_"]
        dy = d["y_signal_preprocess_gps_front_pos_1_"]
        dz = d["y_signal_preprocess_gps_front_pos_2_"]
        markers = self.computeMarkers(dx)
        pos = []
        t = []
        for i in range(0, len(markers)):
            pos.append(vec3(dx[markers[i]], dy[markers[i]], dz[markers[i]]))
            t.append(d["t"][markers[i]])
        return [pos, t]

    def computeVelFromGPS(self, d):
        d1 = d["y_signal_preprocess_gps_front_pos_0_"]
        d2 = d["y_signal_preprocess_gps_front_pos_1_"]
        markers = self.computeMarkers(d1)
        vel = []
        t = []
        for i in range(1, len(markers)):
            a = markers[i - 1]
            b = markers[i]
            dt = d["t"][b] - d["t"][a]
            p1 = vec3(d1[a], d1[a], d1[a])
            p2 = vec3(d2[b], d2[b], d2[b])
            vel.append((p2 - p1) / dt)
            t.append(d["t"][b])
        return [vel, t]

    def computeAccelFromGPS(self, d):
        vx = d["vel_x_from_GPS"]
        vy = d["vel_y_from_GPS"]
        vz = d["vel_z_from_GPS"]
        acc = []
        t = []
        for i in range(1, len(d["vel_x_from_GPS"])):
            a = i - 1
            b = i
            dt = d["t"][b] - d["t"][a]
            p1 = vec3(vx[a], vy[a], vz[a])
            p2 = vec3(vx[b], vy[b], vz[b])
            acc.append((p2 - p1) / dt)
            t.append(d["t"][b])
        return [acc, t]

    def computeWeightedAverage(self, markers, Fpositions, Bpositions):
        # Compute weighted average positions from forward and backward integrations.
        avgPos = []

        for i in range(len(markers) - 1):
            siA = markers[i]
            siB = markers[i + 1]
            for j in range(siA, siB):
                wBwd = (j - siA) / (
                    siB - siA
                )  # Graded weight from GPS points A to B. Sums to unity.
                wFwd = 1.0 - wBwd
                avgPos.append(
                    vec3(
                        (wFwd * Fpositions[j].x) + (wBwd * Bpositions[0][j]),
                        (wFwd * Fpositions[j].y) + (wBwd * Bpositions[1][j]),
                        (wFwd * Fpositions[j].z) + (wBwd * Bpositions[2][j]),
                    )
                )
        return avgPos

    def computeMarkers(self, d):
        markers = []
        markers.append(0)  # Start with index 0.
        last_marked_val = d[0]
        for i in range(1, len(d)):
            if abs(d[i] - last_marked_val) > 1e-12:
                markers.append(i)
                last_marked_val = d[i]
        if (
            markers[-1] != len(d) - 1
        ):  # Make sure the last index is included at the end of the markers.
            markers.append(len(d) - 1)

        return markers

    def computeIntermediatePositions(
        self, startIdx, endIdx, prevIdx, gpsX, gpsY, gpsZ, AccX, AccY, AccZ, times
    ):
        # If there are no intermediate points, leave immediately with empty lists.
        pos = []
        if endIdx - startIdx < 1:
            return pos

        # Compute oldV.
        p = vec3(gpsX[startIdx], gpsY[startIdx], gpsZ[startIdx])
        pLast = vec3(gpsX[prevIdx], gpsY[prevIdx], gpsZ[prevIdx])
        pNext = vec3(gpsX[endIdx], gpsY[endIdx], gpsZ[endIdx])
        vA = (p - pLast) / (times[startIdx] - times[prevIdx])
        vB = (pNext - p) / (times[endIdx] - times[startIdx])
        vel = (vA + vB) * 0.5

        # Iterate over each index and estimate the position vector using Verlet integration.
        for j in range(startIdx + 1, endIdx):
            # Compute dt.
            lastTime = 0.0
            if j > 0:
                lastTime = times[j - 1]
            dt = times[j] - lastTime

            # Use Leapfrog Integration to get the current position value: newPos = oldPos + newV * dt [where newV = oldV + Accel * dt]
            acc = vec3(AccX[j], AccY[j], AccZ[j])
            vel = vel + (acc * dt)
            p = p + (vel * dt)
            pos.append(p)
        return pos

    def integrate(self, markers, gpsX, gpsY, gpsZ, AccX, AccY, AccZ, times):
        # Iterate over the sub-intervals given by the marker points.
        pos = []
        for i in range(len(markers) - 1):
            startIdx = markers[i]
            endIdx = markers[i + 1]
            prevIdx = -1
            if i > 0:
                prevIdx = markers[i - 1]

            # First, append the interval start point (GPS data).
            pos.append(vec3(gpsX[startIdx], gpsY[startIdx], gpsZ[startIdx]))

            # Append predictions for the intermediate positions in this sub-interval.
            posData = self.computeIntermediatePositions(
                startIdx, endIdx, prevIdx, gpsX, gpsY, gpsZ, AccX, AccY, AccZ, times
            )
            for j in range(len(posData)):
                pos.append(posData[j])

        # Append the last point at the very end (GPS data).
        lastIdx = markers[-1]
        pos.append(vec3(gpsX[lastIdx], gpsY[lastIdx], gpsZ[lastIdx]))

        return pos

    def geometrically_smooth(self, gpsX, gpsY, gpsZ):
        rope_length = 0.2
        rope_pos = vec3(gpsX[0], gpsY[0], gpsZ[0])
        outX, outY, outZ = [], [], []
        for i in range(1, len(gpsX)):
            cur_pos = vec3(gpsX[i], gpsY[i], gpsZ[i])
            rope_pos = cur_pos - (cur_pos - rope_pos).normalize() * min(
                (cur_pos - rope_pos).length(), rope_length
            )
            outX.append(rope_pos.x)
            outY.append(rope_pos.y)
            outZ.append(rope_pos.z)
        return [outX, outY, outZ]

    def getTrajectoryBasic(self, d):
        # Translate the GPS data to always start at the origin (we use the front GPS device).
        dx = d["y_signal_preprocess_gps_front_pos_0_"]
        dy = d["y_signal_preprocess_gps_front_pos_1_"]
        dz = d["y_signal_preprocess_gps_front_pos_2_"]
        gpsX2 = [x - dx[0] for x in dx]
        gpsY2 = [y - dy[0] for y in dy]
        gpsZ2 = [z - dz[0] for z in dz]

        # Geometrically smooth the GPS positions.
        return self.geometrically_smooth(gpsX2, gpsY2, gpsZ2)

    def getTrajectoryBasic2(self, d):
        # Translate the GPS data to always start at the origin (we use the front GPS device).
        dx = d["y_signal_preprocess_gps_front_pos_0_"]
        dy = d["y_signal_preprocess_gps_front_pos_1_"]
        dz = d["y_signal_preprocess_gps_front_pos_2_"]
        gpsX2 = [x - dx[0] for x in dx]
        gpsY2 = [y - dy[0] for y in dy]
        gpsZ2 = [z - dz[0] for z in dz]

        # Geometrically smooth the GPS positions.
        return [gpsX2, gpsY2, gpsZ2]  # geometrically_smooth(gpsX2, gpsY2, gpsZ2)

    def getTrajectoryFromGPS(self, d):
        # Translate the GPS data to always start at the origin (we use the front GPS device).
        dx = d["y_signal_preprocess_gps_front_pos_0_"]
        dy = d["y_signal_preprocess_gps_front_pos_1_"]
        dz = d["y_signal_preprocess_gps_front_pos_2_"]
        gpsX = [x - dx[0] for x in dx]
        gpsY = [y - dy[0] for y in dy]
        gpsZ = [z - dz[0] for z in dz]

        # Cache the acceleration vector values.
        AccX = d["y_signal_preprocess_imu_1_Occurence1__acc_0_"]
        AccY = d["y_signal_preprocess_imu_1_Occurence1__acc_1_"]
        AccZ = d["y_signal_preprocess_imu_1_Occurence1__acc_2_"]

        # Cache the time values.
        times = d["t"]

        # Compute predictions for the positions using forward-integration.
        markers = self.computeMarkers(gpsX)
        Fpositions = self.integrate(markers, gpsX, gpsY, gpsZ, AccX, AccY, AccZ, times)

        # Now flip all the data.
        BgpsX = gpsX[::-1]
        BgpsY = gpsY[::-1]
        BgpsZ = gpsZ[::-1]
        flippedAccX = AccX[::-1]
        flippedAccY = AccY[::-1]
        flippedAccZ = AccZ[::-1]
        BAccX = [x * -1 for x in flippedAccX]
        BAccY = [y * -1 for y in flippedAccY]
        BAccZ = [z * -1 for z in flippedAccZ]
        Btimes = times[::-1]

        # Compute predictions for the positions using backwards-integration.
        flippedMarkers = self.computeMarkers(BgpsX)
        BpositionsRaw = self.integrate(
            flippedMarkers, BgpsX, BgpsY, BgpsZ, BAccX, BAccY, BAccZ, Btimes
        )
        BposXFlippedBack = []
        BposYFlippedBack = []
        BposZFlippedBack = []
        for i in range(len(BpositionsRaw) - 1, 0, -1):
            BposXFlippedBack.append(BpositionsRaw[i].x)
            BposYFlippedBack.append(BpositionsRaw[i].y)
            BposZFlippedBack.append(BpositionsRaw[i].z)
        Bpositions = [BposXFlippedBack, BposYFlippedBack, BposZFlippedBack]

        # Get the final data for presentation.
        finalPositions = []
        finalPositions.append(
            self.computeWeightedAverage(markers, Fpositions, Bpositions)
        )

        finalPositions.append(markers)
        finalPositions.append(gpsX)
        finalPositions.append(gpsY)
        finalPositions.append(gpsZ)

        return finalPositions

    def getTrajectoryLength(self, traj):
        dx, dy, dz = traj[0], traj[1], traj[2]
        dist = 0.0
        for i in range(1, len(dx)):
            p0 = vec3(dx[i - 1], dy[i - 1], dz[i - 1])
            p1 = vec3(dx[i], dy[i], dz[i])
            dist = dist + p0.distance(p1)
        return dist

    def getTrajectoryFromPos(self, d):
        # Translate the position data to always start at the origin.
        startX = d["pos1"][0]
        startY = d["pos2"][0]
        startZ = d["pos3"][0]
        translatedX = [x - startX for x in d["pos1"]]
        translatedY = [y - startY for y in d["pos2"]]
        translatedZ = [z - startZ for z in d["pos3"]]

        return [translatedX, translatedY, translatedZ]

    def orientOurData(self, ourData, theirData):
        p1x = ourData["pos1"][0]
        p1y = ourData["pos2"][0]
        p2x = ourData["pos1"][50]
        p2y = ourData["pos2"][50]
        xa = p2x - p1x
        ya = p2y - p1y
        mag = math.sqrt(xa * xa + ya * ya)
        xa /= mag
        ya /= mag

        # Translate the GPS data to always start at the origin (we use the front GPS device).
        startX = theirData["y_signal_preprocess_gps_front_pos_0_"][0]
        startY = theirData["y_signal_preprocess_gps_front_pos_1_"][0]
        gpsX = [x - startX for x in theirData["y_signal_preprocess_gps_front_pos_0_"]]
        gpsY = [y - startY for y in theirData["y_signal_preprocess_gps_front_pos_1_"]]
        idx = -1
        for i in range(len(gpsX)):
            if abs(gpsX[i] - gpsX[0]) > 1e-2 and abs(gpsY[i] - gpsY[0]) > 1e-2:
                idx = i
                break

        xb = gpsX[idx] - gpsX[0]
        yb = gpsY[idx] - gpsY[0]
        mag = math.sqrt(xb * xb + yb * yb)
        xb /= mag
        yb /= mag

        mat = np.matrix(
            [
                [xa * xb + ya * yb, xb * ya - xa * yb],
                [xa * yb - xb * ya, xa * xb + ya * yb],
            ]
        )

        xNew = []
        yNew = []
        for i in range(len(ourData["pos1"])):
            v1 = np.array([ourData["pos1"][i], ourData["pos2"][i]])
            rot = np.matmul(mat, v1)
            xNew.append(rot[0, 0])
            yNew.append(rot[0, 1])
        ourData["pos1"] = xNew
        ourData["pos2"] = yNew

        return ourData

    def saveBothTrajectories(self, dTheirs, dOurs, theirData):
        sns.set_theme()  # Let seaborn apply better styling to all matplotlib graphs
        # for flipping the plot in x and/or y.
        dff = [x * 1 for x in dOurs[0]]
        eff = [x * 1 for x in dOurs[1]]
        fig, ax = plt.subplots(figsize=(15, 15))

        # Compute appropriate axis limits for the plot.
        ourMinX = 1e24
        ourMaxX = -1e24
        ourMinY = 1e24
        ourMaxY = -1e24
        for i in range(len(dOurs[0])):
            ourMaxX = max(ourMaxX, dOurs[0][i])
            ourMinX = min(ourMinX, dOurs[0][i])
            ourMaxY = max(ourMaxY, dOurs[1][i])
            ourMinY = min(ourMinY, dOurs[1][i])

        theirMinX = 1e24
        theirMaxX = -1e24
        theirMinY = 1e24
        theirMaxY = -1e24
        for i in range(len(dTheirs[0])):
            theirMaxX = max(theirMaxX, dTheirs[0][i].x)
            theirMinX = min(theirMinX, dTheirs[0][i].x)
            theirMaxY = max(theirMaxY, dTheirs[0][i].y)
            theirMinY = min(theirMinY, dTheirs[0][i].y)

        axisMinX = min(ourMinX, theirMinX)
        axisMinY = min(ourMinY, theirMinY)
        finalMin = min(axisMinX, axisMinY)
        axisMaxX = max(ourMaxX, theirMaxX)
        axisMaxY = max(ourMaxY, theirMaxY)
        finalMax = max(axisMaxX, axisMaxY)
        margin = (finalMax - finalMin) * 0.1

        plt.xlim([finalMin - margin, finalMax + margin])
        plt.ylim([finalMin - margin, finalMax + margin])
        xx = []
        yy = []
        for i in range(len(dTheirs[0])):
            xx.append(dTheirs[0][i].x)
            yy.append(dTheirs[0][i].y)
        ax.plot(xx, yy, "-b", label="theirs")
        ax.plot(dff, eff, "-g", label="ours")
        ax.set_title("Trajectory")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.legend(loc="upper right")

        # PLOT DIRECTION DATA.
        dir = []
        gps = []
        markers = self.computeMarkers(theirData["y_signal_preprocess_gps_front_pos_0_"])
        o = vec3(
            theirData["y_signal_preprocess_gps_front_pos_0_"][0],
            theirData["y_signal_preprocess_gps_front_pos_1_"][0],
            theirData["y_signal_preprocess_gps_front_pos_2_"][0],
        )
        for i in range(len(markers)):
            idx = markers[i]
            dir.append(theirData["dir"][idx])
            gps.append(
                vec3(
                    theirData["y_signal_preprocess_gps_front_pos_0_"][idx] - o.x,
                    theirData["y_signal_preprocess_gps_front_pos_1_"][idx] - o.y,
                    theirData["y_signal_preprocess_gps_front_pos_2_"][idx] - o.z,
                )
            )

        # COMPUTE THE VELOCITIES.
        vel = []
        for i in range(len(gps) - 1):
            vel.append(gps[i + 1] - gps[i])
        vel.append(vec3(0, 0, 0))

        for i in range(len(dir)):
            x = gps[i].x
            y = gps[i].y
            # plt.plot(x, y,'ro')                                                                         # Silence to remove GPS markers
            # label = theirData['t'][i]
            # plt.annotate(label, (x, y), textcoords="offset points", xytext=(0, 10), ha='center')       # Silence this line to remove numbers from plot.
            dx = dir[i].x
            dy = dir[i].y
            # plt.arrow(x, y, dx, dy, width = 0.05, ec = 'green')                                         # Silence to remove dir arrows.

            vx = vel[i].x
            vy = vel[i].y
            # plt.arrow(x, y, vx, vy, width = 0.05, ec = 'red')                                           # Silence to remove velocities.

        ax.locator_params(axis="x", nbins=30)
        ax.locator_params(axis="y", nbins=30)
        plt.savefig("trajectory.pdf")

    def plotSTWAMapping(self, xx, theirSTWAFL, ourSTWAFL, theirSTWAFR, ourSTWAFR):
        sns.set_theme()  # Let seaborn apply better styling to all matplotlib graphs
        fig, ax = plt.subplots(2, 1, figsize=(15, 15))

        ax[0].plot(xx, theirSTWAFL, "-b", label="Theirs")
        ax[0].plot(xx, ourSTWAFL, "-r", label="Ours")
        ax[0].set_title("Wheel: FL")
        ax[0].set_ylabel("rad")
        ax[0].legend(loc="upper right")

        ax[1].plot(xx, theirSTWAFR, "-b", label="Theirs")
        ax[1].plot(xx, ourSTWAFR, "-r", label="Ours")
        ax[1].set_title("Wheel: FR")
        ax[1].set_ylabel("rad")
        ax[1].legend(loc="upper right")

        plt.show()

    def saveData(self, dTheirs, dOurs):
        sns.set_theme()  # Let seaborn apply better styling to all matplotlib graphs
        tTheirs = dTheirs["t"]
        tOurs = dOurs["t"]

        # Elevation of their trajectory plot.
        fig, ax = plt.subplots(figsize=(15, 15))
        ax.plot(
            tTheirs, np.deg2rad(dTheirs["y_signal_preprocess_gps_front_pos_2_"]), "-b"
        )
        ax.set_title("Elevation of Their Data")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("m")
        ax.locator_params(axis="x", nbins=25)
        ax.locator_params(axis="y", nbins=25)
        plt.savefig("elevation.pdf")

        # STWA.
        ddd = [x * -1 for x in dOurs["STWA"]]
        fig, ax = plt.subplots(figsize=(15, 15))
        ax.plot(
            tTheirs,
            np.deg2rad(dTheirs["y_signal_preprocess_steering_stwa_rad"]),
            "-b",
            label="theirs",
        )
        ax.plot(tOurs, ddd, "-r", label="ours")
        ax.legend(loc="upper right")
        ax.set_title("STWA")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("rad")
        ax.locator_params(axis="x", nbins=25)
        ax.locator_params(axis="y", nbins=25)
        plt.savefig("stwa.pdf")

        # DRIVE TORQUES:
        fig, ax = plt.subplots(2, 2, sharey=True, figsize=(15, 15))
        ax[0, 0].locator_params(axis="x", nbins=15)
        ax[0, 0].locator_params(axis="y", nbins=15)
        ax[0, 0].plot(
            tTheirs,
            dTheirs["y_signal_preprocess_drv_wh_trq_drv_0_"],
            "-b",
            label="theirs",
        )
        ax[0, 0].plot(tOurs, dOurs["Wh1DrvTorque"], "-r", label="ours")
        ax[0, 0].legend(loc="upper right")
        ax[0, 0].set_title("Drive Torque: Wheel 1")
        ax[0, 0].set_xlabel("t (s)")
        ax[0, 0].set_ylabel("N-m")

        ax[0, 1].locator_params(axis="x", nbins=15)
        ax[0, 1].locator_params(axis="y", nbins=15)
        ax[0, 1].plot(
            tTheirs,
            dTheirs["y_signal_preprocess_drv_wh_trq_drv_1_"],
            "-b",
            label="theirs",
        )
        ax[0, 1].plot(tOurs, dOurs["Wh2DrvTorque"], "-r", label="ours")
        ax[0, 1].legend(loc="upper right")
        ax[0, 1].set_title("Drive Torque: Wheel 2")
        ax[0, 1].set_xlabel("t (s)")
        ax[0, 1].set_ylabel("N-m")

        ax[1, 0].locator_params(axis="x", nbins=15)
        ax[1, 0].locator_params(axis="y", nbins=15)
        ax[1, 0].plot(
            tTheirs,
            dTheirs["y_signal_preprocess_drv_wh_trq_drv_2_"],
            "-b",
            label="theirs",
        )
        ax[1, 0].plot(tOurs, dOurs["Wh3DrvTorque"], "-r", label="ours")
        ax[1, 0].legend(loc="upper right")
        ax[1, 0].set_title("Drive Torque: Wheel 3")
        ax[1, 0].set_xlabel("t (s)")
        ax[1, 0].set_ylabel("N-m")

        ax[1, 1].locator_params(axis="x", nbins=15)
        ax[1, 1].locator_params(axis="y", nbins=15)
        ax[1, 1].plot(
            tTheirs,
            dTheirs["y_signal_preprocess_drv_wh_trq_drv_3_"],
            "-b",
            label="theirs",
        )
        ax[1, 1].plot(tOurs, dOurs["Wh4DrvTorque"], "-r", label="ours")
        ax[1, 1].legend(loc="upper right")
        ax[1, 1].set_title("Drive Torque: Wheel 4")
        ax[1, 1].set_xlabel("t (s)")
        ax[1, 1].set_ylabel("N-m")

        plt.savefig("drive_torques.pdf")

        # BRAKE TORQUES:
        fig, ax = plt.subplots(2, 2, sharey=True, figsize=(15, 15))
        ax[0, 0].locator_params(axis="x", nbins=15)
        ax[0, 0].locator_params(axis="y", nbins=15)
        ax[0, 0].plot(
            tTheirs,
            dTheirs["y_signal_preprocess_brk_wh_trq_brk_0_"],
            "-b",
            label="theirs",
        )
        ax[0, 0].plot(tOurs, dOurs["Wh1BrkTorque"], "-r", label="ours")
        ax[0, 0].legend(loc="upper right")
        ax[0, 0].set_title("Brake Torque: Wheel 1")
        ax[0, 0].set_xlabel("t (s)")
        ax[0, 0].set_ylabel("N-m")

        ax[0, 1].locator_params(axis="x", nbins=15)
        ax[0, 1].locator_params(axis="y", nbins=15)
        ax[0, 1].plot(
            tTheirs,
            dTheirs["y_signal_preprocess_brk_wh_trq_brk_1_"],
            "-b",
            label="theirs",
        )
        ax[0, 1].plot(tOurs, dOurs["Wh2BrkTorque"], "-r", label="ours")
        ax[0, 1].legend(loc="upper right")
        ax[0, 1].set_title("Brake Torque: Wheel 2")
        ax[0, 1].set_xlabel("t (s)")
        ax[0, 1].set_ylabel("N-m")

        ax[1, 0].locator_params(axis="x", nbins=15)
        ax[1, 0].locator_params(axis="y", nbins=15)
        ax[1, 0].plot(
            tTheirs,
            dTheirs["y_signal_preprocess_brk_wh_trq_brk_2_"],
            "-b",
            label="theirs",
        )
        ax[1, 0].plot(tOurs, dOurs["Wh3BrkTorque"], "-r", label="ours")
        ax[1, 0].legend(loc="upper right")
        ax[1, 0].set_title("Brake Torque: Wheel 3")
        ax[1, 0].set_xlabel("t (s)")
        ax[1, 0].set_ylabel("N-m")

        ax[1, 1].locator_params(axis="x", nbins=15)
        ax[1, 1].locator_params(axis="y", nbins=15)
        ax[1, 1].plot(
            tTheirs,
            dTheirs["y_signal_preprocess_brk_wh_trq_brk_3_"],
            "-b",
            label="theirs",
        )
        ax[1, 1].plot(tOurs, dOurs["Wh4BrkTorque"], "-r", label="ours")
        ax[1, 1].legend(loc="upper right")
        ax[1, 1].set_title("Brake Torque: Wheel 4")
        ax[1, 1].set_xlabel("t (s)")
        ax[1, 1].set_ylabel("N-m")

        plt.savefig("brake_torques.pdf")

        # SMOOTHED ACCELERATION.
        fig, ax = plt.subplots(3, 1, sharey=True, figsize=(15, 15))
        ax[0].locator_params(axis="x", nbins=38)
        ax[1].locator_params(axis="x", nbins=38)
        ax[2].locator_params(axis="x", nbins=38)

        ax[0].plot(tTheirs, dTheirs["acc_x_smoothed_both_ways"], "-b", label="theirs")
        # ax[0].plot(tOurs, dOurs['IMUAccelSmooth0'], "-r", label="ours")
        # ax[0].plot(tOurs, dOurs['IMUAccelRaw0'], "-r", label="ours")
        ax[0].plot(tOurs, dOurs["IMUAccelPostSmoothX"], "-r", label="ours")
        ax[0].set_title("Acceleration - Front (smoothed)")
        ax[0].set_ylabel("ms^-2")
        ax[0].legend(loc="upper right")

        ax[1].plot(tTheirs, dTheirs["acc_y_smoothed_both_ways"], "-b", label="theirs")
        # ax[1].plot(tOurs, dOurs['IMUAccelSmooth1'], "-r", label="ours")
        # ax[1].plot(tOurs, dOurs['IMUAccelRaw1'], "-r", label="ours")
        ax[1].plot(tOurs, dOurs["IMUAccelPostSmoothY"], "-r", label="ours")
        ax[1].set_title("Acceleration - Right (smoothed)")
        ax[1].set_ylabel("ms^-2")
        ax[1].legend(loc="upper right")

        ax[2].plot(tTheirs, dTheirs["acc_z_smoothed_both_ways"], "-b", label="theirs")
        # ax[2].plot(tOurs, dOurs['IMUAccelSmooth2'], "-r", label="ours")
        # ax[2].plot(tOurs, dOurs['IMUAccelRaw2'], "-r", label="ours")
        ax[2].plot(tOurs, dOurs["IMUAccelPostSmoothZ"], "-r", label="ours")
        ax[2].set_title("Acceleration- Up (smoothed)")
        ax[2].set_ylabel("ms^-2")
        ax[2].set_xlabel("t (s)")
        ax[2].legend(loc="upper right")

        plt.savefig("acceleration.pdf")

        # SMOOTHED GYROSCOPIC DATA.
        fig, ax = plt.subplots(3, 1, sharey=True, figsize=(15, 15))
        ax[0].locator_params(axis="x", nbins=38)
        ax[1].locator_params(axis="x", nbins=38)
        ax[2].locator_params(axis="x", nbins=38)

        ax[0].plot(tTheirs, dTheirs["gyro_x_smoothed_both_ways"], "-b", label="theirs")
        # ax[0].plot(tOurs, dOurs['IMUAngVelSmoothX'], "-r", label="ours")
        # ax[0].plot(tOurs, dOurs['IMUAngVelX'], "-r", label="ours")
        ax[0].plot(tOurs, dOurs["IMUAngVelPostSmoothX"], "-r", label="ours")
        ax[0].set_title("Gyroscopic - Roll [front]")
        ax[0].set_ylabel("rad/s")
        ax[0].legend(loc="upper right")

        ax[1].plot(tTheirs, dTheirs["gyro_y_smoothed_both_ways"], "-b", label="theirs")
        # ax[1].plot(tOurs, dOurs['IMUAngVelSmoothY'], "-r", label="ours")
        # ax[1].plot(tOurs, dOurs['IMUAngVelY'], "-r", label="ours")
        ax[1].plot(tOurs, dOurs["IMUAngVelPostSmoothY"], "-r", label="ours")
        ax[1].set_title("Gyroscopic - Pitch [right]")
        ax[1].set_ylabel("rad/s")
        ax[1].legend(loc="upper right")

        ax[2].plot(tTheirs, dTheirs["gyro_z_smoothed_both_ways"], "-b", label="theirs")
        # ax[2].plot(tOurs, dOurs['IMUAngVelSmoothZ'], "-r", label="ours")
        # ax[2].plot(tOurs, dOurs['IMUAngVelZ'], "-r", label="ours")
        ax[2].plot(tOurs, dOurs["IMUAngVelPostSmoothZ"], "-r", label="ours")
        ax[2].set_title("Gyroscopic - Yaw [up]")
        ax[2].set_ylabel("rad/s")
        ax[2].set_xlabel("t (s)")
        ax[2].legend(loc="upper right")

        plt.savefig("gyroscopic.pdf")

        # WHEEL SPEEDS:
        fig, ax = plt.subplots(2, 2, sharey=True, figsize=(15, 15))
        ax[0, 0].locator_params(axis="x", nbins=15)
        ax[0, 0].locator_params(axis="y", nbins=15)
        ax[0, 0].plot(tTheirs, dTheirs["wh_spd_kph_0"], "-b", label="theirs")
        # ax[0, 0].plot(tOurs, dOurs['Wh1Speed'], "-r", label="ours")
        ax[0, 0].plot(tOurs, dOurs["Wh1SpeedSmooth"], "-r", label="ours")
        ax[0, 0].set_title("Wheel Speed 1")
        ax[0, 0].set_xlabel("t (s)")
        ax[0, 0].set_ylabel("kmph")
        ax[0, 0].legend(loc="upper right")

        ax[0, 1].locator_params(axis="x", nbins=15)
        ax[0, 1].locator_params(axis="y", nbins=15)
        ax[0, 1].plot(tTheirs, dTheirs["wh_spd_kph_1"], "-b", label="theirs")
        # ax[0, 1].plot(tOurs, dOurs['Wh2Speed'], "-r", label="ours")
        ax[0, 1].plot(tOurs, dOurs["Wh2SpeedSmooth"], "-r", label="ours")
        ax[0, 1].set_title("Wheel Speed 2")
        ax[0, 1].set_xlabel("t (s)")
        ax[0, 1].set_ylabel("kmph")
        ax[0, 1].legend(loc="upper right")

        ax[1, 0].locator_params(axis="x", nbins=15)
        ax[1, 0].locator_params(axis="y", nbins=15)
        ax[1, 0].plot(tTheirs, dTheirs["wh_spd_kph_2"], "-b", label="theirs")
        # ax[1, 0].plot(tOurs, dOurs['Wh3Speed'], "-r", label="ours")
        ax[1, 0].plot(tOurs, dOurs["Wh3SpeedSmooth"], "-r", label="ours")
        ax[1, 0].set_title("Wheel Speed 3")
        ax[1, 0].set_xlabel("t (s)")
        ax[1, 0].set_ylabel("kmph")
        ax[1, 0].legend(loc="upper right")

        ax[1, 1].locator_params(axis="x", nbins=15)
        ax[1, 1].locator_params(axis="y", nbins=15)
        ax[1, 1].plot(tTheirs, dTheirs["wh_spd_kph_3"], "-b", label="theirs")
        # ax[1, 1].plot(tOurs, dOurs['Wh4Speed'], "-r", label="ours")
        ax[1, 1].plot(tOurs, dOurs["Wh4SpeedSmooth"], "-r", label="ours")
        ax[1, 1].set_title("Wheel Speed 4")
        ax[1, 1].set_xlabel("t (s)")
        ax[1, 1].set_ylabel("kmph")
        ax[1, 1].legend(loc="upper right")

        plt.savefig("wheel_speeds.pdf")

        # WHEELS ANGULAR VELOCITY:
        fig, ax = plt.subplots(2, 2, sharey=True, figsize=(15, 15))
        ax[0, 0].locator_params(axis="x", nbins=15)
        ax[0, 0].locator_params(axis="y", nbins=15)
        # ax[0, 0].plot(tTheirs, dTheirs['y_signal_preprocess_signals_wh_spd_0_'], "-b", label="theirs")
        ax[0, 0].plot(tTheirs, dTheirs["wh_spd_kph_smooth_0"], "-b", label="theirs")
        # ax[0, 0].plot(tOurs, dOurs['Wh1AngVel'], "-r", label="ours")
        ax[0, 0].plot(tOurs, dOurs["Wh1AngVelSmooth"], "-r", label="ours")
        ax[0, 0].set_title("Wheel Ang Vel 1")
        ax[0, 0].set_xlabel("t (s)")
        ax[0, 0].set_ylabel("rad/s")
        ax[0, 0].legend(loc="upper right")

        ax[0, 1].locator_params(axis="x", nbins=15)
        ax[0, 1].locator_params(axis="y", nbins=15)
        # ax[0, 1].plot(tTheirs, dTheirs['y_signal_preprocess_signals_wh_spd_1_'], "-b", label="theirs")
        ax[0, 1].plot(tTheirs, dTheirs["wh_spd_kph_smooth_1"], "-b", label="theirs")
        # ax[0, 1].plot(tOurs, dOurs['Wh2AngVel'], "-r", label="ours")
        ax[0, 1].plot(tOurs, dOurs["Wh2AngVelSmooth"], "-r", label="ours")
        ax[0, 1].set_title("Wheel Ang Vel 2")
        ax[0, 1].set_xlabel("t (s)")
        ax[0, 1].set_ylabel("rad/s")
        ax[0, 1].legend(loc="upper right")

        ax[1, 0].locator_params(axis="x", nbins=15)
        ax[1, 0].locator_params(axis="y", nbins=15)
        # ax[1, 0].plot(tTheirs, dTheirs['y_signal_preprocess_signals_wh_spd_2_'], "-b", label="theirs")
        ax[1, 0].plot(tTheirs, dTheirs["wh_spd_kph_smooth_2"], "-b", label="theirs")
        # ax[1, 0].plot(tOurs, dOurs['Wh3AngVel'], "-r", label="ours")
        ax[1, 0].plot(tOurs, dOurs["Wh3AngVelSmooth"], "-r", label="ours")
        ax[1, 0].set_title("Wheel Ang Vel 3")
        ax[1, 0].set_xlabel("t (s)")
        ax[1, 0].set_ylabel("rad/s")
        ax[1, 0].legend(loc="upper right")

        ax[1, 1].locator_params(axis="x", nbins=15)
        ax[1, 1].locator_params(axis="y", nbins=15)
        # ax[1, 1].plot(tTheirs, dTheirs['y_signal_preprocess_signals_wh_spd_3_'], "-b", label="theirs")
        ax[1, 1].plot(tTheirs, dTheirs["wh_spd_kph_smooth_3"], "-b", label="theirs")
        # ax[1, 1].plot(tOurs, dOurs['Wh4AngVel'], "-r", label="ours")
        ax[1, 1].plot(tOurs, dOurs["Wh4AngVelSmooth"], "-r", label="ours")
        ax[1, 1].set_title("Wheel Ang Vel 4")
        ax[1, 1].set_xlabel("t (s)")
        ax[1, 1].set_ylabel("rad/s")
        ax[1, 1].legend(loc="upper right")

        plt.savefig("wheel_angular_velocities.pdf")

    def _readData(self, filename):
        with open(filename, newline="") as csvfile:
            data = list(csv.reader(csvfile))
        d = {}
        for i in range(len(data[0])):
            key = data[0][i]
            val = []
            for j in range(1, len(data)):  # Convert the column to a list of floats.
                val.append(float(data[j][i]))
            d[key] = val

        return d

    def _readData4Column(self, filename):
        with open(filename, newline="") as csvfile:
            data = list(csv.reader(csvfile))
        d = {}
        for i in range(len(data[0]) - 1):
            key = data[0][i]
            val = []
            for j in range(1, len(data)):
                val.append(float(data[j][i]))
            d[key] = val

        return d

    def _getSTWAFromRWA(self, rwa, d, rwa_string, stwa_string):

        floorIdx = None  # find the floor number
        bestYet = 1e12
        for i in range(len(d[rwa_string])):
            if d[rwa_string][i] > rwa:
                continue
            if rwa - d[rwa_string][i] < bestYet:
                bestYet = rwa - d[rwa_string][i]
                floorIdx = i

        ceilIdx = None  # find the ceiling number
        bestYet = 1e12
        for i in range(len(d[rwa_string])):
            if d[rwa_string][i] < rwa:
                continue
            if d[rwa_string][i] - rwa < bestYet:
                bestYet = d[rwa_string][i] - rwa
                ceilIdx = i

        if floorIdx == None or ceilIdx == None:
            return 0.0

        floorRWA = d[rwa_string][floorIdx]
        ceilRWA = d[rwa_string][ceilIdx]
        floorSTWA = d[stwa_string][floorIdx]
        ceilSTWA = d[stwa_string][ceilIdx]

        if ceilRWA - floorRWA < 1e-7:
            return floorSTWA

        ratio = (rwa - floorRWA) / (ceilRWA - floorRWA)
        return floorSTWA + ratio * (ceilSTWA - floorSTWA)

    def compare(self, test_string, beamng_folder_path):

        ADataFilename = "f_veh_ident_exp_" + test_string + ".csv"
        BDataFilename = beamng_folder_path + "/0.29/simulation_data.csv"

        # for n in range(1, 10):
        #    fname = "f_veh_ident_exp_00"+str(n)+".csv"
        #    print("running: " + fname)
        #    theirData = readData(fname)
        #    theirTrajectory = tj.getTrajectoryFromGPS(theirData)
        #    pt.plotTheirTrajectories(theirTrajectory, n)

        # Produce steering mapping plots.
        # theirSTWAMap = readData('lotus_steering.csv')
        # ourSTWAMap = readData4Column('stwa_rwa_data.csv')
        # xx = []
        # theirSTWAFL = []
        # ourSTWAFL = []
        # for i in range(1000):
        #    rwa = (i - 500) / 500
        #    xx.append(rwa)
        #    theirSTWAFL.append(getSTWAFromRWA(rwa, theirSTWAMap, 'rwa_FL', 'stwa'))
        #    ourSTWAFL.append(getSTWAFromRWA(rwa, ourSTWAMap, 'FR', 'stwa') * -9.25024503556995)
        # theirSTWAFR = []
        # ourSTWAFR = []
        # for i in range(1000):
        #    rwa = (i - 500) / 500
        #    theirSTWAFR.append(getSTWAFromRWA(rwa, theirSTWAMap, 'rwa_FR', 'stwa'))
        #    ourSTWAFR.append(getSTWAFromRWA(rwa, ourSTWAMap, 'FL', 'stwa') * -9.25024503556995)
        # pt.plotSTWAMapping(xx, theirSTWAFL, ourSTWAFL, theirSTWAFR, ourSTWAFR)

        # Compute and process their data.
        theirData = self._readData(ADataFilename)
        theirData["acc_x_smoothed_both_ways"] = self.smoothTemporalBothWays(
            theirData["y_UDP_RX_udp_rx_IMU_1_acc_x"], theirData["t"], 35
        )
        theirData["acc_y_smoothed_both_ways"] = self.smoothTemporalBothWays(
            theirData["y_UDP_RX_udp_rx_IMU_1_acc_y"], theirData["t"], 35
        )
        theirData["acc_z_smoothed_both_ways"] = self.smoothTemporalBothWays(
            theirData["y_UDP_RX_udp_rx_IMU_1_acc_z"], theirData["t"], 35
        )
        theirData["gyro_x_smoothed_both_ways"] = self.smoothTemporalBothWays(
            theirData["y_UDP_RX_udp_rx_IMU_1_gyro_x"], theirData["t"], 10
        )
        theirData["gyro_y_smoothed_both_ways"] = self.smoothTemporalBothWays(
            theirData["y_UDP_RX_udp_rx_IMU_1_gyro_y"], theirData["t"], 10
        )
        theirData["gyro_z_smoothed_both_ways"] = self.smoothTemporalBothWays(
            theirData["y_UDP_RX_udp_rx_IMU_1_gyro_z"], theirData["t"], 10
        )

        theirData["wh_spd_kph_smooth_0"] = self.smoothTemporalBothWays(
            theirData["y_UDP_RX_udp_rx_WH_SPD_1_spd"], theirData["t"], 35
        )
        theirData["wh_spd_kph_smooth_1"] = self.smoothTemporalBothWays(
            theirData["y_UDP_RX_udp_rx_WH_SPD_2_spd"], theirData["t"], 35
        )
        theirData["wh_spd_kph_smooth_2"] = self.smoothTemporalBothWays(
            theirData["y_UDP_RX_udp_rx_WH_SPD_3_spd"], theirData["t"], 35
        )
        theirData["wh_spd_kph_smooth_3"] = self.smoothTemporalBothWays(
            theirData["y_UDP_RX_udp_rx_WH_SPD_4_spd"], theirData["t"], 35
        )

        (
            theirData["wh_spd_kph_0"],
            theirData["wh_spd_kph_1"],
            theirData["wh_spd_kph_2"],
            theirData["wh_spd_kph_3"],
        ) = ([], [], [], [])
        for i in range(
            len(theirData["wh_spd_kph_smooth_0"])
        ):  # convert wheel speeds from rad/s to kph.
            theirData["wh_spd_kph_0"].append(
                theirData["wh_spd_kph_smooth_0"][i] * 3.6 * 0.3185
            )  # kph
        for i in range(len(theirData["wh_spd_kph_smooth_1"])):
            theirData["wh_spd_kph_1"].append(
                theirData["wh_spd_kph_smooth_1"][i] * 3.6 * 0.3185
            )  # kph
        for i in range(len(theirData["wh_spd_kph_smooth_2"])):
            theirData["wh_spd_kph_2"].append(
                theirData["wh_spd_kph_smooth_2"][i] * 3.6 * 0.3305
            )  # kph
        for i in range(len(theirData["wh_spd_kph_smooth_3"])):
            theirData["wh_spd_kph_3"].append(
                theirData["wh_spd_kph_smooth_3"][i] * 3.6 * 0.3305
            )  # kph

        dir = []
        for i in range(len(theirData["t"])):
            dx = (
                theirData["y_signal_preprocess_gps_front_pos_0_"][i]
                - theirData["y_signal_preprocess_gps_rear_pos_0_"][i]
            )
            dy = (
                theirData["y_signal_preprocess_gps_front_pos_1_"][i]
                - theirData["y_signal_preprocess_gps_rear_pos_1_"][i]
            )
            dz = (
                theirData["y_signal_preprocess_gps_front_pos_2_"][i]
                - theirData["y_signal_preprocess_gps_rear_pos_2_"][i]
            )
            dir.append(vec3(dx, dy, dz))
        theirData["dir"] = dir

        # Compute and process our data.
        ourDataUnoriented = self._readData(BDataFilename)
        ourData = self.orientOurData(ourDataUnoriented, theirData)
        ourData["IMUAccelPostSmoothX"] = self.smoothTemporalBothWays(
            ourData["IMUAccelRaw0"], ourData["t"], 35
        )
        ourData["IMUAccelPostSmoothY"] = self.smoothTemporalBothWays(
            ourData["IMUAccelRaw1"], ourData["t"], 35
        )
        ourData["IMUAccelPostSmoothZ"] = self.smoothTemporalBothWays(
            ourData["IMUAccelRaw2"], ourData["t"], 35
        )
        ourData["IMUAngVelPostSmoothX"] = self.smoothTemporalBothWays(
            ourData["IMUAngVelX"], ourData["t"], 35
        )
        ourData["IMUAngVelPostSmoothY"] = self.smoothTemporalBothWays(
            ourData["IMUAngVelY"], ourData["t"], 35
        )
        ourData["IMUAngVelPostSmoothZ"] = self.smoothTemporalBothWays(
            ourData["IMUAngVelZ"], ourData["t"], 35
        )

        ourData["Wh1SpeedSmooth"] = self.smoothTemporalBothWays(
            ourData["Wh1Speed"], ourData["t"], 35
        )
        ourData["Wh2SpeedSmooth"] = self.smoothTemporalBothWays(
            ourData["Wh2Speed"], ourData["t"], 35
        )
        ourData["Wh3SpeedSmooth"] = self.smoothTemporalBothWays(
            ourData["Wh3Speed"], ourData["t"], 35
        )
        ourData["Wh4SpeedSmooth"] = self.smoothTemporalBothWays(
            ourData["Wh4Speed"], ourData["t"], 35
        )

        ourData["Wh1AngVelSmooth"] = self.smoothTemporalBothWays(
            ourData["Wh1AngVel"], ourData["t"], 35
        )
        ourData["Wh2AngVelSmooth"] = self.smoothTemporalBothWays(
            ourData["Wh2AngVel"], ourData["t"], 35
        )
        ourData["Wh3AngVelSmooth"] = self.smoothTemporalBothWays(
            ourData["Wh3AngVel"], ourData["t"], 35
        )
        ourData["Wh4AngVelSmooth"] = self.smoothTemporalBothWays(
            ourData["Wh4AngVel"], ourData["t"], 35
        )

        # traj_smooth = tj.getTrajectoryBasic(theirData)
        # t_len = tj.getTrajectoryLength(traj_smooth)
        # traj_smooth2 = tj.getTrajectoryBasic2(theirData)
        # t_len2 = tj.getTrajectoryLength(traj_smooth2)
        # self.plotSmoothTraj2(traj_smooth, traj_smooth2)

        theirTrajectory = self.getTrajectoryFromGPS(theirData)
        ourTrajectory = self.getTrajectoryFromPos(ourData)

        self.saveBothTrajectories(theirTrajectory, ourTrajectory, theirData)

        self.saveData(theirData, ourData)

        if not pypdf_installed:
            create_warning(
                "The `PyPDF2` package is not installed. PDFs will not be produced. You can install it using `pip install PyPDF2`."
            )
            return

        now = datetime.now()
        dt_string = now.strftime("%d-%m-%Y %H-%M")
        pdf_path = "comparison_" + test_string + "_" + dt_string + ".pdf"

        img = []
        img.append("trajectory.pdf")
        img.append("stwa.pdf")
        img.append("elevation.pdf")
        img.append("acceleration.pdf")
        img.append("gyroscopic.pdf")
        img.append("drive_torques.pdf")
        img.append("wheel_angular_velocities.pdf")
        img.append("wheel_speeds.pdf")

        mergedObject = PdfMerger()
        for j in range(8):
            mergedObject.append(PdfReader(img[j], "rb"))
        mergedObject.write(pdf_path)
        print("pdf created.")
