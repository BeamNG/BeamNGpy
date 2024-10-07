from __future__ import annotations

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

from beamngpy.connection import CommBase
from beamngpy.logging import LOGGER_ID, BNGError

from .advanced_IMU import AdvancedIMU
from .camera import Camera
from .GPS import GPS
from .ideal_radar import IdealRadar
from .lidar import Lidar
from .mesh import Mesh
from .powertrain_sensor import PowertrainSensor
from .radar import Radar
from .roads_sensor import RoadsSensor
from .ultrasonic import Ultrasonic

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

__all__ = ["VehicleSensorConfig"]


class VehicleSensorConfig(CommBase):
    """
    A class used for managing ADAS vehicle sensor configurations.
    For map-based ADAS sensor configurations (static, fixed-position sensors), see MapSensorConfiguration.py.
    ADAS sensor configurations are produced with the GUI-Based ADAS Sensor Configuration Editor in the BeamNG.Tech World Editor (press F11).
    """

    def __init__(self, name: str, bng: BeamNGpy, vehicle: Vehicle, filepath: str):
        super().__init__(bng, vehicle)
        """
        Imports the ADAS sensor configuration at the given filepath, for the given vehicle.

        Args:
            vehicle: The vehicle to which the ADAS sensor configuration should be attached.
            name: A unique name for the ADAS sensor configuration.
            filepath: The path to the ADAS sensor configuration.
        """
        self.logger = getLogger(f"{LOGGER_ID}.SensorConfig")
        self.logger.setLevel(DEBUG)

        self.name = name
        self.vehicle = vehicle
        self.vid = vehicle.vid
        self.sensors = []

        sData = self.send_recv_ge(
            "UnpackVehicleSensorConfiguration",
            filepath=filepath,
            vid=self.vid,
            name=self.name,
        )["data"]
        for i in range(len(sData)):
            v = sData[i]
            t = v["type"].lower()
            if t == "camera":
                self.sensors.append(
                    Camera(
                        self.name + str(i),
                        self.bng,
                        self.vehicle,
                        requested_update_time=v["updateTime"],
                        update_priority=v["updatePriority"],
                        pos=(v["pos"]["x"], v["pos"]["y"], v["pos"]["z"]),
                        dir=(v["dir"]["x"], v["dir"]["y"], v["dir"]["z"]),
                        up=(v["up"]["x"], v["up"]["y"], v["up"]["z"]),
                        resolution=(v["size"][0], v["size"][1]),
                        field_of_view_y=v["fovY"],
                        near_far_planes=(v["nearFarPlanes"][0], v["nearFarPlanes"][1]),
                        is_using_shared_memory=False,
                        is_streaming=False,
                        is_render_colours=v["isRenderColours"],
                        is_render_annotations=v["isRenderAnnotations"],
                        is_render_instance=v["isRenderInstance"],
                        is_render_depth=v["isRenderDepth"],
                        is_depth_inverted=False,
                        is_visualised=v["isVisualised"],
                        is_dir_world_space=True,
                        is_static=False,
                        is_snapping_desired=v["isSnappingDesired"],
                        is_force_inside_triangle=v["isSnappingDesired"],
                    )
                )
            elif t == "lidar":
                self.sensors.append(
                    Lidar(
                        self.name + str(i),
                        self.bng,
                        self.vehicle,
                        requested_update_time=v["updateTime"],
                        update_priority=v["updatePriority"],
                        pos=(v["pos"]["x"], v["pos"]["y"], v["pos"]["z"]),
                        dir=(v["dir"]["x"], v["dir"]["y"], v["dir"]["z"]),
                        up=(v["up"]["x"], v["up"]["y"], v["up"]["z"]),
                        vertical_resolution=v["verticalResolution"],
                        vertical_angle=v["verticalAngle"],
                        frequency=v["frequency"],
                        horizontal_angle=v["horizontalAngle"],
                        max_distance=v["maxDistance"],
                        is_360_mode=v["is360"],
                        is_rotate_mode=v["isRotate"],
                        is_using_shared_memory=False,
                        is_streaming=False,
                        is_annotated=v["isAnnotated"],
                        is_visualised=v["isVisualised"],
                        is_dir_world_space=True,
                        is_static=False,
                        is_snapping_desired=v["isSnappingDesired"],
                        is_force_inside_triangle=v["isSnappingDesired"],
                    )
                )
            elif t == "ultrasonic":
                self.sensors.append(
                    Ultrasonic(
                        self.name + str(i),
                        self.bng,
                        self.vehicle,
                        requested_update_time=v["updateTime"],
                        update_priority=v["updatePriority"],
                        pos=(v["pos"]["x"], v["pos"]["y"], v["pos"]["z"]),
                        dir=(v["dir"]["x"], v["dir"]["y"], v["dir"]["z"]),
                        up=(v["up"]["x"], v["up"]["y"], v["up"]["z"]),
                        resolution=(v["size"][0], v["size"][1]),
                        field_of_view_y=v["fovY"],
                        near_far_planes=(v["nearFarPlanes"][0], v["nearFarPlanes"][1]),
                        range_roundness=v["rangeRoundness"],
                        range_cutoff_sensitivity=v["rangeCutoffSensitivity"],
                        range_shape=v["rangeShape"],
                        range_focus=v["rangeFocus"],
                        range_min_cutoff=v["rangeMinCutoff"],
                        range_direct_max_cutoff=v["rangeDirectMaxCutoff"],
                        sensitivity=v["sensitivity"],
                        fixed_window_size=v["fixedWindowSize"],
                        is_streaming=False,
                        is_visualised=v["isVisualised"],
                        is_dir_world_space=True,
                        is_static=False,
                        is_snapping_desired=v["isSnappingDesired"],
                        is_force_inside_triangle=v["isSnappingDesired"],
                    )
                )
            elif t == "radar":
                self.sensors.append(
                    Radar(
                        self.name + str(i),
                        self.bng,
                        self.vehicle,
                        requested_update_time=v["updateTime"],
                        update_priority=v["updatePriority"],
                        pos=(v["pos"]["x"], v["pos"]["y"], v["pos"]["z"]),
                        dir=(v["dir"]["x"], v["dir"]["y"], v["dir"]["z"]),
                        up=(v["up"]["x"], v["up"]["y"], v["up"]["z"]),
                        range_bins=v["rangeBins"],
                        azimuth_bins=v["azimuthBins"],
                        vel_bins=v["velBins"],
                        range_min=v["rangeMin"],
                        range_max=v["rangeMax"],
                        vel_min=v["velMin"],
                        vel_max=v["velMax"],
                        half_angle_deg=v["halfAngleDeg"],
                        resolution=(v["size"][0], v["size"][1]),
                        field_of_view_y=v["fovY"],
                        near_far_planes=(v["nearFarPlanes"][0], v["nearFarPlanes"][1]),
                        range_roundness=v["rangeRoundness"],
                        range_cutoff_sensitivity=v["rangeCutoffSensitivity"],
                        range_shape=v["rangeShape"],
                        range_focus=v["rangeFocus"],
                        range_min_cutoff=v["rangeMinCutoff"],
                        range_direct_max_cutoff=v["rangeDirectMaxCutoff"],
                        is_streaming=False,
                        is_visualised=v["isVisualised"],
                        is_dir_world_space=True,
                        is_static=False,
                        is_snapping_desired=v["isSnappingDesired"],
                        is_force_inside_triangle=v["isSnappingDesired"],
                    )
                )
            elif t == "imu":
                self.sensors.append(
                    AdvancedIMU(
                        self.name + str(i),
                        self.bng,
                        self.vehicle,
                        gfx_update_time=v["GFXUpdateTime"],
                        physics_update_time=v["physicsUpdateTime"],
                        pos=(v["pos"]["x"], v["pos"]["y"], v["pos"]["z"]),
                        dir=(v["dir"]["x"], v["dir"]["y"], v["dir"]["z"]),
                        up=(v["up"]["x"], v["up"]["y"], v["up"]["z"]),
                        accel_window_width=v["accelWindowWidth"],
                        gyro_window_width=v["gyroWindowWidth"],
                        accel_frequency_cutoff=None,
                        gyro_frequency_cutoff=None,
                        is_send_immediately=False,
                        is_using_gravity=v["isUsingGravity"],
                        is_allow_wheel_nodes=v["isAllowWheelNodes"],
                        is_visualised=v["isVisualised"],
                        is_dir_world_space=True,
                        is_snapping_desired=v["isSnappingDesired"],
                        is_force_inside_triangle=v["isSnappingDesired"],
                    )
                )
            elif t == "gps":
                self.sensors.append(
                    GPS(
                        self.name + str(i),
                        self.bng,
                        self.vehicle,
                        gfx_update_time=v["GFXUpdateTime"],
                        physics_update_time=v["physicsUpdateTime"],
                        pos=(v["pos"]["x"], v["pos"]["y"], v["pos"]["z"]),
                        ref_lon=v["refLon"],
                        ref_lat=v["refLat"],
                        is_send_immediately=False,
                        is_visualised=v["isVisualised"],
                        is_dir_world_space=True,
                        is_snapping_desired=v["isSnappingDesired"],
                        is_force_inside_triangle=v["isSnappingDesired"],
                    )
                )
            elif t == "idealradar":
                self.sensors.append(
                    IdealRadar(
                        self.name + str(i),
                        self.bng,
                        self.vehicle,
                        is_send_immediately=False,
                        gfx_update_time=v["GFXUpdateTime"],
                        physics_update_time=v["physicsUpdateTime"],
                    )
                )
            elif t == "roads":
                self.sensors.append(
                    RoadsSensor(
                        self.name + str(i),
                        self.bng,
                        self.vehicle,
                        is_send_immediately=False,
                        gfx_update_time=v["GFXUpdateTime"],
                        physics_update_time=v["physicsUpdateTime"],
                    )
                )
            elif t == "powertrain":
                self.sensors.append(
                    PowertrainSensor(
                        self.name + str(i),
                        self.bng,
                        self.vehicle,
                        is_send_immediately=False,
                        gfx_update_time=v["GFXUpdateTime"],
                        physics_update_time=v["physicsUpdateTime"],
                    )
                )
            elif t == "mesh":
                self.sensors.append(
                    Mesh(
                        self.name + str(i),
                        self.bng,
                        self.vehicle,
                        gfx_update_time=v["GFXUpdateTime"],
                        physics_update_time=v["physicsUpdateTime"],
                    )
                )
            else:
                raise BNGError(f"Sensor type '{v['type']}' not found.")
        self.logger.debug(
            "VehicleSensorConfig - sensor configuration imported: " f"{self.name}"
        )

    def remove(self):
        """
        Removes this ADAS sensor configuration.
        """
        for i in range(len(self.sensors)):
            self.sensors[i].remove()
        self.logger.debug(
            "VehicleSensorConfig - sensor configuration removed: " f"{self.name}"
        )
