from __future__ import annotations

from typing import TYPE_CHECKING, Dict

from beamngpy.types import Float3, Quat, StrDict
from beamngpy.vehicle import Vehicle

from .base import Api

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy


class GEVehiclesApi(Api):
    """
    A vehicle API that needs a connected :class:`BeamNGpy` instance.
    It is exposed at the root level (directly accessible from the :class:`Vehicle` object).
    """

    def __init__(self, beamng: BeamNGpy, vehicle: Vehicle):
        super().__init__(beamng)
        self.vehicle = vehicle

    def get_bbox(self) -> Dict[str, Float3]:
        data = dict(type="GetBBoxCorners")
        data["vid"] = self.vehicle.vid
        resp = self._send(data).recv("BBoxCorners")
        points = resp["points"]
        bbox = {
            "front_bottom_left": points[3],
            "front_bottom_right": points[0],
            "front_top_left": points[2],
            "front_top_right": points[1],
            "rear_bottom_left": points[7],
            "rear_bottom_right": points[4],
            "rear_top_left": points[6],
            "rear_top_right": points[5],
        }
        return bbox

    def annotate_parts(self) -> None:
        data = dict(type="AnnotateParts")
        data["vid"] = self.vehicle.vid
        self._send(data).ack("PartsAnnotated")

    def revert_annotations(self) -> None:
        data = dict(type="RevertAnnotations")
        data["vid"] = self.vehicle.vid
        self._send(data).ack("AnnotationsReverted")

    def switch(self):
        return self._beamng.switch_vehicle(self.vehicle)

    def teleport(
        self, pos: Float3, rot_quat: Quat | None = None, reset: bool = True
    ) -> bool:
        return self._beamng.teleport_vehicle(self.vehicle.vid, pos, rot_quat, reset)

    def get_part_options(self) -> StrDict:
        data = dict(type="GetPartOptions")
        data["vid"] = self.vehicle.vid
        resp = self._send(data).recv("PartOptions")
        return resp["options"]

    def get_part_config(self) -> StrDict:
        data = dict(type="GetPartConfig")
        data["vid"] = self.vehicle.vid
        resp = self._send(data).recv("PartConfig")
        resp = resp["config"]
        if "parts" not in resp or not resp["parts"]:
            resp["parts"] = dict()
        if "vars" not in resp or not resp["vars"]:
            resp["vars"] = dict()
        return resp

    def set_part_config(self, cfg: StrDict) -> None:
        data: StrDict = dict(type="SetPartConfig")
        data["vid"] = self.vehicle.vid
        data["config"] = cfg
        self._send(data)
        self._beamng.await_vehicle_spawn(self.vehicle.vid)
        self.vehicle.disconnect()
        self.vehicle.connect(self._beamng)

    def set_license_plate(self, text: str) -> None:
        data: StrDict = dict(type="SetLicensePlate")
        data["vid"] = self.vehicle.vid
        data["text"] = text
        self._send(data).ack("SetLicensePlate")
