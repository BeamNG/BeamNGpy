from __future__ import annotations

from beamngpy.logging import BNGValueError
from beamngpy.misc.colors import coerce_color
from beamngpy.types import Color, Float3, StrDict

from .base import VehicleApi

SHIFT_MODES = {
    "realistic_manual": 0,
    "realistic_manual_auto_clutch": 1,
    "arcade": 2,
    "realistic_automatic": 3,
}


class RootApi(VehicleApi):
    """
    A vehicle API that is exposed at the root level (directly accessible from the :class:`Vehicle` object).

    Args:
        vehicle: An instance of a vehicle object.
    """

    def set_shift_mode(self, mode: str) -> None:
        mode = mode.lower().strip()
        if mode not in SHIFT_MODES:
            raise BNGValueError(f"Non-existent shift mode: {mode}")

        data: StrDict = dict(type="SetShiftMode")
        data["mode"] = SHIFT_MODES[mode]
        self._send(data).ack("ShiftModeSet")

    def control(
        self,
        steering: float | None = None,
        throttle: float | None = None,
        brake: float | None = None,
        parkingbrake: float | None = None,
        clutch: float | None = None,
        gear: int | None = None,
    ) -> None:
        options = {}
        if steering is not None:
            options["steering"] = steering
        if throttle is not None:
            options["throttle"] = throttle
        if brake is not None:
            options["brake"] = brake
        if parkingbrake is not None:
            options["parkingbrake"] = parkingbrake
        if clutch is not None:
            options["clutch"] = clutch
        if gear is not None:
            options["gear"] = gear

        data = dict(type="Control", **options)
        self._send(data).ack("Controlled")

    def set_color(self, rgba: Color = (1.0, 1.0, 1.0, 1.0)) -> None:
        data: StrDict = dict(type="SetColor")
        color = coerce_color(rgba)
        data["r"] = color[0]
        data["g"] = color[1]
        data["b"] = color[2]
        data["a"] = color[3]
        self._send(data).ack("ColorSet")

    def set_velocity(self, velocity: float, dt: float = 1.0) -> None:
        data: StrDict = dict(type="SetVelocity")
        data["velocity"] = velocity
        data["dt"] = dt
        self._send(data).ack("VelocitySet")

    def set_lights(
        self,
        left_signal: bool | None = None,
        right_signal: bool | None = None,
        hazard_signal: bool | None = None,
        headlights: int | None = None,
        fog_lights: int | None = None,
        lightbar: int | None = None,
    ) -> None:
        lights: StrDict = {}

        if left_signal is not None:
            if not isinstance(left_signal, bool):
                raise BNGValueError("Non-boolean value for left_signal.")
            lights["leftSignal"] = left_signal

        if right_signal is not None:
            if not isinstance(right_signal, bool):
                raise BNGValueError("Non-boolean value for right_signal.")
            lights["rightSignal"] = right_signal

        if hazard_signal is not None:
            if not isinstance(hazard_signal, bool):
                raise BNGValueError("Non-boolean value for hazard_signal.")
            lights["hazardSignal"] = hazard_signal

        valid_lights = {0, 1, 2}

        if headlights is not None:
            if not isinstance(headlights, int):
                raise BNGValueError("Non-int value given for headlights.")
            if headlights not in valid_lights:
                msg = (
                    "Invalid value given for headlights, must be "
                    "0, 1, or 2, but was: " + str(headlights)
                )
                raise BNGValueError(msg)
            lights["headLights"] = headlights

        if fog_lights is not None:
            if not isinstance(fog_lights, int):
                raise BNGValueError("Non-int value given for fog lights.")
            if fog_lights not in valid_lights:
                msg = (
                    "Invalid value given for fog lights, must be "
                    "0, 1, or 2, but was: " + str(fog_lights)
                )
                raise BNGValueError(msg)
            lights["fogLights"] = fog_lights

        if lightbar is not None:
            if not isinstance(lightbar, int):
                raise BNGValueError("Non-int value given for lighbar.")
            if lightbar not in valid_lights:
                msg = (
                    "Invalid value given for lightbar, must be "
                    "0, 1, or 2, but was: " + str(lightbar)
                )
                raise BNGValueError(msg)
            lights["lightBar"] = lightbar

        lights["type"] = "SetLights"
        self._send(lights).ack("LightsSet")

    def queue_lua_command(self, chunk: str, response: bool) -> StrDict:
        data = dict(type="QueueLuaCommandVE")
        data["chunk"] = chunk
        data["resp"] = response
        return self._send(data).recv("ExecutedLuaChunkVE").get("resp", None)

    def recover(self) -> None:
        data = dict(type="Recover")
        self._send(data).ack("Recovered")

    def get_center_of_gravity(self, without_wheels: bool) -> Float3:
        data = dict(type="GetCenterOfGravity", withoutWheels=without_wheels)
        return self._send(data).recv()["data"]

    def deflate_tire(self, wheel_id: int) -> None:
        data: StrDict = dict(type="DeflateTire")
        data["wheelId"] = wheel_id
        self._send(data).ack("CompletedDeflateTire")
