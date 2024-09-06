from __future__ import annotations

from beamngpy.types import StrDict, Time

from .base import Api


class EnvironmentApi(Api):
    """
    An API allowing control of the in-game environment variables, such as time, weather or gravity.

    Args:
        beamng: An instance of the simulator.
    """

    def get_tod(self) -> StrDict:
        """
        Gets the current 'time of day' object. That is a dictionary with the following keys:

            * ``time``: Time of day on a scale from 0 to 1. 0/1 is midday, 0.5 is midnight.
            * ``timeStr``: Time of day as a string in the format 'HH:MM:SS'.
            * ``nightScale``: How fast should the night be.
            * ``dayScale``: How fast should the day be.
            * ``azimuthOverride``: Used to specify an azimuth that will stay constant throughout the day cycle.
            * ``startTime``: Time of day when the scenario started.
            * ``dayLength``: Length of the day (24 hours).

        Returns:
            The dictionary with keys specified above.
        """
        data: StrDict = dict(type="GetTimeOfDay")
        resp = self._send(data).recv("TimeOfDay")
        return resp["data"]

    def set_tod(
        self,
        tod: Time | None = None,
        play: bool | None = None,
        day_scale: float | None = None,
        night_scale: float | None = None,
        day_length: float | None = None,
        azimuth_override: float | None = None,
    ) -> None:
        """
        Sets the current time of day. The time of day value is given as a float
        between 0 and 1. How this value affects the lighting of the scene is
        dependant on the map's TimeOfDay object.

        Args:
            tod: Time of day. Can be provided as a float between 0.0 and 1.0, or as a string in the format 'HH:MM:SS'.
            play: False by default.
            day_scale: How fast should the day be.
            night_scale: How fast should the night be.
            day_length: Length of the day (24 hours).
            azimuth_override: Used to specify an azimuth that will stay constant throughout the day cycle.
        """
        data: StrDict = dict(
            type="TimeOfDayChange",
            time=tod,
            play=play,
            dayScale=day_scale,
            nightScale=night_scale,
            dayLength=day_length,
            azimuthOverride=azimuth_override,
        )
        self._send(data).ack("TimeOfDayChanged")

    def set_weather_preset(self, preset: str, time: float = 1) -> None:
        """
        Triggers a change to a different weather preset. Weather presets affect
        multiple settings at once (time of day, wind speed, cloud coverage,
        etc.) and need to have been defined first. Example json objects
        defining weather presets can be found in BeamNG.tech's
        ``art/weather/defaults.json`` file.

        Args:
            preset: The name of the preset to switch to. Needs to be
                            defined already within the simulation.
            time: Time in seconds the transition from the current
                            settings to the preset's should take.
        """
        data: StrDict = dict(type="SetWeatherPreset")
        data["preset"] = preset
        data["time"] = time
        self._send(data).ack("WeatherPresetChanged")

    def get_gravity(self) -> float:
        """
        Gets the strength of gravity in the simulator.

        Returns:
            The gravity value of the simulator.
        """
        data: StrDict = dict(type="GetGravity")
        resp = self._send(data).recv("Gravity")
        return resp["gravity"]

    def set_gravity(self, gravity: float = -9.807) -> None:
        """
        Sets the strength of gravity in the simulator.

        Args:
            gravity: The gravity value to set. The default one is that of earth (-9.807).
        """
        data: StrDict = dict(type="SetGravity")
        data["gravity"] = gravity
        self._send(data).ack("GravitySet")
