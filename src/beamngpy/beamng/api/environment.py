from __future__ import annotations

from beamngpy.types import StrDict

from .base import Api


class EnvironmentApi(Api):
    def set_tod(self, tod: float) -> None:
        """
        Sets the current time of day. The time of day value is given as a float
        between 0 and 1. How this value affects the lighting of the scene is
        dependant on the map's TimeOfDay object.

        Args:
            tod: Time of day beteen 0 and 1.
        """
        data: StrDict = dict(type='TimeOfDayChange')
        data['tod'] = tod
        self.send(data).ack('TimeOfDayChanged')

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
        data: StrDict = dict(type='SetWeatherPreset')
        data['preset'] = preset
        data['time'] = time
        self.send(data).ack('WeatherPresetChanged')

    def set_gravity(self, gravity: float = -9.807) -> None:
        """
        Sets the strength of gravity in the simulator.

        Args:
            gravity: The gravity value to set. The default one is that of earth (-9.807)
        """
        data: StrDict = dict(type='SetGravity')
        data['gravity'] = gravity
        self.send(data).ack('GravitySet')
