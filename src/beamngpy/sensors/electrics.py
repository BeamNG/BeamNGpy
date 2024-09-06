from __future__ import annotations

from typing import List

from beamngpy.types import StrDict

from .sensor import Sensor


class Electrics(Sensor):
    """
    This sensor is used to retrieve various values made available by the car's
    eletrics systems. These values include:

    # TODO: List all the electrics.lua values.
    - abs (int): ABS state
    - abs_active (bool):
    - airspeed (float): Airspeed
    - airflowspeed (float):
    - altitude (float): Z axis position
    - avg_wheel_av (float):
    - brake (int): Brake value
    - brake_lights (int):
    - brake_input (int): Brake input value
    - check_engine (bool): Check engine light state.
    - clutch (int): Clutch value
    - clutch_input (int): Clutch input value
    - clutch_ratio (int):
    - driveshaft (float): Driveshaft
    - engine_load (float):
    - engine_throttle (int): Engine throttle state
    - esc (int): ESC state. 0 = not present/inactive, 1 = disabled, Blink = active
    - esc_active (bool):
    - exhaust_flow (float):
    - fog_lights (int): Fog light state
    - fuel (float): Percentage of fuel remaining.
    - fuel_capacity (int): Total Fuel Capacity [L].
    - fuel_volume (float):
    - gear (int):
    - gear_a (int): Gear selected in automatic mode.
    - gear_index (int):
    - gear_m (int): Gear selected in manual mode.
    - hazard (int): Hazard light state
    - hazard_signal (bool):
    - headlights (int):
    - highbeam (int): High beam state
    - horn (int):
    - ignition (bool): Engine state
    - left_signal (bool):
    - lightbar (int): Lightbar state
    - lights (int): General light state. 1 = low, 2 = high
    - lowbeam (int): Low beam state
    - lowfuel (bool): Low fuel indicator
    - lowhighbeam (int): Low-high beam state
    - lowpressure (int): Low fuel pressure indicator
    - oil (int):
    - oil_temperature (float): Oil temperature [C].
    - parking (int): Parking lights on/off (not implemented yet)
    - parkingbrake (float): Parking brake state. 0.5 = halfway on
    - parkingbrake_input (int): Parking brake input state
    - radiator_fan_spin (int):
    - reverse (int): Reverse gear state
    - right_signal (bool):
    - rpm (float): Engine RPM
    - rpmspin (float):
    - rpm_tacho (float):
    - running (bool): Engine running state
    - signal_l (int): Left signal state. 0.5 = halfway to full blink
    - signal_r (int): Right signal state. 0.5 = halfway to full blink
    - steering (float): Angle of the steering wheel in degrees.
    - steering_input (int): Steering input state
    - tcs (int): TCS state. 0 = not present/inactive, 1 = disabled, Blink = active
    - tcs_active (bool):
    - throttle (int): Throttle state
    - throttle_factor (int):
    - throttle_input (int): Throttle input state
    - turnsignal (int): Turn signal value. -1 = Left, 1 = Right,
    gradually 'fades' between values. Use "signal_L" and "signal_R" for flashing indicators.
    - two_step (bool):
    - water_temperature (float): Water temperature [C].
    - wheelspeed (float): Wheel speed [m/s].
    """

    name_map = {
        "absActive": "abs_active",
        "avgWheelAV": "avg_wheel_av",
        "brakelights": "brake_lights",
        "checkengine": "check_engine",
        "clutchRatio": "clutch_ratio",
        "engineLoad": "engine_load",
        "engineThrottle": "engine_throttle",
        "escActive": "esc_active",
        "exhaustFlow": "exhaust_flow",
        "fog": "fog_lights",
        "fuelVolume": "fuel_volume",
        "fuelCapacity": "fuel_capacity",
        "gear_A": "gear_a",
        "gearIndex": "gear_index",
        "gear_M": "gear_m",
        "hazard_enabled": "hazard_signal",
        "isShifting": "is_shifting",
        "lights_state": "headlights",
        "oiltemp": "oil_temperature",
        "radiatorFanSpin": "radiator_fan_spin",
        "rpmTacho": "rpm_tacho",
        "signal_L": "signal_l",
        "signal_left_input": "left_signal",
        "signal_R": "signal_r",
        "signal_right_input": "right_signal",
        "tcsActive": "tcs_active",
        "throttleFactor": "throttle_factor",
        "twoStep": "two_step",
        "watertemp": "water_temperature",
    }

    def __init__(self):
        super().__init__()

    @staticmethod
    def _rename_values(vals: StrDict) -> StrDict:
        """
        The values returned from the simulator often don't follow any naming
        convention and especially don't follow this library's, so we rename
        some of them here to be more consistent.
        """
        for k, v in Electrics.name_map.items():
            if k in vals:
                vals[v] = vals[k]
                del vals[k]
        return vals

    def _reassign_values(self, vals: StrDict) -> StrDict:
        if "left_signal" in vals:
            vals["left_signal"] = vals["left_signal"] == 1
        if "right_signal" in vals:
            vals["right_signal"] = vals["right_signal"] == 1
        if "hazard_signal" in vals:
            vals["hazard_signal"] = vals["hazard_signal"] == 1
        return vals

    def encode_vehicle_request(self):
        req = dict(type="Electrics")
        return req

    def decode_response(self, resp):
        if "values" in resp:
            ret = self._rename_values(resp["values"])
            ret = self._reassign_values(ret)
            return ret
        return None
