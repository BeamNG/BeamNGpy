from __future__ import annotations

from beamngpy.logging import BNGValueError
from beamngpy.types import Float4, StrDict

from .base import VehicleApi

SHIFT_MODES = {
    'realistic_manual': 0,
    'realistic_manual_auto_clutch': 1,
    'arcade': 2,
    'realistic_automatic': 3,
}

class ControlApi(VehicleApi):
    def set_shift_mode(self, mode: str) -> None:
        """
        Sets the shifting mode of the vehicle. This changes whether or not and
        how the vehicle shifts gears depending on the RPM. Available modes are:

         * ``realistic_manual``: Gears have to be shifted manually by the
                                 user, including engaging the clutch.
         * ``realistic_manual_auto_clutch``: Gears have to be shifted manually
                                             by the user, without having to
                                             use the clutch.
         * ``arcade``: Gears shift up and down automatically. If the brake is
                       held, the vehicle automatically shifts into reverse
                       and accelerates backward until brake is released or
                       throttle is engaged.
         * ``realistic_automatic``: Gears shift up automatically, but reverse
                                    and parking need to be shifted to
                                    manually.

        Args:
            mode: The mode to set. Must be a string from the options listed above.

        Raises:
            BNGValueError: If an invalid mode is given.
        """
        mode = mode.lower().strip()
        if mode not in SHIFT_MODES:
            raise BNGValueError(f'Non-existent shift mode: {mode}')

        data: StrDict = dict(type='SetShiftMode')
        data['mode'] = SHIFT_MODES[mode]
        self._send(data).ack('ShiftModeSet')

    def control(self, steering: float | None = None, throttle: float | None = None, brake: float | None = None,
                parkingbrake: float | None = None, clutch: float | None = None, gear: int | None = None) -> None:
        """
        Sends a control message to the vehicle, setting vehicle inputs
        accordingly.

        Args:
            steering: Rotation of the steering wheel, from -1.0 to 1.0.
            throttle: Intensity of the throttle, from 0.0 to 1.0.
            brake: Intensity of the brake, from 0.0 to 1.0.
            parkingbrake: Intensity of the parkingbrake, from 0.0 to 1.0.
            clutch: Clutch level, from 0.0 to 1.0.
            gear: Gear to shift to, -1 eq backwards, 0 eq neutral, 1 to X eq nth gear
        """
        options = {}
        if steering:
            options['steering'] = steering
        if throttle:
            options['throttle'] = throttle
        if brake:
            options['brake'] = brake
        if parkingbrake:
            options['parkingbrake'] = parkingbrake
        if clutch:
            options['clutch'] = clutch
        if gear:
            options['gear'] = gear

        data = dict(type='Control', **options)
        self._send(data).ack('Controlled')

    def set_color(self, rgba: Float4 = (1., 1., 1., 1.)) -> None:
        """
        Sets the color of this vehicle. Colour can be adjusted on the RGB
        spectrum and the "shininess" of the paint.

        Args:
            rgba: The new colour given as a tuple of RGBA floats, where
                  the alpha channel encodes the shininess of the paint.
        """
        data: StrDict = dict(type='SetColor')
        data['r'] = rgba[0]
        data['g'] = rgba[1]
        data['b'] = rgba[2]
        data['a'] = rgba[3]
        self._send(data).ack('ColorSet')

    def set_velocity(self, velocity: float, dt: float = 1.0) -> None:
        """
        Sets the velocity of this vehicle. The velocity is not achieved instantly,
        it is acquired gradually over the time interval set by the `dt` argument.

        As the method of setting velocity uses physical forces, at high velocities
        it is important to set `dt` to an appropriately high value. The default
        `dt` value of 1.0 is suitable for velocities up to 30 m/s.

        Args:
            velocity: The target velocity in m/s.
            dt: The time interval over which the vehicle reaches the target velocity.
                Defaults to 1.0.
        """
        data: StrDict = dict(type='SetVelocity')
        data['velocity'] = velocity
        data['dt'] = dt
        self._send(data).ack('VelocitySet')

    def set_lights(
            self, left_signal: bool | None = None, right_signal: bool | None = None, hazard_signal:
            bool | None = None, headlights: int | None = None, fog_lights: int | None = None,
            lightbar: int | None = None) -> None:
        """
        Sets the vehicle's lights to given intensity values. The lighting
        system features lights that are simply binary on/off, but also ones
        where the intensity can be varied. Binary lights include:

            * `left_signal`
            * `right_signal`
            * `hazard_signal`

        Non-binary lights vary between 0 for off, 1 for on, 2 for higher
        intensity. For example, headlights can be turned on with 1 and set to
        be more intense with 2. Non-binary lights include:

            * `headlights`
            * `fog_lights`
            * `lightbar`

        Args:
            left_signal: On/off state of the left signal
            right_signal: On/off state of the right signal
            hazard_signal: On/off state of the hazard lights
            headlights: Value from 0 to 2 indicating headlight intensity
            fog_lights: Value from 0 to 2 indicating fog light intensity
            lightbar: Value from 0 to 2 indicating lightbar intensity

        Note:
            Not every vehicle has every type of light. For example, the
            `lightbar` refers to the kind of lights typically found on top of
            police cars. Setting values for non-existent lights will not cause
            an error, but also achieve no effect.

            Note also that lights are not independent. For example, turning on
            the hazard lights will make both signal indicators blink, meaning
            they will be turned on as well. Opposing indicators also turn each
            other off, i.e. turning on the left signal turns off the right one,
            and turning on the left signal during

        Raises:
            BNGValueError: If an invalid light value is given.

        Returns:
            Nothing. To query light states, attach an
            :class:`.sensors.Electrics` sensor and poll it.
        """
        lights: StrDict = {}

        if left_signal is not None:
            if not isinstance(left_signal, bool):
                raise BNGValueError('Non-boolean value for left_signal.')
            lights['leftSignal'] = left_signal

        if right_signal is not None:
            if not isinstance(right_signal, bool):
                raise BNGValueError('Non-boolean value for right_signal.')
            lights['rightSignal'] = right_signal

        if hazard_signal is not None:
            if not isinstance(hazard_signal, bool):
                raise BNGValueError('Non-boolean value for hazard_signal.')
            lights['hazardSignal'] = hazard_signal

        valid_lights = {0, 1, 2}

        if headlights is not None:
            if not isinstance(headlights, int):
                raise BNGValueError('Non-int value given for headlights.')
            if headlights not in valid_lights:
                msg = 'Invalid value given for headlights, must be ' \
                      '0, 1, or 2, but was: ' + str(headlights)
                raise BNGValueError(msg)
            lights['headLights'] = headlights

        if fog_lights is not None:
            if not isinstance(fog_lights, int):
                raise BNGValueError('Non-int value given for fog lights.')
            if fog_lights not in valid_lights:
                msg = 'Invalid value given for fog lights, must be ' \
                      '0, 1, or 2, but was: ' + str(fog_lights)
                raise BNGValueError(msg)
            lights['fogLights'] = fog_lights

        if lightbar is not None:
            if not isinstance(lightbar, int):
                raise BNGValueError('Non-int value given for lighbar.')
            if lightbar not in valid_lights:
                msg = 'Invalid value given for lightbar, must be ' \
                      '0, 1, or 2, but was: ' + str(lightbar)
                raise BNGValueError(msg)
            lights['lightBar'] = lightbar

        lights['type'] = 'SetLights'
        self._send(lights).ack('LightsSet')

    def queue_lua_command(self, chunk: str) -> None:
        """
        Executes lua chunk in the vehicle engine VM.

        Args:
            chunk: lua chunk as a string
        """
        data = dict(type='QueueLuaCommandVE')
        data['chunk'] = chunk
        self._send(data).ack('ExecutedLuaChunkVE')
