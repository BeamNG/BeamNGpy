from __future__ import annotations

import threading
import numpy as np
import cv2
# import matplotlib.pyplot as plt  # DEBUG

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

from beamngpy.logging import LOGGER_ID

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

from beamngpy.sensors import Electrics, Camera


class LaneKeepingAssist:
    """
    A camera sensor-based ADAS feature, preventing overspeeding into corners.
    The system uses the road markings to detect the radius of the corner ahead
    of the vehicle and slow down to a safe speed. The feature is only active at speeds
    above 39.6 km/h and when the vehicle's hazard lights and blinkers are not on.
    Optionally, the assistant can also nudge the user's steering wheel to notify
    the driver that the vehicle is exiting or about to exit the lane. This feature
    activates only when the steering angle is less than 45 degrees.

    Usage with a steering wheel controller and pedals is recommended.
    The feature is designed for the ETK800 vehicle, but can be used with other vehicles.

    Args:
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this feature should be attached.
        electrics: The electrics sensor to use for the vehicle. If not provided, a new one will be created.
        risk_level: Controls the maximum allowed cornering speed. 1 is the lowest, safest, 3 is the highest, most dangerous.
        steering_strength: Controls the strength of steering nudge (0-100). Adjust this value according to preference.
        detect_yellow: Whether to detect yellow road markings, in addition to white.
    """

    def __init__(
        self,
        bng: BeamNGpy,
        vehicle: Vehicle,
        electrics: Electrics | None = None,
        risk_level: int = 2,
        steering_strength: float = 15,
        detect_yellow: bool = False
    ):
        self.logger = getLogger(f"{LOGGER_ID}.LaneKeepingAssist")
        self.logger.setLevel(DEBUG)

        self.bng = bng
        self.vehicle = vehicle
        self.electrics = electrics
        self.risk_level = np.clip(risk_level, 1, 3) - 1
        self.running = False
        self.steering_strength = np.clip(steering_strength, 0, 100)
        self.detect_yellow = detect_yellow
        self.last_color_img = None # Added for yellow line detection

    def start(self):
        """
        This method starts the Lane-keeping assist for the given vehicle.
        """
        # 1. Launch thread
        if self.running:
            self.logger.warning("LKA is already running.")
            return

        if self.electrics is None:
            self.electrics = Electrics()
            self.vehicle.attach_sensor('electrics', self.electrics)
            self.electrics.attach(self.vehicle, 'electrics')
            self.electrics.connect(self.bng, self.vehicle)

        self.camera = Camera(
            'LKA',
            self.bng,
            self.vehicle,
            requested_update_time=0.067,
            pos=(0, -0.35, 1.3),
            resolution=(1280, 720),
            field_of_view_y=30,
            near_far_planes=(0.1, 150),
            is_render_colours=True,
            is_render_depth=False,
            is_render_annotations=False,
            is_render_instance=False,
            is_streaming=True,
            is_using_shared_memory=True,
            is_visualised=False
        )

        self.timestamp = 0.0
        self.last_brake = 0.0
        self.steering = 0.0
        self.running = True

        self.vehicle.control(throttle=0.0, brake=0.0)

        self.last_radii = [None, None]

        self.lka_thread = threading.Thread(target=self.__lka_control_loop)
        self.lka_thread.daemon = True  # Thread will be terminated when main program exits
        self.lka_thread.start()

        self.logger.info("Lane-keeping assist started.")

    def __lka_control_loop(self):
        # 2. Loop until stop() is called
        while self.running:
            self.vehicle.sensors.poll('state', 'electrics')

            if (self.electrics.data['wheelspeed'] > 11  # 11
                and not self.electrics.data['hazard_signal']
                and not self.electrics.data['left_signal']
                and not self.electrics.data['right_signal']):

                if self.vehicle.state['time'] - self.timestamp > 0.067:
                    # 3. Poll sensors
                    img = self.camera.stream()['colour']

                    # 4. Process sensors
                    img = np.array(img, dtype=np.uint8).reshape(720, 1280, 3)

                    # 5. Process lane detection
                    radius, bases = self.__analyze_img(img)

                    if radius is None:
                        self.__reset_adas_controls()
                        self.last_radii = [None, None]
                        continue

                    # 6. Calculate braking/steering correction
                    throttle, brake = self.__calculate_braking(np.sqrt(2.8 * radius))
                    self.last_brake = brake

                    if self.steering_strength > 0:
                        self.steering = self.__calculate_steering(bases)

                    # 7. Output
                    self.vehicle.control(throttle=throttle, brake=brake, steering=self.steering, is_adas=True)

                    self.timestamp = self.vehicle.state['time']
            else:
                self.last_radii = [None, None]
                self.__reset_adas_controls()

    def __reset_adas_controls(self):
        self.vehicle.control(throttle=1.0, brake=0.0, is_adas=True)
        self.last_brake = 0.0
        if self.steering != 0.0:
            self.vehicle.control(steering=0.0, is_adas=True)
            self.steering = 0.0

    def __analyze_img(self, img):
        # TUNING: think about detecting yellow lines
        # TUNING: give steering force in one direction
        # Note: test higher graphics settings

        processed = self.__binary_threshold(img)
        processed = self.__birdeye_view(processed)

        MIDPOINT = 640
        halves = [
            np.sum(processed[650::5, :MIDPOINT], axis=0),
            np.sum(processed[650::5, MIDPOINT:], axis=0)
        ]
        bases = [None, None]

        for i, half in enumerate(halves):
            peak = np.argmax(half)
            if half[peak] > 1:
                bases[i] = peak + (MIDPOINT * i)

        # Detect lane pixels using windowing algorithm
        lane_x, lane_y = self.__detect_lane_lines(processed, bases)

        # Check if we have any valid lanes
        if (lane_x[0] is None or len(lane_x[0]) == 0) and (lane_x[1] is None or len(lane_x[1]) == 0):
            return None, None

        # Fit polynomials
        fitx, ploty = self.__fit_poly(
            processed, lane_x, lane_y, bases
        )

        # DEBUG
        # if fitx[0] is not None or fitx[1] is not None or any(x is not None for x in lane_x):
        #     self.bng.pause()
        #     plt.figure(figsize=(12, 8))
        #     plt.imshow(processed, cmap='gray')

        #     for x_coords, y_coords in zip(lane_x, lane_y):
        #         if x_coords is not None and y_coords is not None:
        #             plt.scatter(x_coords, y_coords, s=1, alpha=0.6)

        #     for fit in fitx:
        #         if fit is not None:
        #             valid = (fit >= 0) & (fit < processed.shape[1])
        #             if np.any(valid):
        #                 plt.plot(fit[valid], ploty[valid], linewidth=3)

        #     plt.show()
        #     self.bng.resume()

        radii = self.__limit_radius_change(self.__measure_curvature(fitx, ploty))
        radii = [r for r in radii if r is not None]

        return np.mean(radii) if radii else None, bases

    def __binary_threshold(self, img):
        gray = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2])

        avg = cv2.mean(gray[580::10, :])[0]
        thresh = avg * 1.5 * np.exp(-0.0027 * (avg - 90))

        white_binary = np.zeros_like(gray)
        white_binary[(gray > thresh) & (gray <= 240)] = 1

        if not self.detect_yellow:
            return white_binary

        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        fac = thresh / 100.0

        yellow_mask = ((hsv[:, :, 0] >= 17) & (hsv[:, :, 0] <= (40 + 25 * np.log(1 + fac))) &  # hue
                       (hsv[:, :, 1] >= (40 + 25 * np.log(1 + fac))) &  # saturation
                       (hsv[:, :, 2] >= (50 + 35 * (fac ** 0.5))))  # value

        yellow_binary = np.zeros_like(white_binary)
        yellow_binary[yellow_mask] = 1

        combined_binary = np.clip(yellow_binary + white_binary, 0, 1)

        return combined_binary

    def __birdeye_view(self, img):
        # Cache transformation matrix if not already computed
        if not hasattr(self, '_birdeye_transform'):
            img_size = (img.shape[1], img.shape[0])
            offset = 400

            src = np.array([
                (117, 719),  # bottom-left corner
                (601, 383),  # top-left corner
                (703, 383),  # top-right corner
                (1215, 719)  # bottom-right corner
            ], dtype='f')
            dst = np.array([
                [offset, img_size[1]],
                [offset, 0],
                [img_size[0] - offset, 0],
                [img_size[0] - offset, img_size[1]]
            ], dtype='f')

            self._birdeye_transform = cv2.getPerspectiveTransform(src, dst)
            self._birdeye_img_size = img_size

        return cv2.warpPerspective(img, self._birdeye_transform, self._birdeye_img_size)

    def __detect_lane_lines(self, binary_birdeye, bases):
        # Constants
        NWINDOWS = 18
        MARGIN = 80
        MINPIX = 30

        if ((bases[0] is None and bases[1] is None) or
           ((bases[0] is not None and bases[1] is not None) and
            (bases[1] - bases[0] < 200 or bases[0] > 590 or bases[1] < 690))):  # MIDPOINT -/+ 50
            return [None, None], [None, None]

        window_h = binary_birdeye.shape[0] // NWINDOWS

        nonzero = binary_birdeye.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        lane_pixels = [[], []]

        for i, base in enumerate(bases):
            if base is None:
                continue

            curr_x = base
            empty_count = 0
            delta = 0

            for window in range(NWINDOWS):
                win_y_low = binary_birdeye.shape[0] - (window + 1) * window_h
                win_y_high = binary_birdeye.shape[0] - window * window_h
                curr_x += delta

                good_lane_pixels = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                                  & (nonzerox >= curr_x - MARGIN)
                                  & (nonzerox < curr_x + MARGIN)).nonzero()[0]

                if good_lane_pixels.size != 0:
                    lane_pixels[i].append(good_lane_pixels)
                    empty_count = 0
                    if len(good_lane_pixels) > MINPIX:
                        delta = curr_x - delta
                        curr_x = np.int32(np.mean(nonzerox[good_lane_pixels]))
                        delta = curr_x - delta
                        if curr_x - MARGIN <= 0 or curr_x + MARGIN >= 1280:
                            break
                else:
                    empty_count += 1

                if empty_count > 1:
                    break

        return ([nonzerox[np.concatenate(pixels)] if pixels else None for pixels in lane_pixels],
                [nonzeroy[np.concatenate(pixels)] if pixels else None for pixels in lane_pixels])

    def __fit_poly(self, binary_birdeye, lane_x, lane_y, bases):
        fits = [None, None]

        for i, x_coords in enumerate(lane_x):
            if x_coords is not None and len(x_coords) > 2 and bases[i] is not None:
                try:
                    # Extend arrays with base constraint points
                    extended_y = np.concatenate([lane_y[i], [719] * 10])
                    extended_x = np.concatenate([x_coords, [bases[i]] * 10])
                    fits[i] = np.polyfit(extended_y, extended_x, 2)
                except np.linalg.LinAlgError:
                    fits[i] = None

        if fits[0] is None and fits[1] is None:
            return [None, None], None

        ploty = np.linspace(0, binary_birdeye.shape[0] - 1, binary_birdeye.shape[0])

        fitx = [None, None]
        for i, fit in enumerate(fits):
            if fit is not None:
                fitx[i] = fit[0] * ploty**2 + fit[1] * ploty + fit[2]

        if fitx[0] is not None and fitx[1] is not None:
            if np.isclose(fitx[0], fitx[1], atol=50).any():
                return [None, None], None

        return fitx, ploty

    def __measure_curvature(self, fitx, ploty):
        # Conversion factors from pixels to real world
        YM_PER_PIXEL = 60 / 720  # meters per pixel in y dimension
        XM_PER_PIXEL = 3.7 / 700  # meters per pixel in x dimension

        radii = [None, None]
        y_eval = 719

        for i, fit in enumerate(fitx):
            if fit is not None:
                fit_cr = np.polyfit(ploty * YM_PER_PIXEL, fit * XM_PER_PIXEL, 2)
                radii[i] = (1 + (2 * fit_cr[0] * y_eval * YM_PER_PIXEL + fit_cr[1])**2)**1.5 / np.absolute(2 * fit_cr[0])

        return radii

    def __limit_radius_change(self, radii):
        for i in range(2):
            if radii[i] is not None and self.last_radii[i] is not None:
                radius_change_pct = abs(radii[i] - self.last_radii[i]) / self.last_radii[i]

                if radius_change_pct > 0.35:
                    max_change = self.last_radii[i] * 0.35
                    radii[i] = self.last_radii[i] + max_change if radii[i] > self.last_radii[i] else self.last_radii[i] - max_change

            self.last_radii[i] = radii[i]

        return radii

    def __calculate_braking(self, target):
        speed_error = self.electrics.data['wheelspeed'] - target

        if speed_error > 1:
            return 0.0, min(1.0, (self.electrics.data['wheelspeed']**2 - target**2) / (981 + 245 * self.risk_level))
        elif speed_error > -11:
            return 0.0, 0.0
        else:
            return 1.0, 0.0

    def __calculate_steering(self, bases):
        if abs(self.electrics.data['steering']) > 45:
            return 0

        if bases[0] is not None and bases[0] > 460:  # 640 - 180
            #  -0.001 * self.steering_strength * (min(bases[0] - 460, 100) / 100)
            return -0.00001 * self.steering_strength * min(bases[0] - 460, 100)
        elif bases[1] is not None and bases[1] < 820:  # 640 + 180
            return 0.00001 * self.steering_strength * min(820 - bases[1], 100)

        return 0

    def stop(self):
        """
        This method stops the Lane-keeping assist for the vehicle it was originally started on.
        """
        if not self.running:
            self.logger.warning("LKA is not running.")
            return

        self.running = False
        self.lka_thread.join(10)
        if self.lka_thread.is_alive():
            self.logger.error("Could not stop LKA.")

        self.vehicle.detach_sensor('electrics')
        self.electrics.detach(self.vehicle, 'electrics')
        self.electrics.disconnect(self.bng, self.vehicle)
        self.electrics = None
        self.camera.remove()
        self.logger.info("Lane-keeping assist stopped.")
