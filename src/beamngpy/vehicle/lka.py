from __future__ import annotations

import threading
import numpy as np
import cv2

from logging import DEBUG, getLogger
from typing import TYPE_CHECKING

from beamngpy.logging import LOGGER_ID

if TYPE_CHECKING:
    from beamngpy.beamng import BeamNGpy
    from beamngpy.vehicle import Vehicle

from beamngpy.sensors import Electrics, Camera

#########
## WIP ##
#########


class LaneKeepingAssist:
    """
    A camera sensor-based ADAS feature, preventing overspeeding into corners.
    The system uses the road markings to detect the radius of the corner ahead
    of the vehicle and slow down to a safe speed.

    Args:
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this feature should be attached.
    """

    def __init__(
        self,
        bng: BeamNGpy,
        vehicle: Vehicle,
        electrics: Electrics | None = None
    ):
        self.logger = getLogger(f"{LOGGER_ID}.LaneKeepingAssist")
        self.logger.setLevel(DEBUG)

        self.bng = bng
        self.vehicle = vehicle
        self.electrics = electrics
        self.running = False

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
            field_of_view_y=55,
            near_far_planes=(0.1, 100),
            is_render_colours=True,
            is_render_depth=False,
            is_render_annotations=False,
            is_render_instance=False,
            is_streaming=True,
            is_using_shared_memory=True,
            is_visualised=True
        )

        self.prev_left_fit = np.array([])
        self.prev_right_fit = np.array([])
        self.left_fit_hist = np.array([])
        self.right_fit_hist = np.array([])
        self.timestamp = 0.0
        self.last_brake = 0.0
        self.running = True

        self.lka_thread = threading.Thread(target=self.__lka_control_loop)
        self.lka_thread.daemon = True  # Thread will be terminated when main program exits
        self.lka_thread.start()

        self.logger.info("Lane-keeping assist started.")

    def __lka_control_loop(self):
        # 2. Loop until stop() is called
        while self.running:
            self.vehicle.sensors.poll('state', 'electrics')

            if (self.vehicle.state['time'] - self.timestamp > 0.067
                and self.electrics.data['wheelspeed'] > 11
                and not self.electrics.data['hazard_signal']
                and not self.electrics.data['left_signal']
                and not self.electrics.data['right_signal']):

                # 3. Poll sensors
                img = self.camera.stream()['colour']

                # 4. Process sensors
                # Get camera image for lane detection
                img = np.array(img, dtype=np.uint8).reshape(720, 1280, 3)
                img = (0.299 * img[:, :, 0] +
                       0.587 * img[:, :, 1] +
                       0.114 * img[:, :, 2])

                # 5. Process lane detection
                radius = self.__analyze_img(img)

                if radius is None: continue

                # 6. Calculate braking/steering correction
                throttle, brake = self.__calculate_braking(np.sqrt(4.6 * radius))
                self.last_brake = brake

                # 7. Output
                self.vehicle.control(throttle=throttle, brake=brake, is_adas=True)

                self.timestamp = self.vehicle.state['time']
            elif self.last_brake > 0.0:
                self.vehicle.control(throttle=1.0, brake=0.0, is_adas=True)
                self.last_brake = 0.0


    def __analyze_img(self, img):
        # Note: DONE: instead of making a histogram of bottom 10%, take more sparse samples e.g. once every 5-10 rows.
        # Note: think about detecting yellow lines
        # Note: detection of only one line
        # Note: give steering force in one direction

        if self.vehicle.state['time'] - self.timestamp > 5:
            self.__delete_hist(True)

        processed = self.__binary_threshold(img)
        processed = self.__birdeye_view(processed)

        if len(self.left_fit_hist) == 0:
            leftx, lefty, rightx, righty = self.detect_lane_lines(processed)
            if leftx is None:
                return None

            left_fit, right_fit, left_fitx, right_fitx, ploty = self.fit_poly(
                processed, leftx, lefty, rightx, righty
            )
            if left_fit is None:
                return None

            self.left_fit_hist = np.array(left_fit)
            self.right_fit_hist = np.array(right_fit)
        else:
            self.prev_left_fit = [
                np.mean(self.left_fit_hist[:, 0]),
                np.mean(self.left_fit_hist[:, 1]),
                np.mean(self.left_fit_hist[:, 2])
            ]
            self.prev_right_fit = [
                np.mean(self.right_fit_hist[:, 0]),
                np.mean(self.right_fit_hist[:, 1]),
                np.mean(self.right_fit_hist[:, 2])
            ]
            leftx, lefty, rightx, righty = self.find_lane_pixels_using_prev_poly(processed)

            if len(lefty) == 0 or len(righty) == 0:
                self.__delete_hist(len(self.left_fit_hist) == 2)
                return None
            left_fit, right_fit, left_fitx, right_fitx, ploty = self.fit_poly(
                processed, leftx, lefty, rightx, righty
            )

            if left_fit is None:
                self.__delete_hist(len(self.left_fit_hist) == 2)
                return None
            if len(self.left_fit_hist) > 9:
                self.__delete_hist(False)

        new_left_fit = np.array(left_fit)
        new_right_fit = np.array(right_fit)
        self.left_fit_hist = np.vstack([self.left_fit_hist, new_left_fit])
        self.right_fit_hist = np.vstack([self.right_fit_hist, new_right_fit])

        left_rad, right_rad = self.measure_curvature(left_fitx, right_fitx, ploty)

        return np.mean([left_rad, right_rad])

    def __binary_threshold(self, img):
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        avg = cv2.mean(img[650:, :])[0]
        thresh = avg + (240 - avg) * 0.7

        white_binary = np.zeros_like(blur)
        white_binary[(blur > thresh) & (blur <= 240)] = 1

        return white_binary

    def __birdeye_view(self, img):
        img_size = (img.shape[1], img.shape[0])
        offset = 400

        src = np.array([
            (390, 547),  # bottom-left corner
            (617, 383),  # top-left corner
            (677, 383),  # top-right corner
            (903, 547)   # bottom-right corner
        ], dtype='f')
        dst = np.array([
            [offset, img_size[1]],               # bottom-left corner
            [offset, 0],                         # top-left corner
            [img_size[0] - offset, 0],           # top-right corner
            [img_size[0] - offset, img_size[1]]  # bottom-right corner
        ], dtype='f')

        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, img_size)

        return warped

    def detect_lane_lines(self, binary_birdeye):
        histogram = np.sum(binary_birdeye[650::10, :], axis=0)

        nonzero_ind = np.nonzero(histogram)[0]
        peak_bounds = np.split(nonzero_ind, np.where(np.diff(nonzero_ind) > 1)[0] + 1)

        if len(peak_bounds) < 2:
            return None, None, None, None

        gap_sizes = np.diff([np.average(peak) for peak in peak_bounds])
        max_gap_index = np.argmax(gap_sizes)

        if gap_sizes[max_gap_index] < 200:
            return None, None, None, None

        midpoint = (peak_bounds[max_gap_index][-1] +
                   peak_bounds[max_gap_index + 1][0]) // 2  # type: ignore
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint  # type: ignore

        nwindows = 15
        margin = 100
        minpix = 20

        window_h = np.int32(binary_birdeye.shape[0] // nwindows)

        nonzero = binary_birdeye.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_curr = left_base
        right_curr = right_base

        left_lane = []
        right_lane = []

        complete_left = False
        complete_right = False

        for window in range(nwindows):
            if complete_left and complete_right: break

            win_y_low = binary_birdeye.shape[0] - (window + 1) * window_h  # type: ignore
            win_y_high = binary_birdeye.shape[0] - window * window_h  # type: ignore

            if not complete_left:
                win_left_low = left_curr - margin  # type: ignore
                win_left_high = left_curr + margin  # type: ignore
                good_left_lane = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                                & (nonzerox >= win_left_low) & (nonzerox < win_left_high)).nonzero()[0]
                if good_left_lane.size != 0:
                    left_lane.append(good_left_lane)
                    if len(good_left_lane) > minpix:
                        left_curr = np.int32(np.mean(nonzerox[good_left_lane]))
                        if left_curr - margin <= 0 or left_curr + margin >= 1280:  # type: ignore
                            complete_left = True

            if not complete_right:
                win_right_low = right_curr - margin  # type: ignore
                win_right_high = right_curr + margin  # type: ignore
                good_right_lane = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                                & (nonzerox >= win_right_low) & (nonzerox < win_right_high)).nonzero()[0]
                if good_right_lane.size != 0:
                    right_lane.append(good_right_lane)
                    if len(good_right_lane) > minpix:
                        right_curr = np.int32(np.mean(nonzerox[good_right_lane]))
                        if right_curr - margin <= 0 or right_curr + margin >= 1280:  # type: ignore
                            complete_right = True

        try:
            left_lane = np.concatenate(left_lane)
            right_lane = np.concatenate(right_lane)
        except ValueError:
            print('No lines detected!')
            return None, None, None, None

        return (nonzerox[left_lane], nonzeroy[left_lane],
                nonzerox[right_lane], nonzeroy[right_lane])  # type: ignore

    def fit_poly(self, binary_birdeye, leftx, lefty, rightx, righty):
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        ploty = np.linspace(0, binary_birdeye.shape[0] - 1, binary_birdeye.shape[0])
        try:
            left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]
        except TypeError:
            print("Failed to fit lane!")
            return None, None, None, None, None

        if np.isclose(left_fitx, right_fitx, atol=250).any():
            return None, None, None, None, None

        return left_fit, right_fit, left_fitx, right_fitx, ploty

    def find_lane_pixels_using_prev_poly(self, binary_birdeye):
        margin = 150

        nonzero = binary_birdeye.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane = ((nonzerox > (self.prev_left_fit[0] * (nonzeroy**2) +
                                 self.prev_left_fit[1] * nonzeroy + self.prev_left_fit[2] - margin)) &
                    (nonzerox < (self.prev_left_fit[0] * (nonzeroy**2) +
                                self.prev_left_fit[1] * nonzeroy + self.prev_left_fit[2] + margin))).nonzero()[0]
        right_lane = ((nonzerox > (self.prev_right_fit[0] * (nonzeroy**2) +
                                  self.prev_right_fit[1] * nonzeroy + self.prev_right_fit[2] - margin)) &
                     (nonzerox < (self.prev_right_fit[0] * (nonzeroy**2) +
                                 self.prev_right_fit[1] * nonzeroy + self.prev_right_fit[2] + margin))).nonzero()[0]

        return (nonzerox[left_lane], nonzeroy[left_lane],
                nonzerox[right_lane], nonzeroy[right_lane])

    def measure_curvature(self, left_fitx, right_fitx, ploty):
        ym_ppx = 30 / 720
        xm_ppx = 3.7 / 700

        left_fit_cr = np.polyfit(ploty * ym_ppx, left_fitx * xm_ppx, 2)
        right_fit_cr = np.polyfit(ploty * ym_ppx, right_fitx * xm_ppx, 2)

        y_eval = np.max(ploty)

        left_rad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_ppx + left_fit_cr[1])**2)**1.5) / np.absolute(2 * left_fit_cr[0])
        right_rad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_ppx + right_fit_cr[1])**2)**1.5) / np.absolute(2 * right_fit_cr[0])

        return left_rad, right_rad

    def __delete_hist(self, complete):
        if complete:
            self.left_fit_hist = np.array([])
            self.right_fit_hist = np.array([])
        else:
            self.left_fit_hist = np.delete(self.left_fit_hist, 0, 0)
            self.right_fit_hist = np.delete(self.right_fit_hist, 0, 0)

    def __calculate_braking(self, target):
        if self.electrics.data['wheelspeed'] > target:
            brake = min(1.0, (self.electrics.data['wheelspeed']**2 - target**2) / 981)  # ... / (dist->100 * 1g->9.81)
        else:
            brake = 0.0
        brake = (brake + self.last_brake) / 2
        if brake > 0.02:
            return 0.0, brake
        else:
            return 1.0, 0.0

    def stop(self):
        """
        This method stops the Lane-keeping assist for the given vehicle.
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
