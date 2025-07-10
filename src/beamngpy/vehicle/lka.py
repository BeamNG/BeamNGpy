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
    of the vehicle and slow down to a safe speed. The feature is only active at speeds
    above 39.6 km/h and when the vehicle's hazard lights and blinkers are not on.

    Args:
        bng: The BeamNGpy instance, with which to communicate to the simulation.
        vehicle: The vehicle to which this feature should be attached.
        electrics: The electrics sensor to use for the vehicle. If not provided, a new one will be created.
        risk_level: Controls the cornering speed limit. 1 is the lowest, 3 is the highest.
    """

    def __init__(
        self,
        bng: BeamNGpy,
        vehicle: Vehicle,
        electrics: Electrics | None = None,
        safety_level: int = 1
    ):
        self.logger = getLogger(f"{LOGGER_ID}.LaneKeepingAssist")
        self.logger.setLevel(DEBUG)

        self.bng = bng
        self.vehicle = vehicle
        self.electrics = electrics
        self.risk_level = safety_level
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
            field_of_view_y=30,
            near_far_planes=(0.1, 120),
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
        # DONE: instead of making a histogram of bottom 10%, take more sparse samples e.g. once every 5-10 rows.
        # Note: think about detecting yellow lines
        # DONE: detection of only one line
        # Note: give steering force in one direction

        if self.vehicle.state['time'] - self.timestamp > 5:
            self.__delete_hist(True)

        processed = self.__binary_threshold(img)
        processed = self.__birdeye_view(processed)

        # Determine which lanes need detection vs previous poly tracking
        need_detect_left = len(self.left_fit_hist) == 0
        need_detect_right = len(self.right_fit_hist) == 0

        # Initialize results
        leftx, lefty, rightx, righty = None, None, None, None

        # Get detection results for lanes that need it
        if need_detect_left or need_detect_right:
            det_leftx, det_lefty, det_rightx, det_righty = self.__detect_lane_lines(
                processed, process_left=need_detect_left, process_right=need_detect_right
            )
            if need_detect_left:
                leftx = det_leftx
                lefty = det_lefty
            if need_detect_right:
                rightx = det_rightx
                righty = det_righty


        # Get poly tracking results for lanes that have history
        if not need_detect_left or not need_detect_right:
            # Calculate previous fits for tracking
            if not need_detect_left:
                self.prev_left_fit = [
                    np.mean(self.left_fit_hist[:, 0]),
                    np.mean(self.left_fit_hist[:, 1]),
                    np.mean(self.left_fit_hist[:, 2])
                ]
            if not need_detect_right:
                self.prev_right_fit = [
                    np.mean(self.right_fit_hist[:, 0]),
                    np.mean(self.right_fit_hist[:, 1]),
                    np.mean(self.right_fit_hist[:, 2])
                ]

            poly_leftx, poly_lefty, poly_rightx, poly_righty = self.__find_lane_pixels_using_prev_poly(
                processed, process_left=not need_detect_left, process_right=not need_detect_right
            )

            if not need_detect_left:
                leftx = poly_leftx
                lefty = poly_lefty
            if not need_detect_right:
                rightx = poly_rightx
                righty = poly_righty

        # Check if we have any valid lanes
        if (leftx is None or len(leftx) == 0) and (rightx is None or len(rightx) == 0):
            # Delete histories for failed lanes
            self.__delete_hist(True,
                             delete_left=len(self.left_fit_hist) <= 2,
                             delete_right=len(self.right_fit_hist) <= 2)
            return None

        # Fit polynomials
        left_fit, right_fit, left_fitx, right_fitx, ploty = self.__fit_poly(
            processed, leftx, lefty, rightx, righty
        )


        # Only return None if both fits failed
        if left_fit is None and right_fit is None:
            # Delete histories only if we were using previous poly (not fresh detection)
            if not need_detect_left or not need_detect_right:
                self.__delete_hist(True,
                                 delete_left=len(self.left_fit_hist) <= 2,
                                 delete_right=len(self.right_fit_hist) <= 2)
            return None
        else:
            # Update histories for successful fits
            self.__update_history(left_fit, right_fit)


        # Clean up long histories after updating
        if len(self.left_fit_hist) > 9 or len(self.right_fit_hist) > 9:
            self.__delete_hist(False,
                             delete_left=len(self.left_fit_hist) > 9,
                             delete_right=len(self.right_fit_hist) > 9)

        return self.__measure_curvature(left_fitx, right_fitx, ploty)

    def __binary_threshold(self, img):
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        avg = cv2.mean(img[650:, :])[0]
        thresh = avg + (240 - avg) * 0.7

        white_binary = np.zeros_like(blur)
        white_binary[(blur > thresh) & (blur <= 240)] = 1

        return white_binary

    def __birdeye_view(self, img):
        # Cache transformation matrix if not already computed
        if not hasattr(self, '_birdeye_transform'):
            img_size = (img.shape[1], img.shape[0])
            offset = 400

            src = np.array([
                (117, 719),  # bottom-left corner
                (617, 363),  # top-left corner
                (662, 363),  # top-right corner
                (1215, 719)   # bottom-right corner
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

    def __detect_lane_lines(self, binary_birdeye, process_left=True, process_right=True):
        # Constants
        MIDPOINT = 640
        NWINDOWS = 15
        MARGIN = 100
        MINPIX = 20

        histogram = np.sum(binary_birdeye[580::10, :], axis=0)

        left_half = histogram[:MIDPOINT]
        right_half = histogram[MIDPOINT:]

        left_base = None
        right_base = None

        if process_left:
            left_peak = np.argmax(left_half)
            if left_half[left_peak] > 0:
                left_base = left_peak

        if process_right:
            right_peak = np.argmax(right_half)
            if right_half[right_peak] > 0:
                right_base = right_peak + MIDPOINT

        if ((left_base is None and right_base is None) or
           (left_base is not None and right_base is not None and right_base - left_base < 200)):
            return None, None, None, None

        window_h = binary_birdeye.shape[0] // NWINDOWS

        nonzero = binary_birdeye.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_curr = left_base
        right_curr = right_base

        left_lane = []
        right_lane = []

        complete_left = left_base is None or not process_left
        complete_right = right_base is None or not process_right

        for window in range(NWINDOWS):
            if complete_left and complete_right:
                break

            win_y_low = binary_birdeye.shape[0] - (window + 1) * window_h
            win_y_high = binary_birdeye.shape[0] - window * window_h

            if not complete_left:
                good_left_lane = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                                & (nonzerox >= left_curr - MARGIN) & (nonzerox < left_curr + MARGIN)).nonzero()[0]
                if good_left_lane.size != 0:
                    left_lane.append(good_left_lane)
                    if len(good_left_lane) > MINPIX:
                        left_curr = np.int32(np.mean(nonzerox[good_left_lane]))
                        if left_curr - MARGIN <= 0 or left_curr + MARGIN >= 1280:
                            complete_left = True

            if not complete_right:
                good_right_lane = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high)
                                & (nonzerox >= right_curr - MARGIN) & (nonzerox < right_curr + MARGIN)).nonzero()[0]
                if good_right_lane.size != 0:
                    right_lane.append(good_right_lane)
                    if len(good_right_lane) > MINPIX:
                        right_curr = np.int32(np.mean(nonzerox[good_right_lane]))
                        if right_curr - MARGIN <= 0 or right_curr + MARGIN >= 1280:
                            complete_right = True

        return (nonzerox[np.concatenate(left_lane)] if left_lane and process_left else None,
                nonzeroy[np.concatenate(left_lane)] if left_lane and process_left else None,
                nonzerox[np.concatenate(right_lane)] if right_lane and process_right else None,
                nonzeroy[np.concatenate(right_lane)] if right_lane and process_right else None)

    def __fit_poly(self, binary_birdeye, leftx, lefty, rightx, righty):
        left_fit = None
        right_fit = None

        if leftx is not None and len(leftx) > 2:
            try:
                left_fit = np.polyfit(lefty, leftx, 2)
            except np.linalg.LinAlgError:
                left_fit = None

        if rightx is not None and len(rightx) > 2:
            try:
                right_fit = np.polyfit(righty, rightx, 2)
            except np.linalg.LinAlgError:
                right_fit = None

        if left_fit is None and right_fit is None:
            return None, None, None, None, None

        ploty = np.linspace(0, binary_birdeye.shape[0] - 1, binary_birdeye.shape[0])

        left_fitx = None
        right_fitx = None

        if left_fit is not None:
            left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
        if right_fit is not None:
            right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

        if left_fitx is not None and right_fitx is not None:
            if np.isclose(left_fitx, right_fitx, atol=250).any():
                return None, None, None, None, None

        return left_fit, right_fit, left_fitx, right_fitx, ploty

    def __find_lane_pixels_using_prev_poly(self, binary_birdeye, process_left=True, process_right=True):
        MARGIN = 150

        nonzero = binary_birdeye.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        left_lane = None
        right_lane = None

        if process_left and len(self.prev_left_fit) == 3:
            left_poly = (self.prev_left_fit[0] * (nonzeroy**2) +
                        self.prev_left_fit[1] * nonzeroy +
                        self.prev_left_fit[2])
            left_lane = ((nonzerox > left_poly - MARGIN) &
                        (nonzerox < left_poly + MARGIN)).nonzero()[0]

        if process_right and len(self.prev_right_fit) == 3:
            right_poly = (self.prev_right_fit[0] * (nonzeroy**2) +
                        self.prev_right_fit[1] * nonzeroy +
                        self.prev_right_fit[2])
            right_lane = ((nonzerox > right_poly - MARGIN) &
                        (nonzerox < right_poly + MARGIN)).nonzero()[0]

        return (nonzerox[left_lane] if left_lane is not None else None,
                nonzeroy[left_lane] if left_lane is not None else None,
                nonzerox[right_lane] if right_lane is not None else None,
                nonzeroy[right_lane] if right_lane is not None else None)

    def __measure_curvature(self, left_fitx, right_fitx, ploty):
        # Conversion factors from pixels to real world
        YM_PER_PIXEL = 60 / 720  # meters per pixel in y dimension
        XM_PER_PIXEL = 3.7 / 700  # meters per pixel in x dimension

        left_rad = None
        right_rad = None

        y_eval = np.max(ploty)

        if left_fitx is not None:
            left_fit_cr = np.polyfit(ploty * YM_PER_PIXEL, left_fitx * XM_PER_PIXEL, 2)
            left_rad = ((1 + (2 * left_fit_cr[0] * y_eval * YM_PER_PIXEL + left_fit_cr[1])**2)**1.5) / np.absolute(2 * left_fit_cr[0])

        if right_fitx is not None:
            right_fit_cr = np.polyfit(ploty * YM_PER_PIXEL, right_fitx * XM_PER_PIXEL, 2)
            right_rad = ((1 + (2 * right_fit_cr[0] * y_eval * YM_PER_PIXEL + right_fit_cr[1])**2)**1.5) / np.absolute(2 * right_fit_cr[0])

        if left_rad is not None and right_rad is not None:
            return np.mean([left_rad, right_rad])
        else:
            return next((rad for rad in [left_rad, right_rad] if rad is not None), None)

    def __update_history(self, left_fit, right_fit):
        if left_fit is not None:
            if len(self.left_fit_hist) == 0:
                self.left_fit_hist = np.array([left_fit])
            else:
                self.left_fit_hist = np.vstack([self.left_fit_hist, [left_fit]])

        if right_fit is not None:
            if len(self.right_fit_hist) == 0:
                self.right_fit_hist = np.array([right_fit])
            else:
                self.right_fit_hist = np.vstack([self.right_fit_hist, [right_fit]])

    def __delete_hist(self, complete, delete_left=True, delete_right=True):
        if complete:
            if delete_left:
                self.left_fit_hist = np.array([])
            if delete_right:
                self.right_fit_hist = np.array([])
        else:
            if delete_left and len(self.left_fit_hist) > 0:
                self.left_fit_hist = np.delete(self.left_fit_hist, 0, 0)
            if delete_right and len(self.right_fit_hist) > 0:
                self.right_fit_hist = np.delete(self.right_fit_hist, 0, 0)

    def __calculate_braking(self, target):
        if self.electrics.data['wheelspeed'] > target:
            brake = min(1.0, (self.electrics.data['wheelspeed']**2 - target**2) / (491 * (1 + np.clip(self.risk_level, 1, 3))))
            # [...] / (dist->100 * (1g->9.81 * (1 + safety_level) / 2))
            # safety_level = 1 -> 1g
            # safety_level = 2 -> 1.5g
            # safety_level = 3 -> 2g
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
