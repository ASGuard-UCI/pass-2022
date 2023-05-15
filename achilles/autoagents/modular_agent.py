#!/usr/bin/env python

"""
This module provides a modular agent to control the ego vehicle
"""

from __future__ import print_function

import os
import sys
import yaml
import logging
import numpy as np
import lgsvl

from achilles.utils.file_parser import parse_yaml
from achilles.autoagents.autonomous_agent import AutonomousAgent

from achilles.autoagents.modules.detector_faster_rcnn_inception import FasterRCNNInception
from achilles.autoagents.modules.obstacle_tracker_2d import Tracker2D
from achilles.autoagents.modules.fusion_tracker_2d import FusionTracker2D
from achilles.autoagents.modules.obstacle_locator import StopSignLocator
from achilles.autoagents.modules.controllers import LateralController, LongitudinalController
from achilles.autoagents.modules.hdmap import HDMap


def get_entry_point():
    return 'ModularAgent'


class ModularAgent(AutonomousAgent):

    """
    Modular autonomous agent to control the ego vehicle
    """

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        assert path_to_conf_file
        conf = parse_yaml(path_to_conf_file)
        self._target_speed = conf.get('target_speed', 11.176)
        self._avg_acceleration = conf.get('avg_acceleration', 5.4)
        self._safe_deceleration = conf.get('safe_deceleration', -3.4)
        self._max_deceleration = conf.get('max_deceleration', -4.0)
        self._detection_frequency = conf.get('detection_frequency', 20)
        self._control_frequency = conf.get('control_frequency', 100)
        self._control_period = 1.0/self._control_frequency
        self._2d_to_3d = conf.get('2d_to_3d', 'Pinhole')
        self._fusion = conf.get('fusion', False)

        self._hdmap = HDMap()

        self._need_stop = False  # for stop sign detection based control

        # Modules
        self._detector = getattr(sys.modules[__name__], conf.get('detector'))()
        self._tracker = getattr(sys.modules[__name__], conf.get('tracker'))()

        if self._fusion:
            self._tracker_hdmap = getattr(sys.modules[__name__], conf.get('tracker'))()
            self._tracker_fusion = FusionTracker2D()

        self._stop_sign_locator = StopSignLocator()
        self._lat_ctrl = getattr(sys.modules[__name__], conf.get('lat_controller'))(self._global_plan)
        self._lon_ctrl = getattr(sys.modules[__name__], conf.get('lon_controller'))(
                self._global_plan, self._target_speed, self._control_period)

        class_name = get_entry_point()
        logdir = os.environ.get('ACHILLES_LOGDIR', '/tmp/svl_log')
        logging.basicConfig(filename=os.path.join(logdir, f"{class_name}.log"),
                level=logging.INFO, format="[%(asctime)s] %(message)s")

    def sensors(self):
        sensors = []
        return sensors

    def run_step(self, input_data, timestamp):

        dist_to_stop_sign_hdmap = self._hdmap.query_distance_to_stop_sign(input_data)

        v = input_data['ego_state'].speed
        stop_dist = self.stopping_distance(v, self._safe_deceleration)

        if 'main_camera' in input_data:
            logging.info("{:.2f}: GT stop sign distance: {:.2f}".format(timestamp, dist_to_stop_sign_hdmap))
            print("{:.2f}: GT stop sign distance: {:.2f}".format(timestamp, dist_to_stop_sign_hdmap))

            obstacles = self._detector.run_step(input_data, timestamp)
            logging.info("{:.2f}: detection results: {}".format(timestamp, len(obstacles)))
            for obs in obstacles:
                if obs.is_stop_sign():
                    logging.info(f"\t\t stop sign: {obs.get_bbox()}, score: {obs.get_score()}")
                    print(f"\t\t stop sign: {obs.get_bbox()}, score: {obs.get_score()}")
                else:
                    logging.info(f"\t\t obs type {obs.get_raw_type()}: {obs.get_bbox()}, score: {obs.get_score()}")
                    print(f"\t\t obs type {obs.get_raw_type()}: {obs.get_bbox()}, score: {obs.get_score()}")

            logging.info("Tracking:")
            print("Tracking")
            tracks = self._tracker.run_step(input_data, timestamp, obstacles)

            if self._fusion:
                logging.info("Fusing obstacle tracks from camera:")
                print("Fusing obstacle tracks from camera:")
                tracks_fusion = self._tracker_fusion.run_step(input_data, timestamp, tracks)

                obstacles_hdmap = self._hdmap.query_stop_sign_obs_front(input_data)
                logging.info("Tracking from HDMap:")
                print("Tracking from HDMap:")
                tracks_hdmap = self._tracker_hdmap.run_step(input_data, timestamp, obstacles_hdmap)

                logging.info("Fusing obstacle tracks from HDMap:")
                print("Fusing obstacle tracks from HDMap:")
                tracks_fusion = self._tracker_fusion.run_step(input_data, timestamp, tracks_hdmap)

                tracks = tracks_fusion

            if not self._need_stop:
                for trk in tracks:
                    if trk.is_stop_sign():
                        if self._2d_to_3d == 'HDMap':
                            dist_to_stop_sign = dist_to_stop_sign_hdmap
                        elif self._2d_to_3d == 'Pinhole':
                            dist_to_stop_sign_pinhole = \
                                    self._stop_sign_locator.run_step(
                                            input_data, timestamp, trk)
                            dist_to_stop_sign = dist_to_stop_sign_pinhole
                            print("Dist to stop sign: HDMap {:.2f} vs Pinhole {:.2f}".format(
                                dist_to_stop_sign_hdmap, dist_to_stop_sign_pinhole))
                        if dist_to_stop_sign < stop_dist + 1.0:
                            self._need_stop = True
            else:
                found_stop_track = False
                for trk in tracks:
                    if trk.is_stop_sign():
                        if self._2d_to_3d == 'Pinhole':
                            dist_to_stop_sign_pinhole = self._stop_sign_locator.run_step(
                                    input_data, timestamp, trk)
                        found_stop_track = True
                logging.info(f"Found stop track: {found_stop_track}")
                if not found_stop_track:
                    self._need_stop = False

            logging.info(f"Number of published tracks: {len(tracks)}, need stop? {self._need_stop}")
            print(f"Number of published tracks: {len(tracks)}, need stop? {self._need_stop}")

        if self._need_stop:
            a = self._safe_deceleration
        elif v < self._target_speed:
            a = self._avg_acceleration
        else:
            a = 0

        if self._need_stop and v > self._target_speed - 1.0:
            k = 0.025
            coeff = k * a  # Overshoot the PID in the beginning
        else:
            coeff = 0.
        dt = 1.0/self._control_frequency
        next_v = min(self._target_speed, max(0., v + (a * dt) + coeff))
        self._lon_ctrl.set_target_speed(next_v)
        if 'main_camera' in input_data:
            logging.info(f"Current speed: {v}, target speed: {next_v}")
            print(f"Current speed: {v}, target speed: {next_v}")

        steering = self._lat_ctrl.run_step(input_data, timestamp)
        throttle, braking = self._lon_ctrl.run_step(input_data, timestamp)

        # RETURN CONTROL
        control = lgsvl.VehicleControl()
        control.steering = steering
        control.throttle = throttle
        control.braking = braking
        control.reverse = False
        control.handbrake = False

        # optional
        control.headlights = None  # int, 0=off, 1=low, 2=high beams
        control.windshield_wipers = None  # int, 0=off, 1-3=on
        control.turn_signal_left = None  # bool
        control.turn_signal_right = None  # bool

        return control

    @staticmethod
    def stopping_distance(v_0, a):
        if a > 0:
            return float('inf')
        t = v_0 / (-a)
        d = v_0 * t + (0.5 * a * (t * t))
        return d

    @staticmethod
    def stopping_duration(v_0, a):
        if a > 0:
            return float('inf')
        t = v_0 / (-a)
        return t

    @staticmethod
    def speeds_profile(v_0, a, t, freq):
        steps = int(t / freq) + 1
        vs = []
        dt = 1.0/freq
        v_i = v_0
        for i in range(steps):
            v_i = max(0., v_i + (a * dt))
            vs.append(v_i)

        return vs
