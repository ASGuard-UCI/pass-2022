#!/usr/bin/env python

"""
This module provides simple lateral and longitudinal controllers
"""

from __future__ import print_function

import math
import numpy as np
from simple_pid import PID

delta_steer_per_iter = 0.001

# Stanley controller parameters
k_e = 0.33
k_v = 0.01


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0*np.pi
    while angle < -np.pi:
        angle += 2.0*np.pi

    return angle


def distance_to_point_and_angle(pt, line_pt, line_angle_radian):
    slope = np.sin(line_angle_radian)
    b = line_pt[1] - slope*line_pt[0]
    c = pt[1] + (pt[0]/slope)
    dist = abs(pt[1] - slope*pt[0] - b) / math.sqrt(1+slope*slope)

    return dist


class LateralController(object):

    """
    Lateral controller class.
    Ref: https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf
    """

    def __init__(self, plan):
        self._plan = []
        for wp in plan:
            self._plan.append([wp["position"][2], wp["position"][0]])
        self._last_steer = 0.0

    def run_step(self, input_data, timestamp):
        position = input_data['ego_state'].transform.position
        waypt = [position.z, position.x]
        v = input_data['ego_state'].speed
        yaw = np.radians(input_data['ego_state'].rotation.y)

        steering = self._lateral_control(waypt, v, yaw)

        return steering

    def _lateral_control(self, waypt, v, yaw):
        yaw_path = np.arctan2(
                self._plan[-1][1] - self._plan[0][1],
                self._plan[-1][0] - self._plan[0][0])
        yaw_diff = normalize_angle(yaw_path - yaw)
        # print("yaw diff:", yaw_diff)
        closest_idx = np.argmin(
                np.sum((np.array(waypt) - np.array(self._plan)[:])**2, axis=1))
        crosstrack_error = distance_to_point_and_angle(
                self._plan[closest_idx], waypt, yaw)
        # print("crosstrack error:", crosstrack_error)
        yaw_crosstrack = np.arctan2(
                waypt[1] - self._plan[0][1],
                waypt[0] - self._plan[0][0])
        yaw_path2ct = normalize_angle(yaw_path - yaw_crosstrack)
        if yaw_path2ct > 0:
            crosstrack_error = abs(crosstrack_error)
        else:
            crosstrack_error = -abs(crosstrack_error)
        yaw_diff_crosstrack = np.arctan(k_e*crosstrack_error/(k_v + v))
        steer = normalize_angle(yaw_diff + yaw_diff_crosstrack)
        steer = np.clip(steer, -1., 1.)
        steer_limited = np.clip(steer,
                self._last_steer - delta_steer_per_iter,
                self._last_steer + delta_steer_per_iter)
        self._last_steer = steer_limited

        return steer_limited


class LongitudinalController(object):

    """
    Longitudinal controller class. A simple PID based controller.
    """

    def __init__(self, plan, target_speed=11.176, control_period=0.01):
        self._plan = []
        for wp in plan:
            self._plan.append([wp["position"][2], wp["position"][0]])
        # PID to decide the acceleration to apply (or deceleration if negative)
        self._pid = PID(1.0, 0.1, 0.05, setpoint=target_speed)
        self._pid.sample_time = control_period
        self._pid.output_limits = (-1, 1)

    def run_step(self, input_data, timestamp):
        v = input_data['ego_state'].speed
        acceleration = self._pid(v)
        if acceleration >= 0.0:
            throttle = acceleration
            braking = 0
        else:
            throttle = 0
            braking = abs(acceleration)

        return throttle, braking

    def set_target_speed(self, target_speed):
        self._pid.setpoint = target_speed
