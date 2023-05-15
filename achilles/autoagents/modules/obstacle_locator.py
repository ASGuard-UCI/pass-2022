#!/usr/bin/env python

"""
Base class for object locators
"""
from __future__ import print_function

import math
import numpy as np

from enum import Enum


class CameraModel(object):
    # TODO: extract this part as separate file

    def __init__(self, width, height, fov):
        self.width = width
        self.height = height
        self.fov = fov
        aspect_ratio = float(width) / float(height)
        vertical_fov = fov
        horizon_fov = 2*math.degrees(math.atan(math.tan(math.radians(vertical_fov)/2)*aspect_ratio))
        self.fx = width / (2 * math.tan(0.5 * math.radians(horizon_fov)))
        self.fy = height / (2 * math.tan(0.5 * math.radians(vertical_fov)))
        self.cx = width / 2.
        self.cy = height / 2.

    def projection_matrix(self):
        return [self.fx, 0.0,     self.cx, 0.0,
                0.0,     self.fy, self.cy, 0.0,
                0.0,     0.0,     1.0,     0.0]

    def rectification_matrix(self):
        return [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]


class ObstacleLocator(object):

    """
    TODO: document me!
    """

    def __init__(self):
        pass

    def run_step(self, input_data, timestamp, track):
        return None


class StopSignLocator(ObstacleLocator):

    # st_size = 0.6  # standard size is 0.6m x 0.6m
    st_size = 0.7  # standard size is 0.7m x 0.7m in SF map

    history_weight = 0.25

    def __init__(self):
        self.pix_sz = None

    def run_step(self, input_data, timestamp, track):

        cam_params = input_data['main_camera_params']
        cam_model = CameraModel(cam_params['width'], cam_params['height'], cam_params['fov'])
        center_pix = track.get_centroid()
        w_pix, h_pix = track.get_width_height()

        if not self.pix_sz:
            self.pix_sz = (w_pix + h_pix) / 2.0
        else:
            min_pix = min(w_pix, h_pix)
            max_pix = max(w_pix, h_pix)
            if abs(min_pix - self.pix_sz) < abs(max_pix - self.pix_sz):
                self.pix_sz = self.history_weight * self.pix_sz + (1 - self.history_weight) * min_pix
            else:
                self.pix_sz = self.history_weight * self.pix_sz + (1 - self.history_weight) * max_pix

        # TODO: use KF to track real_dist
        real_dist = (cam_model.fx * self.st_size) / self.pix_sz  # Pinhole camera model

        return real_dist
