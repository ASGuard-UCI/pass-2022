#!/usr/bin/env python

"""
Sensor interface
"""
from __future__ import print_function

import numpy as np
import os

import lgsvl

from achilles.utils.file_parser import read_image


class SensorConfigurationInvalid(Exception):
    """
    Exceptions thrown when the sensors used by the agent are not allowed for that specific submissions
    """

    def __init__(self, message):
        super(SensorConfigurationInvalid, self).__init__(message)


class SensorReceivedNoData(Exception):
    """
    Exceptions thrown when the sensors used by the agent take too long to receive data
    """

    def __init__(self, message):
        super(SensorReceivedNoData, self).__init__(message)


class SensorInterface(object):
    def __init__(self):
        self._sensors_objects = {}
        self._sensors_transforms = {}
        self._frame_count = 0

    def frame_count(self):
        return self._frame_count

    def register_sensor(self, tag, sensor_type, sensor):
        self._sensors_objects[tag] = sensor
        self._sensors_transforms[tag] = sensor.transform

    def get_data(self, get_camera=False):
        # default_sensors = ['velodyne', 'Main Camera', 'Telephoto Camera', 'GPS', 'IMU']
        logdir = os.environ.get("ACHILLES_LOGDIR", '/tmp/svl_log')
        frame_dir = os.path.join(logdir, 'frames')
        os.makedirs(frame_dir, exist_ok=True)
        try:
            data_dict = {}
            for tag in self._sensors_objects.keys():
                if tag.startswith('main_camera'):
                    if get_camera:
                        frame_path = os.path.join(frame_dir, f'main_camera-{self._frame_count}.png')
                        self._sensors_objects[tag].save(frame_path, compression=3)
                        img = read_image(frame_path)
                        data_dict['main_camera'] = img
                        data_dict['frame_id'] = self._frame_count
                        data_dict['main_camera_params'] = {
                                'width': self._sensors_objects[tag].width,
                                'height': self._sensors_objects[tag].height,
                                'fov': self._sensors_objects[tag].fov,
                                'trans': self._sensors_objects[tag].transform
                                }
                        self._frame_count += 1
                elif tag.startswith('velodyne'):
                    pass
                elif tag.startswith('telephoto_camera'):
                    pass
                elif tag.startswith('gps'):
                    data_dict['gps'] = self._sensors_objects[tag].data
                elif tag.startswith('imu'):
                    pass
                    # for carla challenge agents' sensors like learning by cheating
                elif tag.endswith('rgb'):
                    if get_camera:
                        frame_path = os.path.join(frame_dir, f'rgb-{self._frame_count}.png')
                        self._sensors_objects[tag].save(frame_path, compression=3)
                        img = read_image(frame_path)
                        data_dict['rgb'] = img
                elif tag.startswith('rgb_left'):
                    if get_camera:
                        frame_path = os.path.join(frame_dir, f'rgbl-{self._frame_count}.png')
                        self._sensors_objects[tag].save(frame_path, compression=3)
                        img = read_image(frame_path)
                        data_dict['rgb_left'] = img
                elif tag.startswith('rgb_right'):
                    if get_camera:
                        frame_path = os.path.join(frame_dir, f'rgbr-{self._frame_count}.png')
                        self._sensors_objects[tag].save(frame_path, compression=3)
                        img = read_image(frame_path)
                        data_dict['rgb_right'] = img

        except:
            raise RuntimeError("Fail to get sensor data")

        return data_dict
