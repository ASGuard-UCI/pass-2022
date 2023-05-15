#!/usr/bin/env python

"""
Wrapper for autonomous agents required for tracking and checking of used sensors
"""

from __future__ import print_function
import math
import os
import time
import cv2

import lgsvl

MAX_ALLOWED_RADIUS_SENSOR = 3.0

SENSORS_LIMITS = {
    'sensor.camera.rgb': 4,
    'sensor.lidar.ray_cast': 1,
    'sensor.other.radar': 2,
    'sensor.other.gnss': 1,
    'sensor.other.imu': 1,
    'sensor.opendrive_map': 1,
    'sensor.speedometer': 1
}


class AgentError(Exception):
    """
    Exceptions thrown when the agent returns an error during the simulation
    """

    def __init__(self, message):
        super(AgentError, self).__init__(message)


class AgentWrapper(object):

    """
    Wrapper for autonomous agents required for tracking and checking of used sensors
    """

    allowed_sensors = [
        'sensor.opendrive_map',
        'sensor.speedometer',
        'sensor.camera.rgb',
        'sensor.camera',
        'sensor.lidar.ray_cast',
        'sensor.other.radar',
        'sensor.other.gnss',
        'sensor.other.imu'
    ]

    _agent = None

    def __init__(self, agent, ego_vehicle):
        """
        Set the autonomous agent
        """
        self._agent = agent
        self._ego = ego_vehicle

    def __call__(self, input_data={}):
        """
        Pass the call directly to the agent
        """
        input_data['ego_state'] = self._ego.state
        return self._agent(input_data)

    def setup_sensors(self, vehicle, sensor_list):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param vehicle: ego vehicle
        :return:
        """
        # default_sensors = ['velodyne', 'Main Camera', 'Telephoto Camera', 'GPS', 'IMU']
        for s in vehicle.get_sensors():
            if s.name in sensor_list:
                s.enabled = True
                sensor_tag = sensor_type = '_'.join(list(map(str.lower, s.name.split())))
                self._agent.sensor_interface.register_sensor(sensor_tag, sensor_type, s)
            # else:
            #     s.enabled = False

    @staticmethod
    def validate_sensor_configuration(sensors):
        """
        Ensure that the sensor configuration is valid, in case the challenge mode is used
        Returns true on valid configuration, false otherwise
        """

        sensor_count = {}
        sensor_ids = []

        for sensor in sensors:

            # Check if the is has been already used
            sensor_id = sensor['id']
            if sensor_id in sensor_ids:
                raise RuntimeError("Duplicated sensor tag [{}]".format(sensor_id))
            else:
                sensor_ids.append(sensor_id)

            # Check the sensors validity
            if sensor['type'] not in AgentWrapper.allowed_sensors:
                raise RuntimeError("Illegal sensor used. {} are not allowed!".format(sensor['type']))

            # Check the amount of sensors
            if sensor['type'] in sensor_count:
                sensor_count[sensor['type']] += 1
            else:
                sensor_count[sensor['type']] = 1


        for sensor_type, max_instances_allowed in SENSORS_LIMITS.items():
            if sensor_type in sensor_count and sensor_count[sensor_type] > max_instances_allowed:
                raise RuntimeError(
                    "Too many {} used! "
                    "Maximum number allowed is {}, but {} were requested.".format(sensor_type,
                                                                                  max_instances_allowed,
                                                                                  sensor_count[sensor_type]))

    def cleanup(self):
        """
        Remove and destroy all sensors
        """
        logdir = os.environ.get('ACHILLES_LOGDIR', '/tmp/svl_log')
        frame_dir = os.path.join(logdir, 'frames')
        frame_count = self._agent.sensor_interface.frame_count()

        print(f"[{os.path.basename(__file__)}] saving simulation video")

        # Save video
        frame_array = []
        w = 1920
        h = 1080
        for i in range(0, frame_count, 1):
            png_file = os.path.join(frame_dir, f'main_camera-{i}.png')
            bgr_img = cv2.imread(png_file)
            frame_array.append(bgr_img)
        video_file = os.path.join(logdir, 'main_camera.mp4')
        fps = 20
        out = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        for i in range(len(frame_array)):
            out.write(frame_array[i])
        out.release()

        # Create video with annotations
        frame_array = []
        w = 1920
        h = 1080
        for i in range(0, frame_count, 1):
            png_file = os.path.join(frame_dir, f'main_camera_anno-{i}.png')
            bgr_img = cv2.imread(png_file)
            frame_array.append(bgr_img)
        video_file = os.path.join(logdir, 'main_camera_anno.mp4')
        fps = 20
        out = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        for i in range(len(frame_array)):
            out.write(frame_array[i])
        out.release()
