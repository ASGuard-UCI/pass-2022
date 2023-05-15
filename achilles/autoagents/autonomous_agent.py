#!/usr/bin/env python

"""
This module provides the base class for all autonomous agents
"""

from __future__ import print_function

from enum import Enum

import copy
import cv2
import lgsvl

from achilles.utils.timer import GameTime
from achilles.utils.file_parser import parse_yaml
from achilles.envs.sensor_interface import SensorInterface


class AutonomousAgent(object):

    """
    Autonomous agent base class. All user agents have to be derived from this class
    """

    def __init__(self, path_to_conf_file, ego_routes):

        assert path_to_conf_file
        conf = parse_yaml(path_to_conf_file)
        self._detection_frequency = conf.get('detection_frequency')
        self._control_frequency = conf.get('control_frequency')
        self._detection_interval = self._control_frequency // self._detection_frequency
        self._control_iterations = 0

        # current global plans to reach a destination
        self._global_plan = copy.deepcopy(ego_routes)

        # this data structure will contain all sensor data
        self.sensor_interface = SensorInterface()

        # agent's initialization
        self.setup(path_to_conf_file)

        self.wallclock_t0 = None

    def setup(self, path_to_conf_file):
        """
        Initialize everything needed by your agent and set the track attribute to the right type:
            Track.SENSORS : CAMERAS, LIDAR, RADAR, GPS and IMU sensors are allowed
            Track.MAP : OpenDRIVE map is also allowed
        """
        pass

    def sensors(self):  # pylint: disable=no-self-use
        """
        Define the sensor suite required by the agent

        :return: a list containing the required sensors in the following format:

        [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}
        ]

        """
        sensors = []

        return sensors

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        :return: control
        """
        control = lgsvl.VehicleControl()
        control.steering = 0.0
        control.throttle = 0.0
        control.braking = 0.0
        control.reverse = False
        control.handbrake = False

        # optional
        control.headlights = None  # int, 0=off, 1=low, 2=high beams
        control.windshield_wipers = None  # int, 0=off, 1-3=on
        control.turn_signal_left = None  # bool
        control.turn_signal_right = None  # bool

        return control

    def destroy(self):
        """
        Destroy (clean-up) the agent
        :return:
        """
        pass

    def __call__(self, input_data={}):
        """
        Execute the agent call, e.g. agent()
        Returns the next vehicle controls
        """
        get_camera = True if self._control_iterations % self._detection_interval == 0 else False
        self._control_iterations += 1
        sensor_data = self.sensor_interface.get_data(get_camera=get_camera)

        # if get_camera:
        #     img = sensor_data['main_camera']
        #     self._video_out.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        input_data = {**input_data, **sensor_data}
        timestamp = GameTime.get_time()
        if not self.wallclock_t0:
            self.wallclock_t0 = GameTime.get_wallclocktime()
        wallclock = GameTime.get_wallclocktime()
        wallclock_diff = (wallclock - self.wallclock_t0).total_seconds()

        print('======[Agent] Wallclock_time = {} / Sim_time = {:.2f}'.format(wallclock, timestamp))

        control = self.run_step(input_data, timestamp)

        return control
