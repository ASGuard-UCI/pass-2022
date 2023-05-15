#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a human agent to control the ego vehicle via keyboard
"""
import lgsvl
# from achilles.autoagents.autonomous_agent import AutonomousAgent, Track
from achilles.autoagents.autonomous_agent import AutonomousAgent
import numpy as np

from collections import deque
# import screeninfo


import pygame
import cv2
import math
# import carla
from simple_pid import PID

from redis import ConnectionPool, Redis




def get_entry_point():
    return 'TmbhAgent'



class TmbhAgent(AutonomousAgent):
    """
    Human agent to control the ego vehicle via keyboard
    """

    current_control = None
    agent_engaged = False

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        # self.track = Track.SENSORS

        self.agent_engaged = False
        # self._hic = TmbhInterface()
        self._controller = MsgControl()
        self._prev_timestamp = 0

    def sensors(self):
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

        # sensors = [
        #     {'type': 'sensor.speedometer', 'reading_frequency': 20, 'id': 'speed'},
        #     {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        #      'width': 640, 'height': 480, 'fov': 90, 'id': 'secondary', 'skip': 'true'},
        #     {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        #      'width': self._hic._width, 'height': self._hic._height, 'fov': 90, 'id': 'Center'},
        # ]
        sensors = []

        return sensors

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """
        self.agent_engaged = True
        msg = self._controller.parse_events(timestamp, input_data)
        self._prev_timestamp = timestamp
        return msg

    def destroy(self):
        """
        Cleanup
        """
        # self._hic._quit = True
        del self._controller
        # cv2.destroyAllWindows()


class MsgControl(object):
    """
    Keyboard control for the human agent
    """

    def __init__(self):
        """
        Init
        """
        self._clock = pygame.time.Clock()
        self._speed_cache = 0
        self._angle_cache = 0
        self._control = lgsvl.VehicleControl()
        pool = ConnectionPool(host='localhost', port=6379, decode_responses=True)
        self.rd = Redis(connection_pool=pool)

    def parse_events(self, timestamp, input_data):
        """
        Calculate new vehicle controls based on input msg
        """
        msg = self.rd.hgetall('autopilot')
        # msg = {'angle_ori': 10., 'speed': 10}  # for test only
        if msg is not None and len(msg) > 0:
            angle = round(float(msg['angle_ori']), 3)
            if -1.0 < angle < 1.0:
                angle = 0
            self._angle_cache = angle
            self._speed_cache = float(msg['speed'])


        self._control.steering = self._angle_cache / 70
        _pid = PID(1.0, 0.1, 0.05, setpoint=self._speed_cache / 3.6)
        self._control.steering = np.clip(self._control.steering, -1., 1.)
        acceleration = _pid(input_data['ego_state'].speed)
        if acceleration >= 0.0:
            self._control.throttle = acceleration
            self._control.braking = 0
        else:
            self._control.throttle = 0
            self._control.braking = abs(acceleration)
        self._control.reverse = False
        self._control.handbrake = False

        # optional
        self._control.headlights = None  # int, 0=off, 1=low, 2=high beams
        self._control.windshield_wipers = None  # int, 0=off, 1-3=on
        self._control.turn_signal_left = None  # bool
        self._control.turn_signal_right = None  # bool

        return {'control': self._control, 'target_speed': self._speed_cache}


