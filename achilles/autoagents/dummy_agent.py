#!/usr/bin/env python

"""
This module provides a dummy agent to control the ego vehicle
"""

from __future__ import print_function

# import carla
import lgsvl

from achilles.autoagents.autonomous_agent import AutonomousAgent

def get_entry_point():
    return 'DummyAgent'

class DummyAgent(AutonomousAgent):

    """
    Dummy autonomous agent to control the ego vehicle
    """

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        pass

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

        sensors = [
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
             'width': 800, 'height': 600, 'fov': 100, 'id': 'Center'},
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0,
             'width': 800, 'height': 600, 'fov': 100, 'id': 'Left'},
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 45.0,
             'width': 800, 'height': 600, 'fov': 100, 'id': 'Right'},
            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': -45.0, 'id': 'LIDAR'},
            {'type': 'sensor.other.radar', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': -45.0, 'fov': 30, 'id': 'RADAR'},
            {'type': 'sensor.other.gnss', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'id': 'GPS'},
            {'type': 'sensor.other.imu', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': -45.0, 'id': 'IMU'},
            {'type': 'sensor.opendrive_map', 'reading_frequency': 1, 'id': 'OpenDRIVE'},
        ]

        return sensors

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """
        print("=====================>")
        for key, val in input_data.items():
            if hasattr(val, 'shape'):
                print("[{}] with shape {}".format(key, val.shape))
            else:
                print("[{}] with value {}".format(key, val))
        print("<=====================")

        # DO SOMETHING SMART

        # RETURN CONTROL
        control = lgsvl.VehicleControl()
        control.steering = 0.0
        control.throttle = 1.0
        control.braking = 0.0
        control.reverse = False
        control.handbrake = False

        # optional
        control.headlights = None  # int, 0=off, 1=low, 2=high beams
        control.windshield_wipers = None  # int, 0=off, 1-3=on
        control.turn_signal_left = None  # bool
        control.turn_signal_right = None  # bool

        return control
