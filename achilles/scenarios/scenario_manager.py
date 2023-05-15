#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the ScenarioManager implementations.
It must not be modified and is for reference only!
"""

from __future__ import print_function
import signal
import time

import lgsvl

from achilles.autoagents.agent_wrapper import AgentWrapper, AgentError
from achilles.utils.timer import GameTime
from achilles.utils.watchdog import Watchdog
from achilles.utils.result_writer import ResultOutputProvider


class ScenarioManager(object):

    """
    Basic scenario manager class. This class holds all functionality
    required to start, run and stop a scenario.

    The user must not modify this class.

    To use the ScenarioManager:
    1. Create an object via manager = ScenarioManager()
    2. Load a scenario via manager.load_scenario()
    3. Trigger the execution of the scenario manager.run_scenario()
       This function is designed to explicitly control start and end of
       the scenario execution
    4. If needed, cleanup with manager.stop_scenario()
    """

    def __init__(self, timeout):
        """
        Setups up the parameters, which will be filled at load_scenario()
        """
        self.ego_vehicles = None
        self.npc_vehicles = None

        self._agent = None
        self._running = False
        self._timestamp_last_run = 0.0
        self._timeout = float(timeout)

        # Used to detect if the simulation is down
        watchdog_timeout = max(5, self._timeout - 2)
        self._watchdog = Watchdog(watchdog_timeout)

        # Avoid the agent from freezing the simulation
        agent_timeout = watchdog_timeout - 1
        # self._agent_watchdog = Watchdog(agent_timeout)

        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.end_game_time = None

        # Use the callback_id inside the signal handler to allow external interrupts
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        self._running = False

    def cleanup(self):
        """
        Reset all parameters
        """
        self._timestamp_last_run = 0.0
        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.end_game_time = None
        self.ego_vehicles = None
        self.npc_vehicles = None
        self._agent = None

    def load_scenario(self, client, agent, ego_vehicles, npc_vehicles):
        """
        Load a new scenario
        """

        GameTime.restart()
        self._client = client
        self.ego_vehicles = ego_vehicles
        self.npc_vehicles = npc_vehicles
        self._agent = AgentWrapper(agent, self.ego_vehicles[0])

        sensor_list = ['velodyne', 'Main Camera', 'Telephoto Camera', 'GPS', 'IMU', 'rgb', 'rgb_left', 'rgb_right']
        self._agent.setup_sensors(self.ego_vehicles[0], sensor_list)

    def run_scenario(self, total_sim_duration):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        self.start_system_time = time.time()
        self.start_game_time = GameTime.get_time()

        self._watchdog.start()
        self._running = True

        self._client.run(time_limit=0.01)
        while self._running:
            timestamp = self._client.current_time
            frame = self._client.current_frame
            if timestamp >= total_sim_duration:
                self._running = False
                break
            self._tick_scenario(timestamp, frame)
            self._client.run(time_limit=0.01)
            time.sleep(0.1)

    def _tick_scenario(self, timestamp, frame):
        """
        Run next tick of scenario and the agent and tick the world.
        """

        if self._timestamp_last_run < timestamp and self._running:
            self._timestamp_last_run = timestamp

            self._watchdog.update()
            GameTime.on_svl_tick(timestamp, frame)

            try:
                ego_action = self._agent()

            except Exception as e:
                self._running = False
                raise AgentError(e)

            self.ego_vehicles[0].apply_control(ego_action, sticky=True)

    def get_running_status(self):
        """
        returns:
           bool: False if watchdog exception occured, True otherwise
        """
        return self._watchdog.get_status()

    def stop_scenario(self):
        """
        This function triggers a proper termination of a scenario
        """
        self._watchdog.stop()

        self.end_system_time = time.time()
        self.end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - self.start_system_time
        self.scenario_duration_game = self.end_game_time - self.start_game_time

        if self.get_running_status():

            if self._agent is not None:
                self._agent.cleanup()
                self._agent = None

            self.analyze_scenario()

    def analyze_scenario(self):
        """
        Analyzes and prints the results of the route
        """
        # TODO
        print("Done")
        # global_result = '\033[92m'+'SUCCESS'+'\033[0m'
        # global_result = '\033[91m'+'FAILURE'+'\033[0m'

        # ResultOutputProvider(self, global_result)
