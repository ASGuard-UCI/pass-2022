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
import sys
import time

import py_trees
import carla
import multiprocessing
import argparse

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.watchdog import Watchdog
from srunner.scenariomanager.traffic_events import TrafficEventType

from leaderboard.autoagents.agent_wrapper import AgentWrapper, AgentError
from leaderboard.envs.sensor_interface import SensorReceivedNoData
from leaderboard.utils.autosec_result_writer_multi_agents import ResultOutputProvider

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


    def __init__(self, timeout, debug_mode=False):
        """
        Setups up the parameters, which will be filled at load_scenario()
        """
        # self.scenario = None
        self.scenarios = []
        # self.scenario_tree = None
        self.scenario_trees = []
        self.ego_idx = []
        self.scenario_class = None
        # self.ego_vehicles = None
        self.ego_vehicles = []
        self.other_actors = None

        self._debug_mode = debug_mode
        self._agents = []
        self._running = False
        self._timestamp_last_run = 0.0
        self._timeout = float(timeout)

        self.scenario_duration_system = 0.0
        self.scenario_duration_game = 0.0
        self.start_system_time = None
        self.end_system_time = None
        self.end_game_time = None

        self._watchdog = None
        self._agent_watchdog = None
        self.background_process = None

        # Use the callback_id inside the signal handler to allow external interrupts
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        if self._agent_watchdog and not self._agent_watchdog.get_status():
            raise RuntimeError("Agent took longer than {}s to send its command".format(self._timeout))
        elif self._watchdog and not self._watchdog.get_status():
            raise RuntimeError("The simulation took longer than {}s to update".format(self._timeout))
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

        self._spectator = None
        self._watchdog = None
        self._agent_watchdog = None

    def load_scenario(self, scenario, agent, ego_idx):
        """
        Load a new scenario
        ego_idx: the index of the ego car, e.g., 0 for the first ego car
        """

        GameTime.restart()
        self._agents.append(AgentWrapper(agent))
        self.scenario_class = scenario
        # self.scenario = scenario.scenario
        self.scenarios.append(scenario.scenario)
        # self.scenario_tree = self.scenario.scenario_tree
        self.scenario_trees.append(scenario.scenario.scenario_tree)
        self.ego_vehicles.append(scenario.ego_vehicles[0])  # the ego vehicles are from my_route_scenario, not from scenario_config
        self.other_actors = scenario.other_actors
        # self.repetition_number = rep_number
        self.ego_idx.append(ego_idx)

        self._spectator = CarlaDataProvider.get_world().get_spectator()

        # To print the scenario tree uncomment the next line
        # py_trees.display.render_dot_tree(self.scenario_tree)

        # self._agent.setup_sensors(self.ego_vehicles[0], self._debug_mode)
        # setup the ego to be controlled
        self._agents[-1].setup_sensors(self.ego_vehicles[-1], self._debug_mode)
        self._agents[-1].setup_ego_id(self.ego_vehicles[-1].id)  # this id is different from idx, it is carla actor.id

    def run_wheel_process(self):
        # after load scenario, run the wheel control in the background
        from my_wheel_control import game_loop
        args_wheel = argparse.ArgumentParser(description='CARLA Wheel Control Client')
        args_wheel.host = '127.0.0.1'
        args_wheel.port = 2000
        args_wheel.width, args_wheel.height = 1920, 1080
        args_wheel.scenario = 'OfflineChallenge_1'
        args_wheel.autopilot = False
        args_wheel.rolename = 'hero'
        args_wheel.keep_ego_vehicle = False

        self.p_conn, self.c_conn = multiprocessing.Pipe()
        args_wheel.c_conn = self.c_conn  # child connection
        self.background_process = multiprocessing.Process(name='background_process', target=game_loop,
                                                          args=(args_wheel,))
        self.background_process.daemon = False
        self.background_process.start()

    def run_wheel_process_win(self):
        # after load scenario, run the wheel control in the background
        from my_wheel_control_win import game_loop, GameArgs
        args_wheel = GameArgs()
        args_wheel.host = '127.0.0.1'
        args_wheel.port = 2000
        args_wheel.width, args_wheel.height = 1920, 1080
        args_wheel.scenario = 'OfflineChallenge_1'
        args_wheel.autopilot = False
        args_wheel.rolename = 'hero'
        args_wheel.keep_ego_vehicle = False

        self.p_conn, self.c_conn = multiprocessing.Pipe()
        args_wheel.c_conn = self.c_conn  # child connection
        self.background_process = multiprocessing.Process(name='background_process', target=game_loop,
                                                          args=(args_wheel,))
        self.background_process.daemon = False
        self.background_process.start()

    def run_keyboard_process_win(self):
        # after load scenario, run the wheel control in the background
        from my_manual_control import game_loop, GameArgs
        args_wheel = GameArgs()
        args_wheel.host = '127.0.0.1'
        args_wheel.port = 2000
        args_wheel.width, args_wheel.height = 1920, 1080
        args_wheel.scenario = 'OfflineChallenge_1'
        args_wheel.autopilot = False
        args_wheel.rolename = 'hero'
        args_wheel.keep_ego_vehicle = False

        self.p_conn, self.c_conn = multiprocessing.Pipe()
        args_wheel.c_conn = self.c_conn  # child connection
        self.background_process = multiprocessing.Process(name='background_process', target=game_loop,
                                                          args=(args_wheel,))
        self.background_process.daemon = False
        self.background_process.start()


    def run_scenario(self):
        """
        Trigger the start of the scenario and wait for it to finish/fail
        """
        self.start_system_time = time.time()
        self.start_game_time = GameTime.get_time()


        self._running = True

        while self._running:
            timestamp = None
            world = CarlaDataProvider.get_world()
            if world:
                snapshot = world.get_snapshot()
                if snapshot:
                    timestamp = snapshot.timestamp
            if timestamp:
                self._tick_scenario(timestamp)
            # update checkpoint info
            _checkpoint_num = 0
            for _scenario in self.scenarios:
                for criterion in _scenario.get_criteria():
                    _name = criterion.name
                    if _name == "CollisionTest":
                        for collision_event in criterion.list_traffic_events:
                            collision_type = collision_event.get_type()
                            if collision_type == TrafficEventType.COLLISION_PEDESTRIAN or collision_type == TrafficEventType.COLLISION_STATIC:
                                _checkpoint_num += 1
            if self.background_process is not None:
                if self.background_process.is_alive():
                    self.p_conn.send(_checkpoint_num)

    def _tick_scenario(self, timestamp):
        """
        Run next tick of scenario and the agent and tick the world.
        support multi-agents/egos
        """

        if self._timestamp_last_run < timestamp.elapsed_seconds and self._running:
            self._timestamp_last_run = timestamp.elapsed_seconds

            # self._watchdog.update()
            # Update game time and actor information
            GameTime.on_carla_tick(timestamp)
            CarlaDataProvider.on_carla_tick()
            # self._watchdog.pause()

            try:
                # ego_action = self._agent()
                ego_action = {}
                for _ego_idx in self.ego_idx:
                    ego_action[_ego_idx] = self._agents[_ego_idx]()

            # Special exception inside the agent that isn't caused by the agent
            except SensorReceivedNoData as e:
                raise RuntimeError(e)

            except Exception as e:
                raise AgentError(e)

            # self.ego_vehicles[0].apply_control(ego_action)
            # apply control to all egos
            for _ego_idx in self.ego_idx:
                self.ego_vehicles[_ego_idx].apply_control(ego_action[_ego_idx])

            # Tick scenario
            # self.scenario_tree.tick_once()
            #
            # if self._debug_mode:
            #     print("\n")
            #     py_trees.display.print_ascii_tree(
            #         self.scenario_tree, show_status=True)
            #     sys.stdout.flush()
            #
            # if self.scenario_tree.status != py_trees.common.Status.RUNNING:
            #     self._running = False

            # when all ego finish their routes, stop
            _total_stop_status = []
            for _st in self.scenario_trees:
                _st.tick_once()
                if _st.status != py_trees.common.Status.RUNNING:
                    _total_stop_status.append(True)
                else:
                    _total_stop_status.append(False)
            if all(_total_stop_status):
                self._running = False

            ego0_trans = self.ego_vehicles[2].get_transform()
            self._spectator.set_transform(carla.Transform(ego0_trans.location + carla.Location(z=50),
                                                          carla.Rotation(pitch=-90)))

        # if self._running and self.get_running_status():
        #     CarlaDataProvider.get_world().tick(self._timeout)
        if self._running:
            CarlaDataProvider.get_world().tick(self._timeout)

    def get_running_status(self):
        """
        returns:
           bool: False if watchdog exception occured, True otherwise
        """
        if self._watchdog:
            return self._watchdog.get_status()
        return False

    def stop_egos(self):
        """
        This function triggers a proper termination of egos (old stop_scenario)
        """
        # if self._watchdog:
        #     self._watchdog.stop()

        # if self._agent_watchdog:
        #     self._agent_watchdog.stop()

        if self.background_process is not None:
            self.background_process.terminate()
            self.background_process.join()
            self.background_process.close()

        self.end_system_time = time.time()
        self.end_game_time = GameTime.get_time()

        self.scenario_duration_system = self.end_system_time - self.start_system_time
        self.scenario_duration_game = self.end_game_time - self.start_game_time

        # if self.scenario is not None:
        #     self.scenario.terminate()
        #
        # if self._agent is not None:
        #     self._agent.cleanup()
        #     self._agent = None
        for _scenario in self.scenarios:
            if _scenario is not None:
                _scenario.terminate()
        for _agent in self._agents:
            if _agent is not None:
                _agent.cleanup()
        # self.scenarios = []
        # self._agents = []


        self.analyze_scenario()

    def analyze_scenario(self):
        """
        Analyzes and prints the results of the route
        """
        global_result = '\033[91m'+'No Pengci'+'\033[0m'

        for _scenario in self.scenarios:
            for criterion in _scenario.get_criteria():
                if criterion.test_status != "SUCCESS":
                    global_result = '\033[92m'+'Pengci Succeed!'+'\033[0m'
                    break

        for _scenario in self.scenarios:
            if _scenario.timeout_node.timeout:
                global_result = '\033[92m'+'TimeOut'+'\033[0m'
                break

        ResultOutputProvider(self, global_result)
