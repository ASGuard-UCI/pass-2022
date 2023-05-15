#!/usr/bin/env python

"""
Achilles Evaluator

Provisional code to evaluate attacks/defenses under AD stacks
"""
from __future__ import print_function

import traceback
import argparse
from argparse import RawTextHelpFormatter
from datetime import datetime
from distutils.version import LooseVersion
import importlib
import os
import pkg_resources
import sys
import lgsvl
import signal
import subprocess

from achilles.utils.timer import GameTime
from achilles.utils.scenario_parser import ScenarioParser
from achilles.utils.file_parser import parse_yaml
from achilles.utils.bridge_connecter import BridgeConnector
from achilles.scenarios.scenario_manager import ScenarioManager
from achilles.autoagents.agent_wrapper import AgentWrapper, AgentError


class AchillesEvaluator(object):

    """
    Entry point for evaluating the attacks or defenses under a certain scenario
    """

    def __init__(self, args):
        """
        Setup SVL client and world
        Setup ScenarioManager
        """
        self.ego_vehicles = []
        self.npc_vehicles = []

        self.sensors = None
        self.total_sim_duration = 60.0

        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        self.client = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", args.host), int(args.port))

        dist = pkg_resources.get_distribution("lgsvl")
        if LooseVersion(dist.version) < LooseVersion('2021.1'):
            raise ImportError("SVL PythonAPI version 2021.1 or newer required. SVL PythonAPI version found: {}".format(dist))

        # Load agent
        module_name = os.path.basename(args.agent).split('.')[0]
        sys.path.insert(0, os.path.dirname(args.agent))
        self.module_agent = importlib.import_module(module_name)

        # Create the ScenarioManager
        self.manager = ScenarioManager(args.timeout)

        # Time control for summary purposes
        self._start_time = GameTime.get_time()
        self._end_time = None

        # Create the agent timer
        signal.signal(signal.SIGINT, self._signal_handler)

        # bridge process
        self.p_listen = None

    def _signal_handler(self, signum, frame):
        """
        Terminate scenario ticking when receiving a signal interrupt
        """
        print(f"[{os.path.basename(__file__)}] Caught SIGINT, terminating...")
        if self.manager:
            self.manager.signal_handler(signum, frame)
        self._cleanup()
        sys.exit()

    def __del__(self):
        """
        Cleanup and delete actors and ScenarioManager
        """

        self._cleanup()
        if hasattr(self, 'manager') and self.manager:
            del self.manager

    def _cleanup(self):
        """
        Remove and destroy all actors
        """

        if self.p_listen:
            if not self.p_listen.poll():
                bridge_dir = os.path.join(os.environ.get('ACHILLES_ROOT'), 'achilles/utils')
                kill_command = 'sudo {}/kill_listen_ADchecker.sh'.format(bridge_dir)
                p_kill = subprocess.Popen(kill_command, shell=True)
                if p_kill.poll() == 0:
                    self.p_listen.kill()

        if self.manager:
            self.manager.cleanup()

        self.ego_vehicles = []
        self.npc_vehicles = []

        if hasattr(self, 'agent_instance') and self.agent_instance:
            self.agent_instance.destroy()
            self.agent_instance = None

        # Note: if the machine is powerful enough, consider enable
        # below line to free up more resources at end of simulation.
        # Currently disabled since the simulator crashes from
        # time to time due to limited CPU computational power. Once
        # the simulation crashes, the below call will keep blocking.
        # self.client.reset()

    def _prepare_ego_vehicles(self, ego_vehicles, agent_conf):
        """
        Spawn or update the ego vehicles
        """
        for vehicle in ego_vehicles:
            ego_name = vehicle['name']
            ego_state = lgsvl.AgentState()
            ego_state.transform.position = lgsvl.Vector(vehicle['position']['x'], vehicle['position']['y'], vehicle['position']['z'])
            ego_state.transform.rotation = lgsvl.Vector(vehicle['rotation']['x'], vehicle['rotation']['y'], vehicle['rotation']['z'])

            # Initialize ego vehicle speed
            forward = lgsvl.utils.transform_to_forward(ego_state.transform)
            ego_state.velocity = agent_conf['target_speed'] * forward

            ego = self.client.add_agent(ego_name, lgsvl.AgentType.EGO, ego_state)
            self.ego_vehicles.append(ego)

    def _prepare_npc_vehicles(self, npc_vehicles):
        """
        Spawn or update the npc vehicles
        """
        for vehicle in npc_vehicles:
            npc_name = vehicle['name']
            npc_state = lgsvl.AgentState()
            npc_state.transform.position = lgsvl.Vector(vehicle['position']['x'], vehicle['position']['y'], vehicle['position']['z'])
            npc_state.transform.rotation = lgsvl.Vector(vehicle['rotation']['x'], vehicle['rotation']['y'], vehicle['rotation']['z'])
            npc = self.client.add_agent(npc_name, lgsvl.AgentType.NPC, npc_state)
            if vehicle['behavior'] == 'NPCLaneFollowBehavior':
                npc.follow_closest_lane(True, vehicle['max_speed'], vehicle['is_lane_change'])
            elif vehicle['behavior'] == 'NPCWaypointBehavior' and len(vehicle['waypoints']):
                npc.follow(vehicle['waypoints'], loop=True)
            self.npc_vehicles.append(npc)

    def _physical_add_object(self, is_3d, path, transform):
        state = lgsvl.ObjectState()
        state.transform.position = lgsvl.Vector(
                transform['position'][0],
                transform['position'][1],
                transform['position'][2])
        state.transform.rotation = lgsvl.Vector(
                transform['rotation'][0],
                transform['rotation'][1],
                transform['rotation'][2])
        state.velocity = lgsvl.Vector(0, 0, 0)
        state.angular_velocity = lgsvl.Vector(0, 0, 0)
        patch = self.client.controllable_add("Patch", state)
        print("Added Patch:", patch)

        args = f"texture={path}"
        patch.control(args)

        return True

    def _load_and_wait_for_scene(self, scene):
        """
        Load a new SVL world and provide data to CarlaDataProvider
        """

        # SVL does not provide a world handle
        if self.client.current_scene == scene:
            self.client.reset()
        else:
            self.client.load(scene)

        if self.client.current_scene != scene:
            raise Exception("The SVL server uses the wrong map!"
                            "This scenario requires to use map {}".format(scene))

    def _load_security_config(self, sec_config):
        """
        Apply attacks & defenses
        """

        # TODO: merge the patch position, rotation, path in configuration
        stop_sign_poster = os.path.join(os.environ.get('ACHILLES_ROOT'), "achilles/data/stop_sign_poster_shapeshifter.png")
        trans = {'position': [-714.316, 12.03, -207.78], 'rotation': [0, -10.05, 0]}
        self._physical_add_object(False, stop_sign_poster, trans)

    def _load_and_run_scenario(self, args, config):
        """
        Load and run the scenario given by config.

        Depending on what code fails, the simulation will either stop the route and
        continue from the next one, or report a crash and stop.
        """
        crash_message = ""
        entry_status = "Started"

        print("\n\033[1m========= Preparing {} =========".format(config['scene']))
        print("> Setting up the agent\033[0m")

        if args.agent_config:
            agent_conf = parse_yaml(args.agent_config)
            self.total_sim_duration = float(agent_conf.get('total_sim_duration'))

        # Set up the user's agent, and the timer to avoid freezing the simulation
        try:
            agent_class_name = getattr(self.module_agent, 'get_entry_point')()
            self.agent_instance = getattr(self.module_agent, agent_class_name)(args.agent_config, config['ego_routes'][0])

            # Check and store the sensors
            if not self.sensors:
                self.sensors = self.agent_instance.sensors()
                AgentWrapper.validate_sensor_configuration(self.sensors)

        except Exception as e:
            # The agent setup has failed -> start the next route
            print("\n\033[91mCould not set up the required agent:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Agent couldn't be set up"

            self._cleanup()
            return

        print("\033[1m> Loading the world\033[0m")

        # Load the world and the scenario
        try:
            self._load_and_wait_for_scene(config['scene'])
            self._load_security_config([])
            self._prepare_ego_vehicles(config['ego_vehicles'], agent_conf)
            self._prepare_npc_vehicles(config['npc_vehicles'])

            # Load scenario and run it
            self.manager.load_scenario(self.client, self.agent_instance, self.ego_vehicles, self.npc_vehicles)

        except Exception as e:
            # The scenario is wrong -> set the ejecution to crashed and stop
            print("\n\033[91mThe scenario could not be loaded:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Simulation crashed"
            entry_status = "Crashed"

            self._cleanup()
            sys.exit(-1)


        def is_docker():
            path = '/proc/self/cgroup'
            return (
                    os.path.exists('/.dockerenv') or
                    os.path.isfile(path) and any('docker' in line for line in open(path))
                    )

        if not is_docker():
            print("Currently not in any docker container")

            print("> Connecting to the bridge\033[0m")
            try:
                self.p_listen = BridgeConnector.connect_and_log(self.ego_vehicles, config['scene'], agent_class_name)
            except Exception as e:
                # The agent connection has failed -> start the next route
                print("\n\033[91mCould not connect to bridge:")
                print("> {}\033[0m\n".format(e))
                traceback.print_exc()

                crash_message = "Agent couldn't connect to bridge"

                self._cleanup()
                sys.exit(-1)

        print("\033[1m> Running the scenario\033[0m")

        # Run the scenario
        try:
            self.manager.run_scenario(self.total_sim_duration)

        except AgentError as e:
            # The agent has failed -> stop the route
            print("\n\033[91mStopping the route, the agent has crashed:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Agent crashed"

        except Exception as e:
            print("\n\033[91mError during the simulation:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Simulation crashed"
            entry_status = "Crashed"

        # Stop the scenario
        try:
            print("\033[1m> Stopping the route\033[0m")
            self.manager.stop_scenario()

            self._cleanup()

        except Exception as e:
            print("\n\033[91mFailed to stop the scenario, the statistics might be empty:")
            print("> {}\033[0m\n".format(e))
            traceback.print_exc()

            crash_message = "Simulation crashed"

        if crash_message == "Simulation crashed":
            sys.exit(-1)

    def run(self, args):
        """
        Run the simulation
        """

        config = ScenarioParser.parse_scenario_file(args.route, args.scenario)
        self._load_and_run_scenario(args, config)


def main():
    description = "SVL AD Leaderboard Evaluation: evaluate your Agent in SVL scenarios\n"

    # general parameters
    parser = argparse.ArgumentParser(description=description, formatter_class=RawTextHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1',
                        help='IP of the host server (default: localhost)')
    parser.add_argument('--port', default='8181', help='TCP port to listen to (default: 8181)')
    parser.add_argument('--timeout', default="10.0",
                        help='Set the SVL client timeout value in seconds')

    # simulation setup
    parser.add_argument('--scenario', help='SVL VSE config file.', required=True)
    parser.add_argument('--route', help='SVL route config file.', required=True)

    # agent-related options
    parser.add_argument("-a", "--agent", type=str, help="Path to Agent's py file to evaluate", required=True)
    parser.add_argument("--agent-config", type=str, help="Path to Agent's configuration file", default="")

    arguments = parser.parse_args()

    try:
        achilles_evaluator = AchillesEvaluator(arguments)
        achilles_evaluator.run(arguments)

    except Exception as e:
        traceback.print_exc()
    finally:
        del achilles_evaluator


if __name__ == '__main__':
    main()
