#!/usr/bin/env python

"""
Module used to connect agent to bridge.
"""
import os
import time
import subprocess

class BridgeConnector(object):

    """
    Pure static class used to connect bridge and agent.
    """

    @staticmethod
    def connect_and_log(ego_vehicles, scene, agent_name):
        """
        connect ego vehicles to bridge.
        :param ego_vehicles: the list of ego vehicles.
        :param scene: the running scene.
        :param agent_name: the Agent type.
        TODO: need to run Apollo docker first, will add ROS
        """
        # set up listen subprocess
        bridge_dir = os.path.join(os.environ.get('ACHILLES_ROOT'), 'achilles/utils')
        command = 'sudo {}/dev_listen_pds_ADchecker.sh record_scene:{}_AD:{}'.format(bridge_dir, scene, agent_name)
        p_listen = subprocess.Popen(command, shell=True)

        for ego in ego_vehicles:
            if ego.get_bridge_type() == 'CyberRT':
                print("Connecting to CyberRT")
                ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)
                while not ego.bridge_connected:
                    time.sleep(1)
                print("Bridge connected")
        return p_listen


