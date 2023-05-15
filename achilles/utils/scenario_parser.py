#!/usr/bin/env python

"""
Module used to parse all the route and scenario configuration parameters.
"""
from collections import OrderedDict
import json
import math

import lgsvl

from achilles.utils.file_parser import parse_json

# TODO  check this threshold, it could be a bit larger but not so large that we cluster scenarios.
TRIGGER_THRESHOLD = 2.0  # Threshold to say if a trigger position is new or repeated, works for matching positions
TRIGGER_ANGLE_THRESHOLD = 10  # Threshold to say if two angles can be considering matching when matching transforms.


class ScenarioParser(object):

    """
    Pure static class used to parse all the route and scenario configuration parameters.
    """

    @staticmethod
    def parse_annotations_file(annotation_filename):
        """
        Return the annotations of which positions where the scenarios are going to happen.
        :param annotation_filename: the filename for the anotations file
        :return:
        """
        with open(annotation_filename, 'r') as f:
            annotation_dict = json.loads(f.read(), object_pairs_hook=OrderedDict)

        final_dict = OrderedDict()

        for town_dict in annotation_dict['available_scenarios']:
            final_dict.update(town_dict)

        return final_dict  # the file has a current maps name that is an one element vec

    @staticmethod
    def parse_scenario_file(route_file, scenario_file):
        """
        Returns a list of route elements.
        :param route_filename: the path to a set of routes.
        :param single_route: If set, only this route shall be returned
        :return: List of dicts containing the waypoints, id and town of the routes
        """

        vse_config = parse_json(scenario_file)
        route_config = parse_json(route_file)

        scene = vse_config['map']['name']

        ego_vehicles = []
        npc_vehicles = []
        for agent in vse_config['agents']:
            if agent['type'] == 1:
                ego_vehicles.append({
                    'name': agent['sensorsConfigurationId'],
                    'position': agent['transform']['position'],
                    'rotation': agent['transform']['rotation']
                    })
            else:
                npc_behavior = ''
                npc_is_lane_change = False
                npc_max_speed = float('inf')
                waypoints = []
                if 'behaviour' in agent.keys() and agent['behaviour']['name'] == 'NPCLaneFollowBehaviour':
                    npc_behavior = 'NPCLaneFollowBehavior'
                    npc_is_lane_change = agent['behaviour']['parameters']['isLaneChange']
                    npc_max_speed = agent['behaviour']['parameters']['maxSpeed']
                elif 'behaviour' in agent.keys() and agent['behaviour']['name'] == 'NPCWaypointBehaviour':
                    npc_behavior = 'NPCWaypointBehavior'
                    for wp in agent['waypoints']:
                        des = lgsvl.Vector(wp['position']['x'], wp['position']['y'], wp['position']['z'])
                        ang = lgsvl.Vector(wp['angle']['x'], wp['angle']['y'], wp['angle']['z'])
                        waypoints.append(lgsvl.DriveWaypoint(
                            des, wp['speed'], ang, idle=0, deactivate=False, trigger_distance=100))
                npc_vehicles.append({
                    'name': agent['variant'],
                    'position': agent['transform']['position'],
                    'rotation': agent['transform']['rotation'],
                    'behavior': npc_behavior,
                    'is_lane_change': npc_is_lane_change,
                    'max_speed': npc_max_speed,
                    'waypoints': waypoints
                })

        ego_routes = []
        for i in range(len(ego_vehicles)):
            # waypoints = []
            # for wp in route_config[i]:
            #     waypoints.append([wp["position"][2], wp["position"][0]])
            # ego_routes.append(waypoints)
            ego_routes.append(route_config[i])

        config = {
                'scene': scene,
                'ego_vehicles': ego_vehicles,
                'npc_vehicles': npc_vehicles,
                'ego_routes': ego_routes
                }

        return config

    @staticmethod
    def check_trigger_position(new_trigger, existing_triggers):
        """
        Check if this trigger position already exists or if it is a new one.
        :param new_trigger:
        :param existing_triggers:
        :return:
        """

        for trigger_id in existing_triggers.keys():
            trigger = existing_triggers[trigger_id]
            dx = trigger['x'] - new_trigger['x']
            dy = trigger['y'] - new_trigger['y']
            distance = math.sqrt(dx * dx + dy * dy)

            dyaw = (trigger['yaw'] - new_trigger['yaw']) % 360
            if distance < TRIGGER_THRESHOLD \
                and (dyaw < TRIGGER_ANGLE_THRESHOLD or dyaw > (360 - TRIGGER_ANGLE_THRESHOLD)):
                return trigger_id

        return None

    @staticmethod
    def convert_waypoint_float(waypoint):
        """
        Convert waypoint values to float
        """
        waypoint['x'] = float(waypoint['x'])
        waypoint['y'] = float(waypoint['y'])
        waypoint['z'] = float(waypoint['z'])
        waypoint['yaw'] = float(waypoint['yaw'])
