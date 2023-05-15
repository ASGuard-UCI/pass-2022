#!/usr/bin/env python3

import os
import argparse
import logging
import lgsvl
import copy
import cv2
import json
import time
import numpy as np


def parse_json(json_file):
    with open(json_file, 'r') as f:
        dat = json.load(f)
    return dat


def interp_100hz_from_20hz(arr):
    ratio = 5
    times_20hz = [float(k) for k in range(len(arr))]
    times_100hz = [float(k)/ratio for k in range((len(arr) - 1) * ratio + 1)]
    arr_200hz = np.interp(times_100hz, times_20hz, arr)
    return arr_200hz


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--route', type=str, default="route_stop_sign_20hz.json", help='Route file, e.g., route_stop_sign.json')

    args = parser.parse_args()
    route = parse_json(args.route)

    route_pos_xs = []
    route_pos_ys = []
    route_pos_zs = []
    route_rot_xs = []
    route_rot_ys = []
    route_rot_zs = []
    for wp in route:
        route_pos_xs.append(wp["position"][0])
        route_pos_ys.append(wp["position"][1])
        route_pos_zs.append(wp["position"][2])
        route_rot_xs.append(wp["rotation"][0])
        route_rot_ys.append(wp["rotation"][1])
        route_rot_zs.append(wp["rotation"][2])


    route_pos_xs_100hz = interp_100hz_from_20hz(np.array(route_pos_xs))
    route_pos_ys_100hz = interp_100hz_from_20hz(np.array(route_pos_ys))
    route_pos_zs_100hz = interp_100hz_from_20hz(np.array(route_pos_zs))
    route_rot_xs_100hz = interp_100hz_from_20hz(np.array(route_rot_xs))
    route_rot_ys_100hz = interp_100hz_from_20hz(np.array(route_rot_ys))
    route_rot_zs_100hz = interp_100hz_from_20hz(np.array(route_rot_zs))

    route_100hz = []
    for i in range(len(route_pos_xs_100hz)):
        route_100hz.append({"position":[
                    route_pos_xs_100hz[i],
                    route_pos_ys_100hz[i],
                    route_pos_zs_100hz[i],
                ], "rotation":[
                    route_rot_xs_100hz[i],
                    route_rot_ys_100hz[i],
                    route_rot_zs_100hz[i],
                ]})
    with open("route_stop_sign_100hz.json", 'w') as f:
        json.dump(route_100hz, f, indent=2)
