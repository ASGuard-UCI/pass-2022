#!/usr/bin/env python

from __future__ import print_function

import math
import numpy as np
from enum import Enum

import lgsvl
from lgsvl.utils import transform_to_matrix

from achilles.autoagents.modules.obstacle_detector import Obstacle
from achilles.autoagents.modules.obstacle_detector import ObstacleType
from achilles.autoagents.modules.obstacle_locator import CameraModel


def project_3D_to_2D(proj_mat, rect_mat, corners_3D):
    proj_mat = np.array(proj_mat).reshape((3, 4))

    rect_3x3 = np.array(rect_mat).reshape((3, 3))
    rect_mat = np.zeros([4, 4], dtype=rect_3x3.dtype)
    rect_mat[3, 3] = 1
    rect_mat[:3, :3] = rect_3x3

    corners_2D = np.dot(rect_mat, np.vstack((corners_3D, np.ones([1, 8]))))
    corners_2D = np.dot(proj_mat, corners_2D)
    corners_2D[0, :] = corners_2D[0, :] / corners_2D[2, :]
    corners_2D[1, :] = corners_2D[1, :] / corners_2D[2, :]
    corners_2D = np.delete(corners_2D, (2), axis=0)

    return corners_2D


def get_corners_3D(location, rotation_y, dimension):
    # Returns a bounding box around a 3D location in the camera space
    # Note: the location is at the center of the bounding box
    h, w, l = dimension[0], dimension[1], dimension[2]
    x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
    # y_corners = [0, 0, 0, 0, -h, -h, -h, -h]
    y_corners = [h/2, h/2, h/2, h/2, -h/2, -h/2, -h/2, -h/2]
    z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]

    rot_mat = [
            [math.cos(rotation_y), 0, math.sin(rotation_y)],
            [0, 1, 0],
            [-math.sin(rotation_y), 0, math.cos(rotation_y)],
            ]

    corners_3D = np.dot(rot_mat, [x_corners, y_corners, z_corners])
    corners_3D[0, :] += location[0]
    corners_3D[1, :] += location[1]
    corners_3D[2, :] += location[2]

    return corners_3D


class HDMap(object):

    """
    TODO: document me!
    """

    # _stop_sign_pos = [-643.74, 372.863]  # SVL v2021.2.2
    _stop_sign_pos = lgsvl.Vector(-714.316, 12.03, -207.78)  # SVL v2021.3
    _stop_sign_rot = lgsvl.Vector(0., -10.05, 0.)

    _stop_sign_size = [0.7, 0.7]

    def __init__(self):
        self._stop_sign_trans = lgsvl.Transform(
                position=self._stop_sign_pos,
                rotation=self._stop_sign_rot)
        self._stop_sign_mat = transform_to_matrix(self._stop_sign_trans)

    def query_stop_sign_obs_front(self, input_data):
        ego_trans = input_data['ego_state'].transform
        cam_params = input_data['main_camera_params']
        cam_model = CameraModel(cam_params['width'], cam_params['height'], cam_params['fov'])

        projection_matrix = cam_model.projection_matrix()
        rectification_matrix = cam_model.rectification_matrix()

        cam_mat = transform_to_matrix(cam_params['trans'])
        ego_mat = transform_to_matrix(ego_trans)
        tf_mat = np.dot(np.linalg.inv(ego_mat), np.linalg.inv(cam_mat))
        stop_sign_tf = np.dot(self._stop_sign_mat, tf_mat)
        stop_sign_pos_cam_3d = (stop_sign_tf[3][0], -stop_sign_tf[3][1], stop_sign_tf[3][2])
        stop_sign_yaw = np.arctan2(stop_sign_tf[2][0], stop_sign_tf[0][0]) - (np.pi / 2)
        if stop_sign_yaw < -np.pi:
            stop_sign_yaw += 2 * np.pi

        h = w = 0.7
        l = 0.01

        corners_3d = get_corners_3D(stop_sign_pos_cam_3d, stop_sign_yaw, (h, w, l))
        corners_2d = project_3D_to_2D(projection_matrix, rectification_matrix, corners_3d)

        p_min, p_max = corners_2d[:, 0], corners_2d[:, 0]
        for i in range(corners_2d.shape[1]):
            p_min = np.minimum(p_min, corners_2d[:, i])
            p_max = np.maximum(p_max, corners_2d[:, i])
        left, top = p_min
        right, bottom = p_max
        bbox = [int(top), int(left), int(bottom), int(right)]
        print("Stop sign from HDMap", bbox)

        std_score = 0.5
        coco_clss_stop_sign = 13
        obs_stop_sign = Obstacle(bbox, coco_clss_stop_sign, std_score)

        return [obs_stop_sign]

    def query_distance_to_stop_sign(self, input_data):
        ego_pos = input_data['ego_state'].transform.position
        waypt_vec = np.array([ego_pos.x, ego_pos.z])

        offset = 0.
        stop_sign_vec = np.array([self._stop_sign_pos.x, self._stop_sign_pos.z])
        dist = np.linalg.norm(stop_sign_vec - waypt_vec)
        dist = max(0.0, dist - offset)

        return dist

