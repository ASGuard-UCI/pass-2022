#!/usr/bin/env python

"""
Base class for object detectors
"""
from __future__ import print_function

import numpy as np

from enum import Enum


class ObstacleType(Enum):
    NONE = 0
    STOP_SIGN = 1
    OBSTACLE = 2


class Obstacle(object):
    def __init__(self, bbox, coco_type=None, score=1.0):
        self._type = ObstacleType.NONE
        self._raw_type = 0
        if coco_type:
            if coco_type == 13:
                self._type = ObstacleType.STOP_SIGN
            elif coco_type == 0 or coco_type == 83:
                self._type = ObstacleType.NONE
            else:
                self._type = ObstacleType.OBSTACLE
            self._raw_type = coco_type

        # bbox convention: [top_left.h, top_left.w, bot_right.h, bot_right.w]
        self._bbox = bbox
        self._score = score

    def is_stop_sign(self):
        return self._type == ObstacleType.STOP_SIGN

    def is_obstacle(self):
        return self._type == ObstacleType.OBSTACLE

    def get_bbox(self):
        return self._bbox

    def get_score(self):
        return self._score

    def get_raw_type(self):
        return self._raw_type

    def get_type(self):
        return self._type


class ObstacleDetector(object):

    """
    TODO: document me!
    """

    def __init__(self):
        pass

    def run_step(self, input_data, timestamp):
        bbox = None
        return bbox

    @staticmethod
    def _nms(bounding_boxes, confidence_score, threshold=0.6):
        # If no bounding boxes, return empty list
        if len(bounding_boxes) == 0:
            return [], []

        # Bounding boxes
        boxes = np.array(bounding_boxes)

        # coordinates of bounding boxes
        start_x = boxes[:, 0]
        start_y = boxes[:, 1]
        end_x = boxes[:, 2]
        end_y = boxes[:, 3]

        # Confidence scores of bounding boxes
        score = np.array(confidence_score)

        # Picked bounding boxes
        picked_boxes = []
        picked_score = []

        # Compute areas of bounding boxes
        areas = (end_x - start_x + 1) * (end_y - start_y + 1)

        # Sort by confidence score of bounding boxes
        order = np.argsort(score)

        # Iterate bounding boxes
        while order.size > 0:
            # The index of largest confidence score
            index = order[-1]

            # Pick the bounding box with largest confidence score
            picked_boxes.append(bounding_boxes[index])
            picked_score.append(confidence_score[index])

            # Compute ordinates of intersection-over-union(IOU)
            x1 = np.maximum(start_x[index], start_x[order[:-1]])
            x2 = np.minimum(end_x[index], end_x[order[:-1]])
            y1 = np.maximum(start_y[index], start_y[order[:-1]])
            y2 = np.minimum(end_y[index], end_y[order[:-1]])

            # Compute areas of intersection-over-union
            w = np.maximum(0.0, x2 - x1 + 1)
            h = np.maximum(0.0, y2 - y1 + 1)
            intersection = w * h

            # Compute the ratio between intersection and union
            ratio = intersection / (areas[index] + areas[order[:-1]] - intersection)

            left = np.where(ratio < threshold)
            order = order[left]

        return picked_boxes, picked_score
