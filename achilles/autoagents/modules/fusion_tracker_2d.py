#!/usr/bin/env python

"""
Base class for object trackers
"""
from __future__ import print_function

import logging
import numpy as np
from numpy import dot
from scipy.linalg import inv, block_diag

from enum import Enum

from achilles.autoagents.modules.obstacle_detector import Obstacle, ObstacleType
from achilles.autoagents.modules.obstacle_tracker_2d import Track, Tracker2D


class FusionTracker2D(Tracker2D):

    """
    Fusion of obstacle tracks from different sources, e.g., camera, HDMap
    """

    def __init__(self, hit_count=4, reserve_age=40):
        self._tracks = []
        self._next_oid = 0

        # [Zhu et al., ECCV 2018]: hit_count = 0.2 * fps, reserve_age = 2 * fps
        self._hit_count = 4
        self._reserve_age = 40

        # [MATLAB]: hit_count = 1, reserve_age = 20
        # self._hit_count = 1
        # self._reserve_age = 20

        # [Wang et al., ECCV 2020]
        # self._hit_count = 2
        # self._reserve_age = 30

        self._age_threshold = 8
