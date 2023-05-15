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

from achilles.autoagents.modules.obstacle_detector import Obstacle
from achilles.autoagents.modules.obstacle_detector import ObstacleType


class FirstOrderRCLowPassFilter(object):
    def __init__(self, x=[0., 0.], alpha=0.5):
        self.alpha_ = alpha
        self.state_ = np.array(x)

    def set_alpha(self, alpha):
        self.alpha_ = alpha

    def measure(self, z):
        z = np.array(z)
        self.state_ = z + self.alpha_ * (self.state_ - z)

    def measure_noinput(self):
        z = self.state_
        self.measure(z)

    def get_state(self):
        return self.state_.astype(int).flatten().tolist()


class KalmanFilter2D(object):

    def __init__(self, x=[0., 0., 0., 0.], wh=[0., 0.]):
        """
        Initialize parameters for Kalman Filter
        The state is the (x, y) coordinates of the center of detection box
        """
        # State: [center_c, center_c_dot, center_r, center_r_dot]
        self.x = np.array(x).reshape((4,1))
        self.dt = 1.

        # State transition matrix F
        self.F = np.array([[1., 0., 0., 0.],
                           [0., 1., 0., 0.],
                           [0., 0., 1., 0.],
                           [0., 0., 0., 1.]])
        self.F[0, 1] = self.dt
        self.F[2, 3] = self.dt

        # Measurement matrix H, assuming we can only measure the coordinates
        self.H = np.array([[1., 0., 0., 0.],
                           [0., 0., 1., 0.]])

        # State covariance matrix P
        self.L = 10.
        self.P = np.diag(self.L * np.ones(4))

        # Process covariance
        self.Q_comp_mat = np.array([[self.dt**4/4., self.dt**3/2.],
                                    [self.dt**3/2., self.dt**2]])
        self.Q = block_diag(self.Q_comp_mat, self.Q_comp_mat)

        # Measurement covariance
        self.R_scaler = 1.
        self.R_diag_array = self.R_scaler * np.array([self.L, self.L])
        self.R = np.diag(self.R_diag_array)

    def set_state(self, x):
        self.x = np.array(x).reshape((4,1))

    def set_R(self, R_scaler):
        R_diag_array = R_scaler * np.array([self.L, self.L])
        self.R = np.diag(R_diag_array)

    def get_state(self):
        # return self.x.astype(int)
        return self.x.astype(int).flatten().tolist()

    def predict(self):
        """
        Implment only the predict stage. This is used for unmatched detections and
        unmatched tracks
        """
        self.x = dot(self.F, self.x)
        self.P = dot(self.F, self.P).dot(self.F.T) + self.Q

    def update(self, z):
        """
        Implement the Kalman Filter, including the predict and the update stages,
        with the measurement z
        """
        z = np.array(z).reshape((2,1))
        S = dot(self.H, self.P).dot(self.H.T) + self.R
        K = dot(self.P, self.H.T).dot(inv(S)) # Kalman gain
        y = z - dot(self.H, self.x) # residual

        self.x += dot(K, y)
        self.P = self.P - dot(K, self.H).dot(self.P)


class Track(object):
    def __init__(self, measure, oid):
        self.oid = oid
        self._type = measure.get_type()
        self.age = 1
        self.total_visible_count = 1
        self.consecutive_invisible_count = 0

        obs_bbox = measure.get_bbox()
        centroid = [(obs_bbox[1] + obs_bbox[3])/2., 0., (obs_bbox[0] + obs_bbox[2])/2., 0.]
        self.centroid_filter = KalmanFilter2D(centroid)

        wh = [obs_bbox[3] - obs_bbox[1] + 1, obs_bbox[2] - obs_bbox[0] + 1]
        self.wh_filter = FirstOrderRCLowPassFilter(wh)

        self._publishable = False

    def predict(self):
        self.centroid_filter.predict()
        self.wh_filter.measure_noinput()
        self.age += 1

    def update(self, measure):
        obs_bbox = measure.get_bbox()
        centroid = [(obs_bbox[1] + obs_bbox[3])/2., (obs_bbox[0] + obs_bbox[2])/2.]
        self.centroid_filter.update(centroid)
        wh = [obs_bbox[3] - obs_bbox[1] + 1, obs_bbox[2] - obs_bbox[0] + 1]
        self.wh_filter.measure(wh)

        self._type = measure.get_type()
        self.total_visible_count += 1
        self.consecutive_invisible_count = 0

    def update_noinput(self):
        self.consecutive_invisible_count += 1

    def mark_publishable(self):
        self._publishable = True

    def is_publishable(self):
        return self._publishable

    def is_stop_sign(self):
        return self._type == ObstacleType.STOP_SIGN

    def get_type(self):
        return self._type

    def get_centroid(self):
        state = self.centroid_filter.get_state()
        return [state[2], state[0]]  # row, column

    def get_width_height(self):
        [width, height] = self.wh_filter.get_state()
        return width, height

    def get_bbox(self):
        center = self.get_centroid()
        print(center)
        w, h = self.get_width_height()
        # bbox convention: [top_left.h, top_left.w, bot_right.h, bot_right.w]
        bbox = [center[0] - (h//2), center[1] - (w//2), center[0] + (h//2), center[1] + (w//2)]
        return bbox


class Tracker2D(object):

    """
    2D obstacle tracker, the measurement can be obstacles or obstacle tracks
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

    def run_step(self, input_data, timestamp, measures):

        # 1. predict each track
        for trk in self._tracks:
            trk.predict()

        # 2. matching tracks and measurements
        matched_tracks, matched_measures, unmatched_tracks, unmatched_measures = \
                self.matching(measures)

        # 3. update matched tracks and obstacles
        for i, trk in enumerate(matched_tracks):
            trk.update(matched_measures[i])
            if trk.total_visible_count >= self._hit_count:
                trk_type = "STOP SIGN" if trk.is_stop_sign() else "OBSTACLE"
                logging.info(f"##### mark track: {trk.oid} publishable, type {trk_type}, age {trk.age}, visible_count {trk.total_visible_count}, invisible_count {trk.consecutive_invisible_count}")
                print(f"##### mark track: {trk.oid} publishable, type {trk_type}, age {trk.age}, visible_count {trk.total_visible_count}, invisible_count {trk.consecutive_invisible_count}")
                trk.mark_publishable()

        # 3. update unmatched tracks
        for i, trk in enumerate(unmatched_tracks):
            trk.update_noinput()

        # 4. delete lost tracks
        del_tracks = []
        for i, trk in enumerate(unmatched_tracks):
            visibility = float(trk.total_visible_count) / float(trk.age)
            if (trk.age < self._age_threshold and visibility < 0.6) or (trk.consecutive_invisible_count >= self._reserve_age):
                trk_type = "STOP SIGN" if trk.is_stop_sign() else "OBSTACLE"
                logging.info(f"##### delete track: {trk.oid}, type {trk_type}, age {trk.age}, visible_count {trk.total_visible_count}, invisible_count {trk.consecutive_invisible_count}")
                print(f"##### delete track: {trk.oid}, type {trk_type}, age {trk.age}, visible_count {trk.total_visible_count}, invisible_count {trk.consecutive_invisible_count}")
                del_tracks.append(trk)

        # 5. create tracks for unmatched obstacles
        new_tracks = []
        for obs in unmatched_measures:
            trk = Track(obs, self._next_oid)
            new_tracks.append(trk)
            self._next_oid += 1

        # 6. clean up
        for trk in del_tracks:
            for i in range(len(self._tracks)):
                if trk == self._tracks[i]:
                    del self._tracks[i]
                    break

        self._tracks += new_tracks
        pub_tracks = []
        for trk in self._tracks:
            if trk.is_publishable():
                pub_tracks.append(trk)

        return pub_tracks

    def matching(self, measures):
        # TODO: should use hunguarian matching
        # Now is a naive type based matcher
        matched_tracks = []
        matched_measures = []
        unmatched_tracks = []
        unmatched_measures = []

        meas_match_flags = [False for _ in range(len(measures))]
        for trk in self._tracks:
            trk_type = trk.get_type()
            for i, meas in enumerate(measures):
                if not meas_match_flags[i] and trk_type == meas.get_type():
                    matched_tracks.append(trk)
                    matched_measures.append(meas)
                    meas_match_flags[i] = True

        for trk in self._tracks:
            if not trk in matched_tracks:
                unmatched_tracks.append(trk)

        for meas in measures:
            if not meas in matched_measures:
                unmatched_measures.append(meas)

        return matched_tracks, matched_measures, unmatched_tracks, unmatched_measures
