#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Useful library to manage ros messages

Update: 230623
Maintainers: louis.munier@epfl.ch
"""

import numpy as np

from typing import List
from geometry_msgs.msg import Point, Pose, Quaternion


def pose_to_numpy(ros_pose: Pose) -> np.array:
    """Translate ros message Pose to numpy array.

    args:
        ros_pose: the Pose ros message to be translated to numpy array

    return:
        numpy array
    """
    return np.array([
        ros_pose.orientation.x,
        ros_pose.orientation.y,
        ros_pose.orientation.z,
        ros_pose.orientation.w,
        ros_pose.position.x,
        ros_pose.position.y,
        ros_pose.position.z
    ])


def pose_to_list(ros_pose: Pose) -> List[float]:
    """Translate ros message Pose to python list.

    args:
        ros_pose: the Pose ros message to be translated to python list

    return:
        python list
    """
    return [
        ros_pose.orientation.x,
        ros_pose.orientation.y,
        ros_pose.orientation.z,
        ros_pose.orientation.w,
        ros_pose.position.x,
        ros_pose.position.y,
        ros_pose.position.z,
    ]


def numpy_to_pose(np_pose: np.array) -> Pose:
    """Translate numpy array to ros message pPse.

    args:
        np_pose: the Pose numpy array to be translated to ros message

    return:
        ros Pose message
    """
    return Pose(
        Point(np_pose[4], np_pose[5], np_pose[6]),
        Quaternion(np_pose[0], np_pose[1], np_pose[2], np_pose[3])
    )
