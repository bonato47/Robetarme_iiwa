#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Inverse Kinematic class object to generate a new one for each robot.

Update: 230623
Maintainers: louis.munier@epfl.ch
"""
import rospy
import numpy as np

from tf.transformations import quaternion_multiply
from geometry_msgs.msg import Pose, Vector3, Quaternion
from relaxed_ik_ros1.msg import EEPoseGoals


class RelaxedIKInterface():
    """Compute relaxed_ik Inverse Kinematic interface."""

    def __init__(self):
        self._dict_ros_comm = {}
        self._seq = 1
        self._cmd_joint_positions = Pose()
        self._setup_ros()

    def _setup_ros(self):
        """Setup all the different ROS requirements."""
        self._dict_ros_comm["pos_controller"] = rospy.Publisher(
            "/relaxed_ik/ee_pose_goals", EEPoseGoals, queue_size=5)

    def compute_ik(self, target: Pose):
        """Compute inverse kinematic for the current pose.

        args:
            target: ROS Pose message to set as the new target for the robot
        """
        tmp_target = Pose(
            Vector3(0.0, 0.0, 0.0), Quaternion(1, 0, 0, 0)
        )

        # Cancel initial position
        init_pos = [0.783, 0, 0.685]
        position_right = -1 * np.array(init_pos)
        position_right += np.array(
            [target.position.x, target.position.y, target.position.z]
        )

        tmp_target.position = Vector3(
            position_right[0], position_right[1], position_right[2]
        )

        # Cancel initial orientation
        q_rot = Quaternion(0.5, 0.5, 0.5, -0.5)
        final_quaternion = quaternion_multiply(
            [target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w],
            [q_rot.x, q_rot.y, q_rot.z, q_rot.w]
        )
        tmp_target.orientation.x = final_quaternion[0]
        tmp_target.orientation.y = final_quaternion[1]
        tmp_target.orientation.z = final_quaternion[2]
        tmp_target.orientation.w = final_quaternion[3]

        self._cmd_joint_positions = tmp_target

    def send_position(self):
        """Send the new position to the robot."""
        if self._cmd_joint_positions is not None:
            target = EEPoseGoals()
            target.ee_poses.append(self._cmd_joint_positions)

            target.header.seq = self._seq
            self._seq += 1

            self._dict_ros_comm["pos_controller"].publish(target)
