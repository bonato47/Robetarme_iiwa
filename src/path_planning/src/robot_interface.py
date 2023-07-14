#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Inverse Kinematic class object to generate a new one for each robot.

Update: 230614
Maintainers: louis.munier@epfl.ch
"""
import csv
import os
from math import dist, fabs, cos

import rospy
import tf2_ros as tf2
from ros_msgs_manager import pose_to_list
from file_manager import get_ros_time, refactor_savepath
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trac_ik_python.trac_ik import IK


class RobotInterface():
    """Compute Inverse Kinematic interface.

    args:
        base_link: first robot joint name
        end_effector: last robot joint name
    """

    def __init__(self, base_link: str, end_effector: str):
        self._csv_header = False
        self._record_path = refactor_savepath()
        self._set_recording(rospy.get_param("/recording"))

        self._dofs = rospy.get_param("/degrees_of_freedom")
        self._base_link = base_link
        self._end_effector = end_effector
        self._tf_buffer = None
        self._tf_listener = None
        self._dict_ros_comm = {}

        self._curr_joint_state = JointState()
        self._cmd_joint_positions = None
        self._target = Pose()

        self._setup_ros()

    def _setup_ros(self):
        """Setup all the different ROS requirements."""
        robot_joint_state_topic = rospy.get_param("/robot_joint_state_topic")
        rospy.Subscriber(
            robot_joint_state_topic, JointState, self._cbk_curr_joint_state)

        pos_controller_topic = rospy.get_param("/position_controller_topic")
        self._dict_ros_comm["pos_controller"] = rospy.Publisher(
            pos_controller_topic, Float64MultiArray, queue_size=1)

    def _cbk_curr_joint_state(self, data: JointState):
        """Callback to retrieve the current joint state and record data if eneabled.

        args:
            data: Data inside current joint message
        """
        self._curr_joint_state = data

        if rospy.get_param("/recording"):
            self._record_data()

    def get_command_joint_pos(self):
        """Getter on the command for each robot joints."""
        return self._cmd_joint_positions

    def get_current_ee_status(self) -> Pose:
        """Reads and stores end effector status.

        args:
            args: Contains the current end effector position and orientation of iiwa
        """
        curr_ee_state = Pose()

        try:
            ros_tf = self._tf_buffer.lookup_transform(
                self._base_link,
                self._end_effector,
                rospy.Time(0)
            )

            curr_ee_state.position = ros_tf.transform.translation
            curr_ee_state.orientation = ros_tf.transform.rotation

            return curr_ee_state
        except(tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
            return None

    def _set_recording(self, recording: bool):
        """Set the ros parameter 'recording' to the needed value.

        args:
            recording: boolean to enable or disable the recording of the data
        """
        dir_path = "/".join(self._record_path.split("/")[:-1])
        rospy.set_param("/recording", recording)

        if not os.path.exists(dir_path):
            os.makedirs(dir_path)

    def set_target(self, target: Pose):
        """Setter function to the set the new target of the robot.

        args:
            target: ROS Pose message to set as the new target for the robot
        """
        self._target = target

    def wait_on_robot(self):
        """Wait for the iiwa robot to be started."""
        self._tf_buffer = tf2.Buffer()
        self._tf_listener = tf2.TransformListener(self._tf_buffer)

        while not rospy.is_shutdown():
            try:
                self._tf_buffer.lookup_transform(
                    self._base_link,
                    self._end_effector,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
            except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
                print("waiting for tf2 iiwa")
            else:
                return True

    def has_reached_target(self) -> bool:
        """Return True if robot reached target position."""
        target = self._target
        if target is None:
            return None

        curr_ee_state = self.get_current_ee_status()

        distance = dist(
            (target.position.x, target.position.y, target.position.z),
            (curr_ee_state.position.x, curr_ee_state.position.y, curr_ee_state.position.z)
        )

        # phi = angle between orientations
        cos_phi_half = fabs(
            target.orientation.x * curr_ee_state.orientation.x +
            target.orientation.y * curr_ee_state.orientation.y +
            target.orientation.z * curr_ee_state.orientation.z +
            target.orientation.w * curr_ee_state.orientation.w
        )

        eps_threshold = float(rospy.get_param("/epsilon"))
        condition_distance = distance <= eps_threshold
        condition_orientation = cos_phi_half >= cos(eps_threshold / 2.0)

        print(distance, condition_distance, condition_orientation)

        return condition_distance and condition_orientation

    def _record_header(self):
        """Store the csv header."""
        with open(self._record_path, "a") as file:
            header = [
                "timestamp",
                "position x", "position y", "position z",
                "orientation x", "orientation y", "orientation z", "orientation w"
            ]

            header += [
                f"pose cmd iiwa_joint_{i}" for i in range(1, self._dofs + 1)]
            header += [
                f"pose curr iiwa_joint_{i}" for i in range(1, self._dofs + 1)]
            header += [
                f"velocity iiwa_joint_{i}" for i in range(1, self._dofs + 1)]
            header += [
                f"effort iiwa_joint_{i}" for i in range(1, self._dofs + 1)]

            csv.writer(file, delimiter=",").writerow(header)

    def _record_data(self):
        """Record necessary data."""
        if not self._csv_header:
            self._record_header()
            self._csv_header = True

        curr_ee_status = self.get_current_ee_status()
        if (curr_ee_status and self._cmd_joint_positions) is not None:
            data_to_write = [get_ros_time()]
            data_to_write += pose_to_list(curr_ee_status)
            data_to_write += list(self._cmd_joint_positions)
            data_to_write += list(self._curr_joint_state.position)
            data_to_write += list(self._curr_joint_state.velocity)
            data_to_write += list(self._curr_joint_state.effort)

            with open(self._record_path, "a") as file:
                csv.writer(file, delimiter=",").writerow(data_to_write)
