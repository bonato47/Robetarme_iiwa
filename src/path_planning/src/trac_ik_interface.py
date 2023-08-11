#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Inverse Kinematic class object to generate a new one for each robot.

Update: 230623
Maintainers: louis.munier@epfl.ch
"""
import rospy

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trac_ik_python.trac_ik import IK


class TracIKInterface():
    """Compute trac_ik Inverse Kinematic interface."""

    def __init__(self):
        """ Class initializer"""
        self._base_link = rospy.get_param("/base_link")
        self._end_effector = rospy.get_param("/end_effector")
        self._dict_ros_comm = {}
        self._ik_solver = IK(
            self._base_link,
            self._end_effector,
            solve_type="Distance",
            urdf_string=rospy.get_param('/robot_description'),
            timeout=0.01
        )

        self._curr_joint_state = JointState()
        self._cmd_joint_positions = None

        self._setup_ros_communication()

    def _setup_ros_communication(self):
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

    def compute_ik(self, target: Pose):
        """Compute inverse kinematic for the current pose.

        args:
            target: ROS Pose message to set as the new target for the robot
        """
        old_cmd_joint_positions = self._cmd_joint_positions

        try:
            self._cmd_joint_positions = self._ik_solver.get_ik(
                self._curr_joint_state.position,
                target.position.x,
                target.position.y,
                target.position.z,
                target.orientation.x,
                target.orientation.y,
                target.orientation.z,
                target.orientation.w,
                bx=1e-6, by=1e-6, bz=1e-6,
                brx=1e-6, bry=1e-6, brz=1e-6
            )  # X, Y, Z, QX, QY, QZ, QW

            if self._cmd_joint_positions is None:
                self._cmd_joint_positions = old_cmd_joint_positions
                print("Reached target position OR Infeasible position")
        except Exception:
            pass

    def send_position(self):
        """Send the new position to the robot."""
        robot_pos_msg = Float64MultiArray()

        if self._cmd_joint_positions is not None:
            robot_pos_msg.data = self._cmd_joint_positions
            self._dict_ros_comm["pos_controller"].publish(robot_pos_msg)
