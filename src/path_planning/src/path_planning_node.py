#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Main module for the path planning implementation.

Update: 230623
Maintainers: louis.munier@epfl.ch
"""
import os
import rospy
import numpy as np

from typing import Union
from relaxed_ik_interface import RelaxedIKInterface
from trac_ik_interface import TracIKInterface
from trajectory_generation import StraightLine, TrajectoryGeneration


def main():
    """Retrieve the different waypoints, setup ROS and call the needed IK"""
    ros_rate = setup_ros("path_planning")

    if rospy.get_param("/ik") == "trac_ik":
        ik_interface = TracIKInterface()
    elif rospy.get_param("/ik") == "relaxed_ik":
        ik_interface = RelaxedIKInterface()
    else:
        raise Exception("No inverse kinematic interface set !")

    # Retrieve waypoints
    dir_path = os.path.dirname(os.path.realpath(__file__))
    waypoints_filename = rospy.get_param("/waypoints_filename")
    waypoints = np.loadtxt(
        dir_path + "/../data/" + waypoints_filename + ".csv",
        delimiter=',',
        skiprows=1
    )

    # Generate trajectory or read it
    trajectory = StraightLine(waypoints)
    if rospy.get_param("/read_trajectory"):
        trajectory.set_trajectory(waypoints)
    else:
        trajectory.generate(0.05, 0.02)

    if rospy.get_param("/save_trajectory"):
        trajectory.save(rospy.get_param("/saved_trajectory_filename"), 0.02)

    trajectory.display(
        trajectory.get_trajectory(),
        bool_waypoints=True,
        bool_plot=True
    )

    # Start ROS loop
    print("Test started")
    ros_loop(
        ros_rate,
        ik_interface,
        trajectory,
        stop_condition="trajectory_finished"
    )
    print("Test finished")


def setup_ros(str_node: str) -> rospy.Rate:
    """Setup all the different ROS requirements.

    args:
        str_node: name of the node to initialize

    return:
        ros_rate: ROS communication rate
    """
    rospy.init_node(str_node, anonymous=True)

    freq = rospy.get_param("/frequency")
    ros_rate = rospy.Rate(freq)
    rospy.sleep(2)

    return ros_rate


def ros_loop(ros_rate: rospy.Rate, obj_ik_interface: Union[TracIKInterface, RelaxedIKInterface],
             obj_trajectory: TrajectoryGeneration, stop_condition: str = None):
    """Main function to communicate the positions to ROS when it runs.

    args:
        ros_rate: ROS communication rate
        obj_ik_interface: Inverse kinematic interface
        obj_trajectory: A trajectory generation object to manage waypoints and interpolate trajectories
        stop_condition: (optional) give a condition to stop the ros loop
    """
    while not rospy.is_shutdown():
        target_pose = obj_trajectory.get_next_pose()
        obj_ik_interface.compute_ik(target_pose)
        obj_ik_interface.send_position()

        if stop_condition == "trajectory_finished" and target_pose is None:
            break

        ros_rate.sleep()


if __name__ == "__main__":
    main()
