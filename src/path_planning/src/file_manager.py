#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Regroup all the needed function to manage files.

Update: 230511
Maintainers: louis.munier@epfl.ch
"""

from datetime import datetime

import rospy


def refactor_savepath() -> str:
    """Adapt the path to save the file and have a common naming convention.

    return:
        string of the absolut pathname to store the csv file
    """
    record_name = rospy.get_param("/record_name")
    save_path = rospy.get_param("/save_path")
    file_save_path = "/".join(__file__.split("/")[:-2]) + "/data"

    if save_path and save_path[0] != "/":
        file_save_path += "/"

    file_save_path += save_path

    if save_path and save_path[-1] != "/":
        file_save_path += "/"

    return file_save_path + get_timestamp() + record_name + ".csv"


def get_timestamp() -> str:
    """Getter on the timestamp.

    return
        string of the current timestamp to save the file without destroying old one
    """
    return datetime.now().strftime("%y%m%d-%H%M%S_")


def get_ros_time() -> float:
    """Getter on the ros time.

    return
        string of the current ros time to be stored
    """
    ros_time = rospy.get_rostime()
    return ros_time.secs + round(ros_time.nsecs * 1e-9, 3)
