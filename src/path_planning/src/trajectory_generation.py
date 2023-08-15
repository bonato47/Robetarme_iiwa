#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Trajectory generation class to generate the different
needed interpolation between all the waypoints submitted.

Update: 230802
Maintainers: louis.munier@epfl.ch
"""

import os
from typing import List, Union

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splev, splprep
from scipy.spatial.transform import Rotation, Slerp

from ros_msgs_manager import numpy_to_pose


class TrajectoryGeneration():
    """Generate trajectory from list of waypoints.

    args:
        waypoints: List of the waypoints to be stored and interpolated.
    """

    def __init__(self, waypoints: np.array):
        self._nb_segments = []
        self._waypoints = waypoints

        self._idx_trajectory = -1
        self._wpositions = self._waypoints[:, 4:7]
        self._worientations = self._waypoints[:, :4]
        self._trajectory = np.empty((0, 7), float)

    def get_waypoints(self) -> np.array:
        """Getter function on the waypoints.

        return:
            _waypoints: Array of the waypoints to be stored and interpolated
        """
        return self._waypoints

    def get_wpositions(self) -> np.array:
        """Getter function on the waypoints positions.

        return:
            _wpositions: Array of the positions of waypoints to be stored and interpolated
        """
        return self._trajectory[:, 4:7]

    def get_worientations(self) -> np.array:
        """Getter function on the waypoints orientations.

        return:
            _worientations: Array of the orientations of waypoints to be stored and interpolated
        """
        return self._trajectory[:, :4]

    def get_trajectory(self) -> np.array:
        """Getter function on the interpolated trajectory.

        return:
            _trajectory: Array of the interpolated trajectory points
        """
        return self._trajectory

    def set_trajectory(self, trajectory: np.array):
        """Setter function on the interpolated trajectory.

        args:
            trajectory: Array of a pre-computed interpolated trajectory points
        """
        self._trajectory = trajectory[:, :7]
        self._nb_segments.append(0)

    def get_next_pose(self, ascending: bool = True) -> Pose:
        """Getter function to have the next Pose on the trajectory.

        return:
            next_pose: The next target position for the robot, if any
            ascending: Boolean to know if we want to go forward or backward in the trajectory
        """
        if ascending:
            self._idx_trajectory += 1
        else:
            self._idx_trajectory -= 1

        if 0 <= self._idx_trajectory < np.shape(self._trajectory)[0]:
            return numpy_to_pose(np.append(
                self.get_worientations()[self._idx_trajectory, :],
                self.get_wpositions()[self._idx_trajectory, :]
            ))

        return None

    @staticmethod
    def _compute_segment_lengths(points: np.array) -> np.array:
        """Compute the length of each segments in between two consecutive waypoints.

        args:
            points: array of the waypoints to use to compute length

        return:
            array of all the computed lengths
        """
        vectors_waypoints = points[1:, :] - points[0:-1, :]
        return np.linalg.norm(vectors_waypoints, axis=1)

    def verify(self):
        """Verify the generated trajectory to check if the different positions are reachable."""
        # TODO : Check if still needed and finish it or remove it
        pos_center_sphere = np.array([0, 0, 0.36])
        pos_center_torus = np.array([0, 0, 0.145])

        # Verify the position on the sphere part
        limit_z = -0.06
        rad_small_sphere = 0.42
        rad_big_sphere = 0.82

        rad_small_torus = 0.4
        rad_big_torus = 0.3637

        tmp_traj_sphere = np.zeros_like(self._trajectory)
        condition = self._trajectory[:, -1] >= limit_z
        row_indices = np.where(condition)[0]
        tmp_traj_sphere[row_indices] = self._trajectory[row_indices]


    def save(self, filename: str, delta_t: float = None):
        """Save generated trajectory to a csv file.

        args:
            filename: name of the file to be saved
            delta_t: value of time between each points if needed, default is None
        """
        to_save = self._trajectory
        header = "quat_x, quat_y, quat_z, quat_w, pos_x, pos_y, pos_z"

        if delta_t is not None:
            header += ", dt"
            to_save = np.hstack((
                self._trajectory,
                delta_t * np.ones((np.shape(self._trajectory)[0], 1))
            ))

        dir_path = os.path.dirname(os.path.realpath(__file__))
        np.savetxt(
            dir_path + "/../data/" + filename + ".csv",
            to_save,
            fmt='%.6e',
            delimiter=',',
            header=header
        )


    def display(self, points: np.array, nb_points: int = None, offset: int = 0, **kwargs):
        """Print the n lines of waypoints.

        args:
            points: The array of points to display
            nb_points (optional): Number of lines of waypoints to print, if None : print all
            offset (optional): Allow to begin where we want in the list, default : 0

        **kwargs:
            bool_print (optional): boolean to print the asked points
            bool_plot (optional): boolean to create a plot of the asked points
        """
        indices = None
        to_display = np.array([])

        if nb_points is None:
            nb_points = np.shape(points)[0]

        if nb_points <= 0:
            print(
                f"ERROR - Nb points : {nb_points} should be bigger or equal to zero or not "
                "specified."
            )
            return

        to_display = self._to_display(points, nb_points, offset)

        if "bool_waypoints" in kwargs and kwargs["bool_waypoints"]:
            indices = []
            sum_temp = 0

            len_segments = len(self._nb_segments)
            for i in range(len_segments + 1):
                if offset <= sum_temp <= offset + nb_points - 1:
                    indices.append(i)
                elif sum_temp > offset + nb_points - 1:
                    break

                if i != len_segments:
                    sum_temp += self._nb_segments[i]

        if "bool_print" in kwargs and kwargs["bool_print"]:
            print(to_display)

        if "bool_plot" in kwargs and kwargs["bool_plot"]:
            self._plot(to_display, indices)

    @staticmethod
    def _to_display(points: np.array, nb_points: int, offset: int = 0) -> np.array:
        """Print the n points from the category passed in input.

        args:
            points: Array of all the points where some have to be printed
            nb_points: Number of lines of points to print
            offset: Allow to begin where we want in the list, default : 0

        return:
            Part of the input points array to be displayed
        """
        lines = nb_points
        max_points = np.shape(points)[0]

        if offset >= max_points:
            print(
                f"ERROR - offset bigger or equal to size : {offset} >= {max_points}.")
            return None

        if nb_points + offset > max_points:
            lines = max_points - offset
            print(
                f"WARNING : nb_points + offset too big, only print the {lines} last lines.")
        else:
            print(f"Points from {offset + 1} to {offset + nb_points}")

        return points[offset: offset + lines, :]

    def _plot(self, points: np.array, waypoints_indices: List[int] = None):
        """Plot the array passed in input on a 3d graph.

        args:
            points: Array of the points to be ploted
            waypoints_indices: if needed, list of the waypoints to plot from that trajectory
        """
        fig = plt.figure()
        axes = fig.add_subplot(111, projection="3d")
        axes.plot(points[:, 4], points[:, 5], points[:, 6], 'b.', markersize=3)

        if waypoints_indices is not None:
            axes.plot(
                self._waypoints[waypoints_indices[:1], 4],
                self._waypoints[waypoints_indices[:1], 5],
                self._waypoints[waypoints_indices[:1], 6],
                'mx'
            )
            axes.plot(
                self._waypoints[waypoints_indices[1:], 4],
                self._waypoints[waypoints_indices[1:], 5],
                self._waypoints[waypoints_indices[1:], 6],
                'rx'
            )

            axes.set_xlabel("x coordinates")
            axes.set_ylabel("y coordinates")
            axes.set_zlabel("z coordinates")

        plt.show()


class StraightLine(TrajectoryGeneration):
    """Generate straight line interpolated trajectory from list of waypoints.

    args:
        waypoints: List of the waypoints to be stored and interpolated.
    """

    def _interpolate_positions(self, v_mean: float, timestep: int):
        """Generate straight lines between kaypoints.

        args:
            v_mean: Average velocity to be used in linear interpolation function
            timestep: Delta of time between each point, related to frequency used
                to communicate with the robot
        """
        interp_positions = np.empty((0, 3), float)
        segment_lengths = self._compute_segment_lengths(self._wpositions)

        for i, pos in enumerate(self._wpositions):
            if i < np.shape(segment_lengths)[0]:
                self._nb_segments.append(int(
                    segment_lengths[i] / (v_mean * timestep)) + 1)

                interp_positions = np.append(
                    interp_positions,
                    np.linspace(
                        self._wpositions[i],
                        self._wpositions[i+1],
                        self._nb_segments[-1] + 1)[:-1, :],
                    axis=0)
            else:
                interp_positions = np.append(interp_positions, [pos], axis=0)

        return np.round(interp_positions, rospy.get_param("/precision"))

    def _interpolate_orientation(self, v_mean: float, timestep: int):
        """Linearly interpolate between kaypoint's orientations using SLERP method.

        args:
            v_mean: Average velocity to be used in linear interpolation function
            timestep: Delta of time between each point, related to frequency used
                to communicate with the robot
        """
        sum_t = 0
        key_time = [0]
        for t in self._nb_segments:
            sum_t += t
            key_time.append(sum_t * v_mean * timestep)

        key_rot = Rotation.from_quat(self._worientations)

        time = []
        slerp = Slerp(key_time, key_rot)
        for i, _ in enumerate(key_time[:-1]):
            time += np.arange(
                key_time[i], key_time[i + 1], step=v_mean * timestep
            ).tolist()

        time = np.round(time, rospy.get_param("/precision"))

        return slerp(time).as_quat()

    def generate(self, v_mean: float, timestep: int):
        """Generate straight lines from the interpolated trajectory.

        args:
            v_mean: Average velocity to be used in linear interpolation function
            timestep: Delta of time between each point, related to frequency used
                to communicate with the robot
        """
        interp_positions = self._interpolate_positions(v_mean, timestep)
        interp_orientations = self._interpolate_orientation(v_mean, timestep)

        print(np.shape(interp_orientations))
        print(np.shape(interp_positions))

        self._trajectory = np.hstack((interp_orientations, interp_positions[:-1, :]))

class Circle(TrajectoryGeneration):
    """Generate circular interpolated trajectory from list of waypoints.

    args:
        waypoints: List of the waypoints to be stored and interpolated.
    """

    def __init__(self, radius: float, clockwise: bool = True):
        """Initialize circle interpolated trajectory.

        args:
            radius: radius value of the circle used to interpolate trajectory
            clockwise (optional): boolean to precise if circles should be traveled clockwise or not
        """
        super().__init__()

        if radius == 0:
            raise ValueError("Radius of zero.")

        self._radius = radius
        self._clockwise = clockwise

    def _verify_waypoints(self):
        """Analyze all the waypoints to be sure a circle can be fitted on them."""
        segment_lengths = self._compute_segment_lengths(self._wpositions)

        if sum(segment_lengths > 2 * self._radius):
            raise ValueError(
                "Separation of some points is bigger than the diameter of the circle.")

        if sum(np.round(segment_lengths, 4) == 0):
            raise ValueError(
                'Coincident points gives infinite number of circles.')

    def generate(self):
        """Generate arcs from the interpolated trajectory."""
        print(f"Generate Arcs.")
        segment_lengths = self._compute_segment_lengths(self._wpositions)
        np_radius = np.resize(
            np.array([self._radius]), np.shape(segment_lengths))

        d_mirror_line = np.sqrt(np_radius**2 - (0.5 * segment_lengths)**2)

        self._trajectory = np.round(
            self._trajectory, rospy.get_param("/precision"))


class BSpline(TrajectoryGeneration):
    """Generate bspline interpolated trajectory from list of waypoints.

    args:
        waypoints: List of the waypoints to be stored and interpolated.
    """

    def __init__(self, waypoints: np.array):
        """Initialize bspline interpolated trajectory."""
        super().__init__(waypoints)
        self._first_estimation_nb = 200
        self._smoothness = 0.0

    def generate(self, v_mean: float, timestep: int):
        """Generate bspline from the interpolated trajectory.

        args:
            v_mean: Average velocity to be used in linear interpolation function
            timestep: Delta of time between each point, related to frequency used
                to communicate with the robot
        """
        tck, u_orig = splprep(self._wpositions.T, u=None, s=self._smoothness)
        u_new = np.linspace(u_orig[:-1], u_orig[1:], self._first_estimation_nb)

        for param in range(np.shape(u_new)[-1]):
            x_new, y_new, z_new = splev(u_new[:, param], tck, der=0)
            coord = np.vstack((x_new, y_new, z_new))
            length = np.sum(self._compute_segment_lengths(coord))

            nb_segments = int(length / (v_mean * timestep)) + 1
            u_temp = np.linspace(u_orig[param], u_orig[param + 1], nb_segments)

            self._trajectory = np.append(
                self._trajectory[:-1, :],
                np.array(splev(u_temp, tck, der=0)).T,
                axis=0
            )
            self._nb_segments.append(nb_segments - 1)

        self._trajectory = np.round(
            self._trajectory, rospy.get_param("/precision"))
