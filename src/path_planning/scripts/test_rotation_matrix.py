#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Test and find the rotation matrix between two referentials.

Update: 230620
Maintainers: louis.munier@epfl.ch
"""
import numpy as np

from tf.transformations import quaternion_multiply
from scipy.spatial.transform import Rotation as R

x_vec_right = np.array([1, 0, 0])
y_vec_right = np.array([0, 1, 0])
z_vec_right = np.array([0, 0, 1])

transform_mat = np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])

# Compute transform
x_vec_left = np.dot(transform_mat, x_vec_right)
y_vec_left = np.dot(transform_mat, y_vec_right)
z_vec_left = np.dot(transform_mat, z_vec_right)

transform_mat_inv = np.linalg.inv(transform_mat)
print(transform_mat_inv)

# Compute rotation quaternion to cancel the initial one
init_rot = [0.5, 0.5, 0.5, 0.5]

r_base_change = R.from_quat(init_rot)
q_inv_base_change = r_base_change.inv().as_quat()
print(q_inv_base_change)
