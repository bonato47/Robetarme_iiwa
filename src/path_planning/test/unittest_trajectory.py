#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Run tests for teh trajectory generation class.

Update: 230501
Maintainers: louis.munier@epfl.ch
"""

import numpy as np
import unittest


class TestTrajectoryGeneration(unittest.TestCase):
    """Test cases for the trajectory generation class."""

    def setUp(self):
        """Implement variables that are needed to run all these tests."""
        num = 1000
        self.waypoints = np.zeros((num, 7))
        self.start_point = np.array(
            [0.6, 0.4, 0.7, 0, 0.7068252, 0, 0.7073883]
        )
        self.end_point = np.array(
            [0.6, -0.2, 0.5, 0, 0.7068252, 0, 0.7073883]
        )

        for i in range(7):
            self.waypoints[:, i] = np.linspace(
                start=self.start_point[i], stop=self.end_point[i], num=num)

    def test_print(self):
        print(self.waypoints)


if __name__ == "__main__":
    unittest.main()
