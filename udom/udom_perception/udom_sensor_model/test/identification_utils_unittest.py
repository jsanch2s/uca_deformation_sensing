#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Unittest for the sensor_model/identification_utils.py module.

"""

import numpy as np
import unittest
import rosunit
import geometry_msgs.msg
import udom_sensor_model.identification_utils as id_utils

PKG = 'udom_sensor_model'


class TestIdentificationUtils(unittest.TestCase):
    """
    Tests the sensor_model/identification_utils.py module.

    """
    def test_compute_v_matrix(self):
        """
        Tests that the compute_v_matrix function returns a 6x10 matrix when an input with
        acceleration (linear and angular), angular velocity and gravity are given.

        """
        acceleration = geometry_msgs.msg.Accel(
            linear=geometry_msgs.msg.Vector3(1, 2, 3),
            angular=geometry_msgs.msg.Vector3(4, 5, 6))
        angular_velocity = geometry_msgs.msg.Vector3(0, -1, -2)
        gravity = geometry_msgs.msg.Vector3(9, 0, 0)

        v_matrix = id_utils.compute_v_matrix(acceleration, angular_velocity, gravity)

        self.assertIsInstance(v_matrix, np.ndarray)
        self.assertEqual(v_matrix.shape, (6, 10))

    def test_compute_v_g_init(self):
        """
        Tests that the compute_v_g_init function returns a 6x10 matrix when an input with
        a gravity vector given.

        """
        gravity = geometry_msgs.msg.Vector3(9, 0, 0)
        g_matrix = id_utils.compute_v_g_init(gravity)

        self.assertIsInstance(g_matrix, np.ndarray)
        self.assertEqual(g_matrix.shape, (6, 10))

    def test_rotate_gravity(self):
        """
        Tests that the rotate_gravity function returns a rotated gravity vector.

        """
        gravity = geometry_msgs.msg.Vector3(9, 0, 0)
        rotated_gravity = geometry_msgs.msg.Vector3(0, 0, 9)
        no_rotation = geometry_msgs.msg.Quaternion
        no_rotation.w = 1.0
        no_rotation.x = 0.0
        no_rotation.y = 0.0
        no_rotation.z = 0.0

        g_rotated = id_utils.rotate_gravity(gravity, no_rotation)

        # No rotation
        self.assertAlmostEqual(g_rotated.x, gravity.x)
        self.assertAlmostEqual(g_rotated.y, gravity.y)
        self.assertAlmostEqual(g_rotated.z, gravity.z)

        rotation = geometry_msgs.msg.Quaternion
        rotation.w = 0.707
        rotation.x = 0.0
        rotation.y = 0.707
        rotation.z = 0.0

        g_rotated = id_utils.rotate_gravity(gravity, rotation)

        # Rotation around Y
        self.assertAlmostEqual(g_rotated.x, rotated_gravity.x)
        self.assertAlmostEqual(g_rotated.y, rotated_gravity.y)
        self.assertAlmostEqual(g_rotated.z, rotated_gravity.z)

    def test_transform_twist(self):
        """
        Tests that the transform_twist function returns a transformed twist.

        """
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 1.0
        twist.linear.y = 2.0
        twist.linear.z = 3.0
        twist.angular.x = 4.0
        twist.angular.y = 5.0
        twist.angular.z = 6.0

        transform = np.identity(4)

        transformed_twist = id_utils.transform_twist(twist, transform)

        # No rotation
        self.assertAlmostEqual(transformed_twist, twist)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_tactile_utils', TestIdentificationUtils)
