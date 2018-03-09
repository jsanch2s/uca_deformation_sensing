#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Unittest for the transformation_utils.py module.

"""

import numpy as np
import unittest
import rosunit
import geometry_msgs.msg
import udom_geometric_transformation.transformation_utils as transformation_utils

PKG = 'udom_geometric_transformation'


class TestTransformationUtils(unittest.TestCase):
    """
    Tests the transformation_utils.py module.

    """
    def test_transform_wrench_translation_basic(self):
        """
        Tests that the transform_wrench can transform a wrench correctly for a translation
        with the force applied on the reference frame.

        """
        wrench_in = geometry_msgs.msg.Wrench()
        wrench_in.force.x = 4.0
        transform_matrix = np.array([
            [1, 0, 0, 1],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)

        desired = geometry_msgs.msg.Wrench()
        desired.force.x = 4.0

        result = transformation_utils.transform_wrench(wrench_in, transform_matrix)
        self.assertEqual(result, desired)

        # Try a different transformation matrix.
        transform_matrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        desired.torque.y = 4.0

        result = transformation_utils.transform_wrench(wrench_in, transform_matrix)
        self.assertEqual(result, desired)

    def test_transform_wrench_rotation_basic(self):
        """
        Tests that the transform_wrench can transform a wrench correctly for a rotation
        with the force applied on the reference frame.

        """
        wrench_in = geometry_msgs.msg.Wrench()
        wrench_in.force.x = 1.0
        wrench_in.torque.x = 1.0

        # Test a roll rotation.
        angle = np.pi / 2.0
        transform_matrix = np.array([
            [1,             0,              0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle),  np.cos(angle), 0],
            [0,             0,              0, 1]
        ], dtype=np.float32)

        desired = geometry_msgs.msg.Wrench()
        desired.force.x = 1.0
        desired.torque.x = 1.0

        result = transformation_utils.transform_wrench(wrench_in, transform_matrix)
        self.assertAlmostEquals(result.force.x, desired.force.x)
        self.assertAlmostEquals(result.force.y, desired.force.y)
        self.assertAlmostEquals(result.force.z, desired.force.z)
        self.assertAlmostEquals(result.torque.x, desired.torque.x)
        self.assertAlmostEquals(result.torque.y, desired.torque.y)
        self.assertAlmostEquals(result.torque.z, desired.torque.z)

        # Test a pitch rotation.
        transform_matrix = np.array([
            [np.cos(angle),  np.sin(angle), 0, 0],
            [0,                          1, 0, 0],
            [-np.sin(angle), np.cos(angle), 1, 0],
            [0,                          0, 0, 1]
        ], dtype=np.float32)

        desired = geometry_msgs.msg.Wrench()
        desired.force.z = -1.0
        desired.torque.z = -1.0

        result = transformation_utils.transform_wrench(wrench_in, transform_matrix)
        self.assertAlmostEquals(result.force.x, desired.force.x)
        self.assertAlmostEquals(result.force.y, desired.force.y)
        self.assertAlmostEquals(result.force.z, desired.force.z)
        self.assertAlmostEquals(result.torque.x, desired.torque.x)
        self.assertAlmostEquals(result.torque.y, desired.torque.y)
        self.assertAlmostEquals(result.torque.z, desired.torque.z)

        # Test a yaw rotation.
        transform_matrix = np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle),  np.cos(angle), 0, 0],
            [0,                          0, 1, 0],
            [0,                          0, 0, 1]
        ], dtype=np.float32)

        desired = geometry_msgs.msg.Wrench()
        desired.force.y = 1.0
        desired.torque.y = 1.0

        result = transformation_utils.transform_wrench(wrench_in, transform_matrix)
        self.assertAlmostEquals(result.force.x, desired.force.x)
        self.assertAlmostEquals(result.force.y, desired.force.y)
        self.assertAlmostEquals(result.force.z, desired.force.z)
        self.assertAlmostEquals(result.torque.x, desired.torque.x)
        self.assertAlmostEquals(result.torque.y, desired.torque.y)
        self.assertAlmostEquals(result.torque.z, desired.torque.z)

    def test_transform_wrench_with_point(self):
        """
        Tests that the transform_wrench raises an error if the 'point' argument is incorrect.

        """
        wrench_in = geometry_msgs.msg.Wrench()
        wrench_in.force.x = 4.0
        transform_matrix = np.array([
            [1, 0, 0, 1],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)

        desired = geometry_msgs.msg.Wrench()
        desired.force.x = 4.0

        # Tests for specifying a wrong 'application_point' argument.
        # Wrong type
        with self.assertRaises(AssertionError) as error_type_int:
            transformation_utils.transform_wrench(wrench_in, transform_matrix, point=1)
        with self.assertRaises(AssertionError) as error_type_list:
            transformation_utils.transform_wrench(
                wrench_in, transform_matrix, point=[1, 0, 0]
            )

        # Wrong number of arguments
        with self.assertRaises(AssertionError) as error_too_many:
            transformation_utils.transform_wrench(
                wrench_in, transform_matrix, point=(1, 0, 0, 0)
            )
        with self.assertRaises(AssertionError) as error_too_few:
            transformation_utils.transform_wrench(wrench_in, transform_matrix, point=(1, 0))

        self.assertIsInstance(error_type_int.exception, AssertionError)
        self.assertIsInstance(error_type_list.exception, AssertionError)
        self.assertIsInstance(error_too_many.exception, AssertionError)
        self.assertIsInstance(error_too_few.exception, AssertionError)

    def test_transform_wrench_translation(self):
        """
        Tests that the transform_wrench can transform a wrench correctly for a translation
        with the force applied on an arbitrary point.

        """
        wrench_in = geometry_msgs.msg.Wrench()
        wrench_in.force.x = 4.0
        transform_matrix = np.array([
            [1, 0, 0, 1],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        application_point = (0, 0, 1)
        transform_matrix_equivalent = np.array([
            [1, 0, 0, 1],
            [0, 1, 0, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1],
        ], dtype=np.float32)

        desired = transformation_utils.transform_wrench(
            wrench_in, transform_matrix_equivalent
        )
        result = transformation_utils.transform_wrench(
            wrench_in, transform_matrix, point=application_point
        )
        self.assertEqual(result, desired)

        # Try a different transformation matrix.
        transform_matrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 1],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        transform_matrix_equivalent = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 2],
            [0, 0, 0, 1],
        ], dtype=np.float32)

        desired = transformation_utils.transform_wrench(
            wrench_in, transform_matrix_equivalent
        )
        result = transformation_utils.transform_wrench(
            wrench_in, transform_matrix, point=application_point
        )
        self.assertEqual(result, desired)

    def test_transform_wrench_rotation(self):
        """
        Tests that the transform_wrench can transform a wrench correctly for a rotation
        with the force applied on an arbitrary point.

        """
        wrench_in = geometry_msgs.msg.Wrench()
        wrench_in.force.x = 1.0
        wrench_in.torque.x = 1.0
        application_point = (1, 0, 0)

        # Test a roll rotation.
        angle = np.pi / 2.0
        transform_matrix_roll = np.array([
            [1,             0,              0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle),  np.cos(angle), 0],
            [0,             0,              0, 1]
        ], dtype=np.float32)

        desired = geometry_msgs.msg.Wrench()
        desired.force.x = 1.0
        desired.torque.x = 1.0

        result = transformation_utils.transform_wrench(
            wrench_in, transform_matrix_roll, point=application_point
        )
        self.assertAlmostEquals(result.force.x, desired.force.x)
        self.assertAlmostEquals(result.force.y, desired.force.y)
        self.assertAlmostEquals(result.force.z, desired.force.z)
        self.assertAlmostEquals(result.torque.x, desired.torque.x)
        self.assertAlmostEquals(result.torque.y, desired.torque.y)
        self.assertAlmostEquals(result.torque.z, desired.torque.z)

        # Test a pitch rotation.
        transform_matrix_pitch = np.array([
            [np.cos(angle),  np.sin(angle), 0, 0],
            [0,                          1, 0, 0],
            [-np.sin(angle), np.cos(angle), 1, 0],
            [0,                          0, 0, 1]
        ], dtype=np.float32)

        desired = geometry_msgs.msg.Wrench()
        desired.force.z = -1.0
        desired.torque.z = -1.0

        result = transformation_utils.transform_wrench(
            wrench_in, transform_matrix_pitch, point=application_point
        )
        self.assertAlmostEquals(result.force.x, desired.force.x)
        self.assertAlmostEquals(result.force.y, desired.force.y)
        self.assertAlmostEquals(result.force.z, desired.force.z)
        self.assertAlmostEquals(result.torque.x, desired.torque.x)
        self.assertAlmostEquals(result.torque.y, desired.torque.y)
        self.assertAlmostEquals(result.torque.z, desired.torque.z)

        # Test a yaw rotation.
        transform_matrix_yaw = np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle),  np.cos(angle), 0, 0],
            [0,                          0, 1, 0],
            [0,                          0, 0, 1]
        ], dtype=np.float32)

        desired = geometry_msgs.msg.Wrench()
        desired.force.y = 1.0
        desired.torque.y = 1.0

        result = transformation_utils.transform_wrench(
            wrench_in, transform_matrix_yaw, point=application_point
        )
        self.assertAlmostEquals(result.force.x, desired.force.x)
        self.assertAlmostEquals(result.force.y, desired.force.y)
        self.assertAlmostEquals(result.force.z, desired.force.z)
        self.assertAlmostEquals(result.torque.x, desired.torque.x)
        self.assertAlmostEquals(result.torque.y, desired.torque.y)
        self.assertAlmostEquals(result.torque.z, desired.torque.z)

        # With a different application point.
        application_point = (0, 1, 0)

        # Test a roll rotation.
        desired = geometry_msgs.msg.Wrench()
        desired.force.x = 1.0
        desired.torque.x = 1.0
        desired.torque.y = 1.0

        result = transformation_utils.transform_wrench(
            wrench_in, transform_matrix_roll, point=application_point
        )
        self.assertAlmostEquals(result.force.x, desired.force.x)
        self.assertAlmostEquals(result.force.y, desired.force.y)
        self.assertAlmostEquals(result.force.z, desired.force.z)
        self.assertAlmostEquals(result.torque.x, desired.torque.x)
        self.assertAlmostEquals(result.torque.y, desired.torque.y)
        self.assertAlmostEquals(result.torque.z, desired.torque.z)

        # Test a pitch rotation.
        desired = geometry_msgs.msg.Wrench()
        desired.force.z = -1.0
        desired.torque.x = -1.0
        desired.torque.y = 1.0
        desired.torque.z = -1.0

        result = transformation_utils.transform_wrench(
            wrench_in, transform_matrix_pitch, point=application_point
        )
        self.assertAlmostEquals(result.force.x, desired.force.x)
        self.assertAlmostEquals(result.force.y, desired.force.y)
        self.assertAlmostEquals(result.force.z, desired.force.z)
        self.assertAlmostEquals(result.torque.x, desired.torque.x)
        self.assertAlmostEquals(result.torque.y, desired.torque.y)
        self.assertAlmostEquals(result.torque.z, desired.torque.z)

        # Test a yaw rotation.
        desired = geometry_msgs.msg.Wrench()
        desired.force.y = 1.0
        desired.torque.y = 1.0
        desired.torque.z = -1.0

        result = transformation_utils.transform_wrench(
            wrench_in, transform_matrix_yaw, point=application_point
        )
        self.assertAlmostEquals(result.force.x, desired.force.x)
        self.assertAlmostEquals(result.force.y, desired.force.y)
        self.assertAlmostEquals(result.force.z, desired.force.z)
        self.assertAlmostEquals(result.torque.x, desired.torque.x)
        self.assertAlmostEquals(result.torque.y, desired.torque.y)
        self.assertAlmostEquals(result.torque.z, desired.torque.z)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_transformation_utils', TestTransformationUtils)
