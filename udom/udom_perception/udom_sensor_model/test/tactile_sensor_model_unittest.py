#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Unittest for the sensor_model/tactile_sensor.py module.

"""

import numpy as np
import unittest
import rosunit
import geometry_msgs.msg
import udom_perception_msgs.msg
import sensor_model.tactile_sensor as tactile_sensor

PKG = 'udom_sensor_model'


class TestTactileSensorModel(unittest.TestCase):
    """
    Tests the sensor_model/tactile_sensor.py module.

    """
    def __init__(self, *args, **kwargs):
        super(TestTactileSensorModel, self).__init__(*args, **kwargs)

        # Set up some data to use across the tests.
        self.tactile_data = udom_perception_msgs.msg.BiotacStamped()
        self.tactile_data.header.frame_id = "test_frame"
        self.tactile_data.electrodes = range(19)

    def test_biotac_simple_instantiation(self):
        """
        Tests that the BioTacSimple class is instantiated correctly.

        """
        tactile_sensor.BioTacSimple()
        tactile_sensor.BioTacSimple("my_surface")
        tactile_sensor.BioTacSimple("my_surface", (1, 1, 1))

    def test_biotac_simple_instantiation_errors(self):
        """
        Tests that the BioTacSimple class throws an exception if instantiated incorrectly.

        """
        # Wrong type for scaling values.
        with self.assertRaises(AssertionError) as error_type_int:
            tactile_sensor.BioTacSimple("my_surface", 3)
        with self.assertRaises(AssertionError) as error_type_list:
            tactile_sensor.BioTacSimple("my_surface", [3])

        # Incorrect number of scaling values.
        with self.assertRaises(AssertionError) as error_too_many:
            tactile_sensor.BioTacSimple("my_surface", (1, 1, 1, 2))
        with self.assertRaises(AssertionError) as error_too_few:
            tactile_sensor.BioTacSimple("my_surface", (1, 2))

        self.assertIsInstance(error_type_int.exception, AssertionError)
        self.assertIsInstance(error_type_list.exception, AssertionError)
        self.assertIsInstance(error_too_many.exception, AssertionError)
        self.assertIsInstance(error_too_few.exception, AssertionError)

    def test_biotac_simple_compute_wrenches(self):
        """
        Tests that a BioTacSimple object can compute wrenches.

        """
        # Check the result's type.
        biotac = tactile_sensor.BioTacSimple("my_surface", (1, 1, 1))
        # Only testing for the first wrench.
        result = biotac.contact_wrenches(self.tactile_data)[0]

        self.assertIsInstance(result, geometry_msgs.msg.Wrench)

        # Check force computation.
        desired = np.array([28.0, 8.4, 28.0])
        self.tactile_data.electrodes = [2, 3, 4, 5]
        biotac.scale_factor = [1, 0.2, 0.5]
        biotac.normals = np.array([
            [2, 2, 2, 2],
            [3, 3, 3, 3],
            [4, 4, 4, 4]
        ])
        # Only testing for the first force.
        result = biotac.contact_wrenches(self.tactile_data)[0]

        self.assertAlmostEqual(result.force.x, desired[0])
        self.assertAlmostEqual(result.force.y, desired[1])
        self.assertAlmostEqual(result.force.z, desired[2])

        # Check torque computation.
        desired = np.array([40.0, -80.0, 40.0])
        # To account for the conversion from mm to meters.
        desired /= 1000.0
        biotac.positions = np.array([
            [1, 2, 3, 4],
            [1, 2, 3, 4],
            [1, 2, 3, 4]
        ])
        # Only testing for the first torque.
        result = biotac.contact_wrenches(self.tactile_data)[0]

        self.assertAlmostEqual(result.torque.x, desired[0])
        self.assertAlmostEqual(result.torque.y, desired[1])
        self.assertAlmostEqual(result.torque.z, desired[2])

        # Set back the original values of the tactile data.
        self.tactile_data.electrodes = range(19)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_tactile_sensor_model', TestTactileSensorModel)
