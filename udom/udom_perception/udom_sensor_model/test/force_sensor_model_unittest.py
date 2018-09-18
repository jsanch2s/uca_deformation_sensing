#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Unittest for the sensor_model/force_sensor_model_node.py module.

"""

import unittest
import rosunit
import geometry_msgs.msg
import udom_sensor_model.force_sensor_model_node as sensor_model

PKG = 'udom_sensor_model'


class TestForceSensorModel(unittest.TestCase):
    """
    Tests the sensor_model/force_sensor_model_node.py module.

    """
    def test_subtract_wrenches(self):
        """
        Tests that the subtract_wrenches function correctly returns the
        difference between wrenches.

        """
        expected = geometry_msgs.msg.Wrench()
        expected.force.x = -3.0
        expected.force.y = -3.0
        expected.force.z = 6.0
        expected.torque.x = 3.0
        expected.torque.y = 13.0
        expected.torque.z = 5.0

        wrench_1 = geometry_msgs.msg.Wrench()
        wrench_1.force.x = 1.0
        wrench_1.force.y = 2.0
        wrench_1.force.z = 3.0
        wrench_1.torque.x = 4.0
        wrench_1.torque.y = 5.0
        wrench_1.torque.z = 6.0

        wrench_2 = geometry_msgs.msg.Wrench()
        wrench_2.force.x = 4.0
        wrench_2.force.y = 5.0
        wrench_2.force.z = -3.0
        wrench_2.torque.x = 1.0
        wrench_2.torque.y = -8.0
        wrench_2.torque.z = 1.0

        result = sensor_model.subtract_wrenches(wrench_1, wrench_2)

        self.assertEqual(result, expected)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_force_sensor_model', TestForceSensorModel)
