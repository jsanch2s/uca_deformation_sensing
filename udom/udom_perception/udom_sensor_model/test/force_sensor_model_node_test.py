#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'force_sensor_model' node.

"""

import numpy as np
import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import udom_common_msgs.msg

PKG = 'udom_sensor_model'


class TestForceSensorModel(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result_wrench = None
        self.result_force_array = None
        self.wait_for_wrench = None
        self.wait_for_force_array = None

        # publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, latch=True)
        self.robot_data = rospy.Publisher(
            '~robot_data', std_msgs.msg.Float64MultiArray, queue_size=1)
        self.wrench_in = rospy.Publisher(
            '~wrench_in', geometry_msgs.msg.WrenchStamped, queue_size=1)

        # subscribers
        self.wrench_out = rospy.Subscriber(
            '~wrench_out', geometry_msgs.msg.WrenchStamped, self.result_wrench_callback)
        self.force_array = rospy.Subscriber(
            '~force_array', udom_common_msgs.msg.ForceMultiArray,
            self.result_force_array_callback)

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.wrench_out.unregister()
        self.force_array.unregister()
        self.event_out.unregister()
        self.robot_data.unregister()
        self.wrench_in.unregister()

    def test_force_sensor_model(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        expected_wrench = geometry_msgs.msg.WrenchStamped()
        expected_wrench.header.frame_id = 'my_frame'
        expected_force_array = udom_common_msgs.msg.ForceMultiArray()

        robot_data = std_msgs.msg.Float64MultiArray()
        robot_data.data = list(np.ones(47))

        wrench_data = geometry_msgs.msg.WrenchStamped()
        wrench_data.header.frame_id = 'my_frame'
        wrench_data.wrench.force.x = 1.0
        wrench_data.wrench.force.y = 2.0
        wrench_data.wrench.force.z = 3.0
        wrench_data.wrench.torque.x = 1.0
        wrench_data.wrench.torque.y = 2.0
        wrench_data.wrench.torque.z = 3.0

        while not (self.wait_for_wrench and self.wait_for_force_array):
            wrench_data.header.stamp = rospy.Time.now()
            self.robot_data.publish(robot_data)
            self.wrench_in.publish(wrench_data)
            self.event_out.publish('e_start')

        self.assertEqual(type(self.result_wrench), type(expected_wrench))
        self.assertEqual(self.result_wrench.header.frame_id, expected_wrench.header.frame_id)
        self.assertEqual(type(self.result_force_array), type(expected_force_array))

    def result_wrench_callback(self, msg):
        self.result_wrench = msg
        self.wait_for_wrench = True

    def result_force_array_callback(self, msg):
        self.result_force_array = msg
        self.wait_for_force_array = True


if __name__ == '__main__':
    rospy.init_node('force_sensor_model_test')
    rostest.rosrun(PKG, 'force_sensor_model_test', TestForceSensorModel)
