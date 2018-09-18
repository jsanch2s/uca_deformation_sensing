#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'wrench_filter' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg

PKG = 'udom_sensor_model'


class TestWrenchFilter(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result_wrench = None
        self.wait_for_wrench = None

        # publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, latch=True)
        self.wrench_in = rospy.Publisher(
            '~wrench_in', geometry_msgs.msg.WrenchStamped, queue_size=1)

        # subscribers
        self.wrench_out = rospy.Subscriber(
            '~wrench_out', geometry_msgs.msg.WrenchStamped, self.result_wrench_callback)

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.wrench_out.unregister()
        self.event_out.unregister()
        self.wrench_in.unregister()

    def test_wrench_filter(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        expected_wrench = geometry_msgs.msg.WrenchStamped()
        expected_wrench.header.frame_id = 'my_frame'

        wrench_data = geometry_msgs.msg.WrenchStamped()
        wrench_data.header.frame_id = 'my_frame'
        wrench_data.wrench.force.x = 1.0
        wrench_data.wrench.force.y = 2.0
        wrench_data.wrench.force.z = 3.0
        wrench_data.wrench.torque.x = 1.0
        wrench_data.wrench.torque.y = 2.0
        wrench_data.wrench.torque.z = 3.0

        while not self.wait_for_wrench:
            wrench_data.header.stamp = rospy.Time.now()
            self.wrench_in.publish(wrench_data)
            self.event_out.publish('e_start')

        self.assertEqual(type(self.result_wrench), type(expected_wrench))
        self.assertEqual(self.result_wrench.header.frame_id, expected_wrench.header.frame_id)

    def result_wrench_callback(self, msg):
        self.result_wrench = msg
        self.wait_for_wrench = True


if __name__ == '__main__':
    rospy.init_node('wrench_filter_test')
    rostest.rosrun(PKG, 'wrench_filter_test', TestWrenchFilter)
