#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'tactile_demux' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import sr_robot_msgs.msg
import udom_perception_msgs.msg

PKG = 'udom_topic_tools'


class TestTactileDemux(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=10)
        self.tactile_data = rospy.Publisher('~tactile_data', sr_robot_msgs.msg.BiotacAll, queue_size=10)

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', udom_perception_msgs.msg.BiotacStamped, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.event_out.unregister()
        self.tactile_data.unregister()
        self.component_output.unregister()

    def test_tactile_demux(self):
        """
        Verifies that the node correctly outputs a udom_perception_msgs/BiotacStamped message
        when it receives a sr_robot_msgs/BiotacAll message.

        """
        # These data must match the one specified in the .test file.
        # sensor_index = 0
        # sensor_frame = 'first_finger'
        electrodes = [
            2541, 3190, 3199, 2981, 3129, 2802, 2699, 2832, 2814, 2945,
            2452, 3167, 3191, 3001, 3142, 2870, 3096, 3171, 2972
        ]
        tactile_data = sr_robot_msgs.msg.BiotacAll()
        tactile_data.tactiles[0].pac0 = 1.0
        tactile_data.tactiles[0].pac1 = 2.0
        tactile_data.tactiles[0].pdc = 3.0
        tactile_data.tactiles[0].tac = 4.0
        tactile_data.tactiles[0].tdc = 5.0
        tactile_data.tactiles[0].electrodes = electrodes

        while not self.wait_for_result:
            self.tactile_data.publish(tactile_data)
            self.event_out.publish('e_start')

        self.assertEqual(type(self.result), type(udom_perception_msgs.msg.BiotacStamped()))
        self.assertEqual(self.result.header.frame_id, 'first_finger')
        self.assertEqual(self.result.pac0, 1.0)
        self.assertEqual(self.result.pac1, 2.0)
        self.assertEqual(self.result.pdc, 3.0)
        self.assertEqual(self.result.tac, 4.0)
        self.assertEqual(self.result.tdc, 5.0)
        self.assertItemsEqual(self.result.electrodes, electrodes)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('tactile_demux_test')
    rostest.rosrun(PKG, 'tactile_demux_test', TestTactileDemux)
