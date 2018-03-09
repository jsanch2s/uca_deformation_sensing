#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'force_merger' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import udom_common_msgs.msg

PKG = 'udom_topic_tools'


class TestForceMerger(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True
        )
        self.force_array_1 = rospy.Publisher(
            '~force_array_1', udom_common_msgs.msg.ForceArray, queue_size=1
        )
        self.force_array_2 = rospy.Publisher(
            '~force_array_2', udom_common_msgs.msg.ForceArray, queue_size=1
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', udom_common_msgs.msg.ForceMultiArray, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.force_array_1.unregister()
        self.force_array_2.unregister()

    def test_force_merger_node(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        force_array_1 = udom_common_msgs.msg.ForceArray()
        force_array_1.header.frame_id = "f1"
        force_array_1.positions = [geometry_msgs.msg.Point(1.0, 1.0, 1.0)]
        force_array_1.wrenches = [
            geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(1.0, 1.0, 1.0))
        ]

        force_array_2 = udom_common_msgs.msg.ForceArray()
        force_array_2.header.frame_id = "f2"
        force_array_2.positions = [geometry_msgs.msg.Point(2.0, 2.0, 2.0)]
        force_array_2.wrenches = [
            geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(2.0, 2.0, 2.0))
        ]

        while not self.wait_for_result:
            self.force_array_1.publish(force_array_1)
            self.force_array_2.publish(force_array_2)
            self.event_out.publish('e_start')

        self.assertIsInstance(self.result, udom_common_msgs.msg.ForceMultiArray)
        self.assertEqual(len(self.result.forces), 2)

        # Checking only for the wrenches, since the sequence (in the header)
        # is not going to be the same.
        self.assertIn(
            force_array_1.wrenches, [
                self.result.forces[0].wrenches, self.result.forces[1].wrenches
            ]
        )
        self.assertIn(
            force_array_2.wrenches, [
                self.result.forces[0].wrenches, self.result.forces[1].wrenches
            ]
        )

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('force_merger_test')
    rostest.rosrun(PKG, 'force_merger_test', TestForceMerger)
