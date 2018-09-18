#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'transform_to_pose_converter' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import tf2_ros

PKG = 'udom_geometric_transformation'


class TestTransformToPoseConverter(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True)

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', geometry_msgs.msg.PoseStamped, self.result_callback)

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()

    def test_transform_pose_converter_node(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        # Note: these should match the one specified in the .test file.
        reference_frame = 'map'
        target_frame = 'link_1'

        # Set transform.
        t_1 = geometry_msgs.msg.TransformStamped()
        t_1.header.frame_id = reference_frame
        t_1.child_frame_id = target_frame
        t_1.transform.translation.x = 1.0
        t_1.transform.translation.y = 0.0
        t_1.transform.translation.z = 0.0
        t_1.transform.rotation.x = 0.0
        t_1.transform.rotation.y = 0.0
        t_1.transform.rotation.z = 0.0
        t_1.transform.rotation.w = 1.0

        broadcaster = tf2_ros.TransformBroadcaster()

        while not self.wait_for_result:
            t_1.header.stamp = rospy.Time.now()
            broadcaster.sendTransform(t_1)
            self.event_out.publish('e_start')

        self.assertIsInstance(self.result, geometry_msgs.msg.PoseStamped)
        self.assertEqual(self.result.header.frame_id, reference_frame)
        self.assertEqual(self.result.pose.position.x, 1.0)
        self.assertEqual(self.result.pose.position.y, 0.0)
        self.assertEqual(self.result.pose.position.z, 0.0)
        self.assertEqual(self.result.pose.orientation.w, 1.0)
        self.assertEqual(self.result.pose.orientation.x, 0.0)
        self.assertEqual(self.result.pose.orientation.y, 0.0)
        self.assertEqual(self.result.pose.orientation.z, 0.0)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('transform_to_pose_converter_test')
    rostest.rosrun(PKG, 'transform_to_pose_converter_test', TestTransformToPoseConverter)
