#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'force_transformer' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import udom_common_msgs.msg
import tf2_ros

PKG = 'udom_geometric_transformation'


class TestForceTransformer(unittest.TestCase):
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
        self.force_in = rospy.Publisher(
            '~force_in', udom_common_msgs.msg.ForceMultiArray, queue_size=1
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', udom_common_msgs.msg.ForceArray, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.force_in.unregister()

    def test_force_transformer_node(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        frame_1 = 'f1'
        frame_2 = 'f2'
        # Note: this should match the one specified in the .test file.
        reference_frame = 'object_frame'

        force_array_1 = udom_common_msgs.msg.ForceArray()
        force_array_1.header.frame_id = frame_1
        force_array_1.positions = [geometry_msgs.msg.Point(1.0, 1.0, 1.0)]
        force_array_1.wrenches = [
            geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(1.0, 1.0, 1.0))
        ]

        force_array_2 = udom_common_msgs.msg.ForceArray()
        force_array_2.header.frame_id = frame_2
        force_array_2.positions = [geometry_msgs.msg.Point(2.0, 2.0, 2.0)]
        force_array_2.wrenches = [
            geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(2.0, 2.0, 2.0))
        ]

        force_in = udom_common_msgs.msg.ForceMultiArray()
        force_in.forces = [force_array_1, force_array_2]

        # Desired values.
        point_1 = geometry_msgs.msg.Point(2.0, 1.0, 1.0)
        point_2 = geometry_msgs.msg.Point(4.0, 2.0, 2.0)
        desired_positions = [point_1, point_2]

        force_1 = geometry_msgs.msg.Vector3(1.0, 1.0, 1.0)
        torque_1 = geometry_msgs.msg.Vector3(0.0, -1.0, 1.0)
        force_2 = geometry_msgs.msg.Vector3(2.0, 2.0, 2.0)
        torque_2 = geometry_msgs.msg.Vector3(0.0, -4.0, 4.0)
        wrench_1 = geometry_msgs.msg.Wrench(force_1, torque_1)
        wrench_2 = geometry_msgs.msg.Wrench(force_2, torque_2)
        desired_wrenches = [wrench_1, wrench_2]

        # Set transforms.
        t_1 = geometry_msgs.msg.TransformStamped()
        t_1.header.frame_id = reference_frame
        t_1.child_frame_id = frame_1
        t_1.transform.translation.x = 1.0
        t_1.transform.translation.y = 0.0
        t_1.transform.translation.z = 0.0
        t_1.transform.rotation.x = 0.0
        t_1.transform.rotation.y = 0.0
        t_1.transform.rotation.z = 0.0
        t_1.transform.rotation.w = 1.0

        t_2 = geometry_msgs.msg.TransformStamped()
        t_2.header.frame_id = reference_frame
        t_2.child_frame_id = frame_2
        t_2.transform.translation.x = 2.0
        t_2.transform.translation.y = 0.0
        t_2.transform.translation.z = 0.0
        t_2.transform.rotation.x = 0.0
        t_2.transform.rotation.y = 0.0
        t_2.transform.rotation.z = 0.0
        t_2.transform.rotation.w = 1.0

        broadcaster_1 = tf2_ros.TransformBroadcaster()
        broadcaster_2 = tf2_ros.TransformBroadcaster()

        while not self.wait_for_result:
            t_1.header.stamp = rospy.Time.now()
            t_2.header.stamp = rospy.Time.now()
            broadcaster_1.sendTransform(t_1)
            broadcaster_2.sendTransform(t_2)
            self.force_in.publish(force_in)
            self.event_out.publish('e_start')

        self.assertIsInstance(self.result, udom_common_msgs.msg.ForceArray)
        self.assertEqual(self.result.header.frame_id, reference_frame)
        self.assertEqual(self.result.positions, desired_positions)
        self.assertEqual(self.result.wrenches, desired_wrenches)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('force_transformer_test')
    rostest.rosrun(PKG, 'force_transformer_test', TestForceTransformer)
