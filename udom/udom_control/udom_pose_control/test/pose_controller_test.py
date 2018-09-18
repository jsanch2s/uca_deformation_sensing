#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'pose_controller' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import tf2_ros

PKG = 'udom_pose_control'


class TestPoseController(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.current_pose = rospy.Publisher(
            '~current_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.target_pose = rospy.Publisher(
            '~target_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True)

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', geometry_msgs.msg.TwistStamped, self.result_callback)

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.current_pose.unregister()
        self.target_pose.unregister()
        self.event_out.unregister()

    def test_pose_controller(self):
        """
        Verifies the node outputs a twist message based on the difference between two poses.
        Note: this is not a functionality test.

        """
        # Note: this should match the one specified in the .test file.
        reference_frame = 'end_effector_frame'
        pose_frame = 'base_frame'

        expected = geometry_msgs.msg.TwistStamped()
        expected.header.frame_id = reference_frame

        current_pose = geometry_msgs.msg.PoseStamped()
        target_pose = geometry_msgs.msg.PoseStamped()

        current_pose.pose.position.x = 1.0
        current_pose.pose.orientation.w = 1.0

        target_pose.pose.orientation.x = 0.707
        target_pose.pose.orientation.w = 0.707

        current_pose.header.frame_id = pose_frame
        target_pose.header.frame_id = pose_frame
        current_pose.header.stamp = rospy.Time.now()
        target_pose.header.stamp = rospy.Time.now()

        # Set transform.
        my_transform = geometry_msgs.msg.TransformStamped()
        my_transform.header.frame_id = reference_frame
        my_transform.child_frame_id = pose_frame
        my_transform.transform.translation.x = 1.0
        my_transform.transform.translation.y = 0.0
        my_transform.transform.translation.z = 0.0
        my_transform.transform.rotation.x = 0.0
        my_transform.transform.rotation.y = 0.0
        my_transform.transform.rotation.z = 0.0
        my_transform.transform.rotation.w = 1.0
        broadcaster = tf2_ros.TransformBroadcaster()

        while not self.wait_for_result:
            my_transform.header.stamp = rospy.Time.now()
            broadcaster.sendTransform(my_transform)
            self.current_pose.publish(current_pose)
            self.target_pose.publish(target_pose)
            self.event_out.publish('e_start')

        self.assertIsInstance(self.result, geometry_msgs.msg.TwistStamped)
        self.assertEqual(self.result.header.frame_id, expected.header.frame_id)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('pose_controller_test')
    rostest.rosrun(PKG, 'pose_controller', TestPoseController)
