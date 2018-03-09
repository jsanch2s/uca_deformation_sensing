#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'wrench_visualizer' node.

"""

import rospy
import unittest
import rostest
import geometry_msgs.msg
import udom_perception_msgs.msg

PKG = 'udom_visualization'


class TestWrenchVisualizer(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result_point = None
        self.wait_for_result_point = None
        self.result_wrench = None
        self.wait_for_result_wrench = None

        # publishers
        self.contact_info = rospy.Publisher(
            '~contact_info', udom_perception_msgs.msg.ContactInfo, queue_size=1)

        # subscribers
        self.component_output_point = rospy.Subscriber(
            '~component_output_point', geometry_msgs.msg.PointStamped, self.result_point_cb)
        self.component_output_wrench = rospy.Subscriber(
            '~component_output_wrench', geometry_msgs.msg.WrenchStamped, self.result_wrench_cb)

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output_point.unregister()
        self.component_output_wrench.unregister()
        self.contact_info.unregister()

    def test_mesh_to_points_node(self):
        """
        Verifies the node's is able to correctly output the Pose and Wrench messages.

        """
        contact_info = udom_perception_msgs.msg.ContactInfo()
        contact_info.header.stamp = rospy.Time.now()
        contact_info.header.frame_id = "frame"

        point = geometry_msgs.msg.Point(1.0, 0.0, 0.0)
        contact_info.contact_points = [point]

        wrench = geometry_msgs.msg.Wrench()
        wrench.force = geometry_msgs.msg.Vector3(0.0, 0.0, 1.0)
        contact_info.wrenches = [wrench]

        while not (self.wait_for_result_point and self.wait_for_result_wrench):
            self.contact_info.publish(contact_info)

        self.assertIsInstance(self.result_point, geometry_msgs.msg.PointStamped)
        self.assertIsInstance(self.result_wrench, geometry_msgs.msg.WrenchStamped)
        self.assertEqual(self.result_point.header.frame_id, 'frame')
        self.assertEqual(self.result_wrench.header.frame_id, 'frame')
        self.assertAlmostEqual(self.result_point.point.x, 1.0)
        self.assertAlmostEqual(self.result_point.point.y, 0.0)
        self.assertAlmostEqual(self.result_point.point.z, 0.0)
        self.assertAlmostEqual(self.result_wrench.wrench.force.x, 0.0)
        self.assertAlmostEqual(self.result_wrench.wrench.force.y, 0.0)
        self.assertAlmostEqual(self.result_wrench.wrench.force.z, 1.0)
        self.assertAlmostEqual(self.result_wrench.wrench.torque.x, 0.0)
        self.assertAlmostEqual(self.result_wrench.wrench.torque.y, 0.0)
        self.assertAlmostEqual(self.result_wrench.wrench.torque.z, 0.0)

    def result_point_cb(self, msg):
        self.result_point = msg
        self.wait_for_result_point = True

    def result_wrench_cb(self, msg):
        self.result_wrench = msg
        self.wait_for_result_wrench = True


if __name__ == '__main__':
    rospy.init_node('wrench_visualizer_test')
    rostest.rosrun(PKG, 'wrench_visualizer_test', TestWrenchVisualizer)
