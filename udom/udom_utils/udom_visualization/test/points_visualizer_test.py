#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'points_visualizer' node.

"""

import rospy
import unittest
import rostest
import geometry_msgs.msg
import visualization_msgs.msg

PKG = 'udom_visualization'


class TestPointsVisualizer(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.points = rospy.Publisher('~points', visualization_msgs.msg.Marker, queue_size=1)

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', visualization_msgs.msg.Marker, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.points.unregister()

    def test_mesh_to_points_node(self):
        """
        Verifies the node's is able to correctly fill the marker_out message.

        """
        marker = visualization_msgs.msg.Marker()

        p_1 = geometry_msgs.msg.Point(0.0, 0.0, 0.0)
        p_2 = geometry_msgs.msg.Point(0.0, 0.0, 1.0)
        p_3 = geometry_msgs.msg.Point(0.0, 1.0, 0.0)
        p_4 = geometry_msgs.msg.Point(0.0, 1.0, 1.0)
        p_5 = geometry_msgs.msg.Point(1.0, 0.0, 0.0)
        p_6 = geometry_msgs.msg.Point(1.0, 0.0, 1.0)
        p_7 = geometry_msgs.msg.Point(1.0, 1.0, 0.0)
        p_8 = geometry_msgs.msg.Point(1.0, 1.0, 1.0)
        marker.points = [p_1, p_2, p_3, p_4, p_5, p_6, p_7, p_8]

        while not self.wait_for_result:
            self.points.publish(marker)

        self.assertIsInstance(self.result, visualization_msgs.msg.Marker)
        # This param should match the .test launch file
        self.assertEqual(self.result.header.frame_id, 'test_frame')

        for ii, point in enumerate(marker.points):
            self.assertAlmostEqual(self.result.points[ii], point)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('points_visualizer_test')
    rostest.rosrun(PKG, 'points_visualizer_test', TestPointsVisualizer)
