#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'deformation_model' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import udom_modeling_msgs.msg

PKG = 'udom_deformation_modeling'


class TestDeformationModel(unittest.TestCase):
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
        self.force_info = rospy.Publisher(
            '~force_info', std_msgs.msg.Float32MultiArray, queue_size=1
        )

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', udom_modeling_msgs.msg.Mesh, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.force_info.unregister()

    def test_deformation_model_node(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        force_info = std_msgs.msg.Float32MultiArray()
        force_info.data = [
            1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        while not self.wait_for_result:
            self.force_info.publish(force_info)
            self.event_out.publish('e_start')

        self.assertIsInstance(self.result, udom_modeling_msgs.msg.Mesh)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('deformation_model_test')
    rostest.rosrun(PKG, 'deformation_model_test', TestDeformationModel)
