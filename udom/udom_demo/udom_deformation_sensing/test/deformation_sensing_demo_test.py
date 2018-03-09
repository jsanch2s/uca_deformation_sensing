#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'deformation_sensing_demo' node.

"""

import numpy as np
import rospy
import unittest
import rostest
import std_msgs.msg
import udom_perception_msgs.msg
import udom_modeling_msgs.msg

PKG = 'udom_deformation_sensing'


class TestDeformationSensingDemo(unittest.TestCase):
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
        self.tactile_data = rospy.Publisher(
            '~tactile_data', udom_perception_msgs.msg.BiotacStamped, queue_size=1
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
        self.tactile_data.unregister()

    def test_nodal_force_calculator_node(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        tactile_data = udom_perception_msgs.msg.BiotacStamped()
        tactile_data.header.frame_id = "object_frame"
        tactile_data.electrodes = np.ones(19)

        while not self.wait_for_result:
            self.tactile_data.publish(tactile_data)
            self.event_out.publish('e_start')

        self.assertIsInstance(self.result, udom_modeling_msgs.msg.Mesh)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('deformation_sensing_demo_test')
    rostest.rosrun(PKG, 'deformation_sensing_demo_test', TestDeformationSensingDemo)
