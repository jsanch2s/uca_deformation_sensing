#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'force_estimation_demo_sim' node.

"""

import numpy as np
import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import sr_robot_msgs.msg
import udom_perception_msgs.msg

PKG = 'udom_force_estimation'


class TestForceEstimationDemoSim(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result_contact_data = None
        self.wait_for_result_contact_data = None
        self.result_wrench_data = None
        self.wait_for_result_wrench_data = None

        # publishers
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True
        )
        self.wrench_in = rospy.Publisher(
            '~wrench_in', geometry_msgs.msg.WrenchStamped, queue_size=1
        )
        self.tactile_data = rospy.Publisher(
            '~tactile_data', sr_robot_msgs.msg.BiotacAll, queue_size=1
        )

        # subscribers
        self.wrench_out = rospy.Subscriber(
            '~wrench_out', geometry_msgs.msg.WrenchStamped, self.wrench_out_callback
        )

        self.contact_info_out = rospy.Subscriber(
            '~contact_info_out', udom_perception_msgs.msg.ContactInfo,
            self.contact_info_out_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.wrench_out.unregister()
        self.contact_info_out.unregister()
        self.event_out.unregister()
        self.wrench_in.unregister()
        self.tactile_data.unregister()

    def test_nodal_force_calculator_node(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        wrench_data = geometry_msgs.msg.WrenchStamped()
        wrench_data.wrench.force.x = 0.0
        wrench_data.wrench.force.y = 0.0
        wrench_data.wrench.force.z = 1.0
        tactile_data = sr_robot_msgs.msg.BiotacAll()
        tactile_data.tactiles[0].electrodes = np.ones(19)
        tactile_data.tactiles[0].pac0 = 1000
        tactile_data.tactiles[0].pdc = 1000
        tactile_data.tactiles[1].electrodes = np.ones(19)
        tactile_data.tactiles[1].pac0 = 1000
        tactile_data.tactiles[1].pdc = 1000
        tactile_data.tactiles[2].electrodes = np.ones(19)
        tactile_data.tactiles[2].pac0 = 1000
        tactile_data.tactiles[2].pdc = 1000
        tactile_data.tactiles[3].electrodes = np.ones(19)
        tactile_data.tactiles[3].pac0 = 1000
        tactile_data.tactiles[3].pdc = 1000
        tactile_data.tactiles[4].electrodes = np.ones(19)
        tactile_data.tactiles[4].pac0 = 1000
        tactile_data.tactiles[4].pdc = 1000

        while not (self.wait_for_result_contact_data and self.wait_for_result_wrench_data):
            self.wrench_in.publish(wrench_data)
            self.tactile_data.publish(tactile_data)
            self.event_out.publish('e_start')

        self.assertIsInstance(self.result_contact_data, udom_perception_msgs.msg.ContactInfo)
        self.assertIsInstance(self.result_wrench_data, geometry_msgs.msg.WrenchStamped)

    def contact_info_out_callback(self, msg):
        self.result_contact_data = msg
        self.wait_for_result_contact_data = True

    def wrench_out_callback(self, msg):
        self.result_wrench_data = msg
        self.wait_for_result_wrench_data = True


if __name__ == '__main__':
    rospy.init_node('force_estimation_demo_sim_test')
    rostest.rosrun(PKG, 'force_estimation_demo_sim_test', TestForceEstimationDemoSim)
