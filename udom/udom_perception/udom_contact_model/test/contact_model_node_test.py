#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'contact_model' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import udom_common_msgs.msg
import udom_perception_msgs.msg

PKG = 'udom_contact_model'


class TestContactModel(unittest.TestCase):
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
        self.contact_info = rospy.Publisher(
            '~contact_info', udom_perception_msgs.msg.ContactInfo, queue_size=1
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
        self.contact_info.unregister()

    def test_contact_model_node(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        expected_result = udom_common_msgs.msg.ForceArray()

        contact_info = udom_perception_msgs.msg.ContactInfo()

        while not self.wait_for_result:
            self.contact_info.publish(contact_info)
            self.event_out.publish('e_start')

        self.assertEqual(type(self.result.wrenches), type(expected_result.wrenches))

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('contact_model_test')
    rostest.rosrun(PKG, 'contact_model_test', TestContactModel)
