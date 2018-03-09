#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'sensor_model' node using a BioTac model.

"""

import random
import rospy
import unittest
import rostest
import std_msgs.msg
import udom_perception_msgs.msg

PKG = 'udom_sensor_model'


class TestTactileSensorModel(unittest.TestCase):
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
            '~component_output', udom_perception_msgs.msg.ContactInfo, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.tactile_data.unregister()

    def test_tactile_sensor_model(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        expected_result = udom_perception_msgs.msg.ContactInfo()

        tactile_info = udom_perception_msgs.msg.BiotacStamped()
        tactile_info.electrodes = range(19)

        while not self.wait_for_result:
            # Hack to send different electrode values on each time step.
            tactile_info.electrodes[0] = random.randint(10, 200)
            tactile_info.electrodes[1] = random.randint(10, 200)
            tactile_info.electrodes[2] = random.randint(10, 200)
            tactile_info.electrodes[3] = random.randint(10, 200)
            tactile_info.electrodes[4] = random.randint(10, 200)
            tactile_info.electrodes[5] = random.randint(10, 200)
            tactile_info.electrodes[6] = random.randint(10, 200)
            tactile_info.electrodes[7] = random.randint(10, 200)
            tactile_info.electrodes[8] = random.randint(10, 200)
            tactile_info.electrodes[9] = random.randint(10, 200)
            tactile_info.electrodes[10] = random.randint(10, 200)
            tactile_info.electrodes[11] = random.randint(10, 200)
            tactile_info.electrodes[12] = random.randint(10, 200)
            tactile_info.electrodes[13] = random.randint(10, 200)
            tactile_info.electrodes[14] = random.randint(10, 200)
            tactile_info.electrodes[15] = random.randint(10, 200)
            tactile_info.electrodes[16] = random.randint(10, 200)
            tactile_info.electrodes[17] = random.randint(10, 200)
            tactile_info.electrodes[18] = random.randint(10, 200)

            self.tactile_data.publish(tactile_info)
            self.event_out.publish('e_start')

        self.assertEqual(type(self.result.wrenches), type(expected_result.wrenches))

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('tactile_sensor_model_test')
    rostest.rosrun(PKG, 'tactile_sensor_model_test', TestTactileSensorModel)
