#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'deformation_control_demo_test' node.

"""

import numpy as np
import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import udom_modeling_msgs.msg

PKG = 'udom_deformation_control'


class TestDeformationControlDemo(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result_mesh = None
        self.wait_for_result_mesh = None
        self.result_twist = None
        self.wait_for_result_twist = None

        # publishers
        self.event_out = rospy.Publisher(
            '~event_out', std_msgs.msg.String, latch=True)
        self.wrench_in = rospy.Publisher(
            '~wrench_in', geometry_msgs.msg.WrenchStamped, queue_size=1)
        self.robot_data = rospy.Publisher(
            '~robot_data', std_msgs.msg.Float64MultiArray, queue_size=1)
        self.current_pose = rospy.Publisher(
            '~current_pose', geometry_msgs.msg.PoseStamped, queue_size=1)

        # subscribers
        self.mesh = rospy.Subscriber(
            '~mesh', udom_modeling_msgs.msg.Mesh, self.mesh_callback)
        self.twist = rospy.Subscriber(
            '~twist', geometry_msgs.msg.TwistStamped, self.twist_callback)

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.mesh.unregister()
        self.twist.unregister()
        self.event_out.unregister()
        self.wrench_in.unregister()
        self.robot_data.unregister()
        self.current_pose.unregister()

    def test_coordinator_node(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        robot_data = std_msgs.msg.Float64MultiArray()
        robot_data.data = list(np.ones(47))

        wrench = geometry_msgs.msg.WrenchStamped()
        wrench.header.frame_id = 'map'
        wrench.wrench.force.x = 1.0
        wrench.wrench.force.y = 2.0
        wrench.wrench.force.z = 3.0
        wrench.wrench.torque.x = 1.0
        wrench.wrench.torque.y = 2.0
        wrench.wrench.torque.z = 3.0

        current_pose = geometry_msgs.msg.PoseStamped()
        current_pose.header.frame_id = 'map'
        current_pose.pose.position.x = 1.0

        while not (self.wait_for_result_mesh and self.wait_for_result_twist):
            wrench.header.stamp = rospy.Time.now()
            self.robot_data.publish(robot_data)
            self.current_pose.publish(current_pose)
            self.wrench_in.publish(wrench)
            self.event_out.publish('e_start')

        self.assertIsInstance(self.result_mesh, udom_modeling_msgs.msg.Mesh)
        self.assertIsInstance(self.result_twist, geometry_msgs.msg.TwistStamped)

    def mesh_callback(self, msg):
        self.result_mesh = msg
        self.wait_for_result_mesh = True

    def twist_callback(self, msg):
        self.result_twist = msg
        self.wait_for_result_twist = True


if __name__ == '__main__':
    rospy.init_node('deformation_control_demo_test')
    rostest.rosrun(PKG, 'deformation_control_demo_test', TestDeformationControlDemo)
