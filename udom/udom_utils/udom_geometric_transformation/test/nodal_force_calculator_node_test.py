#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'nodal_force_calculator' node.

"""

import rospy
import unittest
import rostest
import std_msgs.msg
import geometry_msgs.msg
import udom_common_msgs.msg
import udom_modeling_msgs.msg

PKG = 'udom_geometric_transformation'


class TestNodalForceCalculator(unittest.TestCase):
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
            '~force_in', udom_common_msgs.msg.ForceArray, queue_size=1
        )
        self.mesh = rospy.Publisher('~mesh', udom_modeling_msgs.msg.Mesh, queue_size=1)

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', std_msgs.msg.Float32MultiArray, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.event_out.unregister()
        self.force_in.unregister()
        self.mesh.unregister()

    def test_nodal_force_calculator_node(self):
        """
        Verifies the node's interface is correct.
        Note: this is not a functionality test.

        """
        force_in = udom_common_msgs.msg.ForceArray()
        force_in.header.frame_id = "object_frame"
        force_in.positions = [geometry_msgs.msg.Point(0.0, 0.0, 0.0)]
        force_in.wrenches = [
            geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(1.0, 1.0, 1.0))
        ]

        mesh = udom_modeling_msgs.msg.Mesh()

        p_1 = geometry_msgs.msg.Point(0.0, 0.0, 0.0)
        p_2 = geometry_msgs.msg.Point(0.0, 0.0, 1.0)
        p_3 = geometry_msgs.msg.Point(0.0, 1.0, 0.0)
        p_4 = geometry_msgs.msg.Point(0.0, 1.0, 1.0)
        p_5 = geometry_msgs.msg.Point(1.0, 0.0, 0.0)
        p_6 = geometry_msgs.msg.Point(1.0, 0.0, 1.0)
        p_7 = geometry_msgs.msg.Point(1.0, 1.0, 0.0)
        p_8 = geometry_msgs.msg.Point(1.0, 1.0, 1.0)
        mesh.vertices = [p_1, p_2, p_3, p_4, p_5, p_6, p_7, p_8]

        t_1 = udom_modeling_msgs.msg.MeshTetrahedron([3, 2, 4, 0])
        t_2 = udom_modeling_msgs.msg.MeshTetrahedron([3, 1, 4, 0])
        t_3 = udom_modeling_msgs.msg.MeshTetrahedron([3, 6, 2, 4])
        t_4 = udom_modeling_msgs.msg.MeshTetrahedron([3, 6, 7, 4])
        t_5 = udom_modeling_msgs.msg.MeshTetrahedron([3, 5, 1, 4])
        t_6 = udom_modeling_msgs.msg.MeshTetrahedron([3, 5, 7, 4])
        mesh.tetrahedra = [t_1, t_2, t_3, t_4, t_5, t_6]

        while not self.wait_for_result:
            self.force_in.publish(force_in)
            self.mesh.publish(mesh)
            self.event_out.publish('e_start')

        self.assertIsInstance(self.result, std_msgs.msg.Float32MultiArray)

        # There should be a vector force for each node.
        self.assertEqual(len(self.result.data), len(mesh.vertices) * 3)

        # Since the force is applied to the first node, then the force should only be
        # distributed on the first node and it should be zero for the remaining nodes.
        self.assertAlmostEqual(self.result.data[0], 1.0)
        self.assertAlmostEqual(self.result.data[1], 1.0)
        self.assertAlmostEqual(self.result.data[2], 1.0)
        self.assertAlmostEqual(self.result.data[3], 0.0)
        self.assertAlmostEqual(self.result.data[4], 0.0)
        self.assertAlmostEqual(self.result.data[5], 0.0)
        self.assertAlmostEqual(self.result.data[6], 0.0)
        self.assertAlmostEqual(self.result.data[7], 0.0)
        self.assertAlmostEqual(self.result.data[8], 0.0)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('nodal_force_calculator_test')
    rostest.rosrun(PKG, 'nodal_force_calculator_test', TestNodalForceCalculator)
