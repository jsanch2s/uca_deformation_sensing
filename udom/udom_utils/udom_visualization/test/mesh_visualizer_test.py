#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Integration test for the 'mesh_visualizer' node.

"""

import rospy
import unittest
import rostest
import geometry_msgs.msg
import visualization_msgs.msg
import udom_modeling_msgs.msg

PKG = 'udom_visualization'


class TestMeshVisualizer(unittest.TestCase):
    def setUp(self):
        """
        Sets up the test fixture before exercising it.

        """
        # params
        self.result = None
        self.wait_for_result = None

        # publishers
        self.mesh = rospy.Publisher('~mesh', udom_modeling_msgs.msg.Mesh, queue_size=1)

        # subscribers
        self.component_output = rospy.Subscriber(
            '~component_output', visualization_msgs.msg.Marker, self.result_callback
        )

    def tearDown(self):
        """
        Deconstructs the test fixture after testing it.

        """
        self.component_output.unregister()
        self.mesh.unregister()

    def test_mesh_to_points_node(self):
        """
        Verifies the node's is able to correctly transform a mesh into a marker message.

        """
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
            self.mesh.publish(mesh)

        self.assertIsInstance(self.result, visualization_msgs.msg.Marker)

        for ii, vertex in enumerate(mesh.vertices):
            self.assertAlmostEqual(self.result.points[ii], vertex)

    def result_callback(self, msg):
        self.result = msg
        self.wait_for_result = True


if __name__ == '__main__':
    rospy.init_node('mesh_visualizer_test')
    rostest.rosrun(PKG, 'mesh_visualizer_test', TestMeshVisualizer)
