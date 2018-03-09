#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Unittest for the nodal_force_computation.py module.

"""

import numpy as np
import unittest
import rosunit
import geometry_msgs.msg
import udom_modeling_msgs.msg
import udom_geometric_transformation.nodal_force_computation as nodal_force_computation

PKG = 'udom_geometric_transformation'


class TestNodalForceComputation(unittest.TestCase):
    """
    Tests the nodal_force_computation.py module.

    """
    def test_point_to_array(self):
        """
        Tests that the function returns a numpy.array given a geometry_msgs.msg.Point
        message.

        """
        point = geometry_msgs.msg.Point()
        point.x = 1.0
        point.y = 2.0
        point.z = 3.0
        desired = np.array([1.0, 2.0, 3.0])

        result = nodal_force_computation.point_to_array(point)
        self.assertEqual(result[0], desired[0])
        self.assertEqual(result[1], desired[1])
        self.assertEqual(result[2], desired[2])

    def test_neighbors(self):
        """
        Tests that the function returns a list with the correct neighboring points.

        """
        point = geometry_msgs.msg.Point(1.9, 0.4, 0.0)

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

        desired_points = [p_5, p_7, p_6]
        desired_idx = [4, 6, 5]
        result_points, result_idx, status = nodal_force_computation.neighbors(point, mesh, n=3)

        self.assertTrue(status)
        self.assertEqual(result_idx[0], desired_idx[0])
        self.assertEqual(result_idx[1], desired_idx[1])
        self.assertEqual(result_idx[2], desired_idx[2])
        self.assertEqual(result_points[0], desired_points[0])
        self.assertEqual(result_points[1], desired_points[1])
        self.assertEqual(result_points[2], desired_points[2])

    def test_neighbors_with_distance(self):
        """
        Tests that the function returns a list with the correct neighboring points that are within
        a distance threshold.

        """
        point = geometry_msgs.msg.Point(1.9, 0.4, 0.0)

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

        desired_points = [p_5, p_7, p_6]
        desired_idx = [4, 6, 5]
        result_points, result_idx, status = nodal_force_computation.neighbors(point, mesh, n=3, distance=10.0)

        self.assertTrue(status)
        self.assertEqual(result_idx[0], desired_idx[0])
        self.assertEqual(result_idx[1], desired_idx[1])
        self.assertEqual(result_idx[2], desired_idx[2])
        self.assertEqual(result_points[0], desired_points[0])
        self.assertEqual(result_points[1], desired_points[1])
        self.assertEqual(result_points[2], desired_points[2])

        result_points, result_idx, status = nodal_force_computation.neighbors(point, mesh, n=3, distance=0.1)
        self.assertFalse(status)
        self.assertEqual(result_idx[0], desired_idx[0])
        self.assertEqual(result_idx[1], desired_idx[1])
        self.assertEqual(result_idx[2], desired_idx[2])
        self.assertEqual(result_points[0], desired_points[0])
        self.assertEqual(result_points[1], desired_points[1])
        self.assertEqual(result_points[2], desired_points[2])

    def test_neighbors_with_exact_match(self):
        """
        Tests that the function returns a list with the correct neighboring points for the
        case where the point is exactly the same as one of the neighbors.

        """
        point = geometry_msgs.msg.Point(0.0, 0.0, 0.0)

        mesh = udom_modeling_msgs.msg.Mesh()

        p_1 = geometry_msgs.msg.Point(0.0, 0.0, 0.0)
        p_2 = geometry_msgs.msg.Point(0.0, 0.0, 0.9)
        p_3 = geometry_msgs.msg.Point(0.0, 0.8, 0.0)
        p_4 = geometry_msgs.msg.Point(0.0, 1.0, 1.0)
        p_5 = geometry_msgs.msg.Point(1.0, 0.0, 0.0)
        p_6 = geometry_msgs.msg.Point(1.0, 0.0, 1.0)
        p_7 = geometry_msgs.msg.Point(1.0, 1.0, 0.0)
        p_8 = geometry_msgs.msg.Point(1.0, 1.0, 1.0)
        mesh.vertices = [p_1, p_2, p_3, p_4, p_5, p_6, p_7, p_8]

        t_1 = udom_modeling_msgs.msg.MeshTetrahedron([3, 2, 4, 0])
        t_2 = udom_modeling_msgs.msg.MeshTetrahedron([3, 1, 4, 0])
        t_3 = udom_modeling_msgs.msg.MeshTetrahedron([3, 6, 3, 4])
        t_4 = udom_modeling_msgs.msg.MeshTetrahedron([3, 6, 7, 4])
        t_5 = udom_modeling_msgs.msg.MeshTetrahedron([3, 5, 1, 4])
        t_6 = udom_modeling_msgs.msg.MeshTetrahedron([3, 5, 7, 4])
        mesh.tetrahedra = [t_1, t_2, t_3, t_4, t_5, t_6]

        desired_points = [p_1, p_3, p_2]
        desired_idx = [0, 2, 1]
        result_points, result_idx, status = nodal_force_computation.neighbors(point, mesh, n=3)

        self.assertTrue(status)
        self.assertEqual(result_idx[0], desired_idx[0])
        self.assertEqual(result_idx[1], desired_idx[1])
        self.assertEqual(result_idx[2], desired_idx[2])
        self.assertEqual(result_points[0], desired_points[0])
        self.assertEqual(result_points[1], desired_points[1])
        self.assertEqual(result_points[2], desired_points[2])

    def test_triangle_area(self):
        """
        Tests that the area of a triangle is computed correctly.

        """
        p_1 = geometry_msgs.msg.Point(0.0, 0.0, 0.0)
        p_2 = geometry_msgs.msg.Point(2.0, 0.0, 0.0)
        p_3 = geometry_msgs.msg.Point(1.0, 1.0, 0.0)

        desired = 1.0
        result = nodal_force_computation.triangle_area(p_1, p_2, p_3)

        self.assertAlmostEqual(result, desired)

    def test_compute_nodal_forces(self):
        """
        Tests that the nodal forces are computed correctly.

        """
        point = geometry_msgs.msg.Point(1.0, 1.0 / 3.0, 0.0)

        force = geometry_msgs.msg.Vector3()
        force.x = 3.0
        force.y = 3.0
        force.z = 3.0

        p_1 = geometry_msgs.msg.Point(0.0, 0.0, 0.0)
        p_2 = geometry_msgs.msg.Point(2.0, 0.0, 0.0)
        p_3 = geometry_msgs.msg.Point(1.0, 1.0, 0.0)
        triangle = geometry_msgs.msg.Polygon()
        triangle.points = [p_1, p_2, p_3]

        desired = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        result = nodal_force_computation.compute_nodal_forces(point, force, triangle)

        np.testing.assert_almost_equal(result, desired)

    def test_compute_nodal_forces_exact_match(self):
        """
        Tests that the nodal forces are computed correctly if the force is applied
        exactly at one of the triangle's node.

        """
        point = geometry_msgs.msg.Point(0.0, 0.0, 0.0)

        force = geometry_msgs.msg.Vector3()
        force.x = 1.0
        force.y = 1.0
        force.z = 1.0

        p_1 = geometry_msgs.msg.Point(0.0, 0.0, 0.0)
        p_2 = geometry_msgs.msg.Point(2.0, 0.0, 0.0)
        p_3 = geometry_msgs.msg.Point(1.0, 1.0, 0.0)
        triangle = geometry_msgs.msg.Polygon()
        triangle.points = [p_1, p_2, p_3]

        desired = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        result = nodal_force_computation.compute_nodal_forces(point, force, triangle)

        np.testing.assert_almost_equal(result, desired)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_nodal_force_computation', TestNodalForceComputation)
