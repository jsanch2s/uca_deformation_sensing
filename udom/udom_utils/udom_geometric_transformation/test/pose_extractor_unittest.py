#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Unittest for the pose_extractor_utils.py module.

"""

import unittest
import rosunit
import geometry_msgs.msg
import udom_modeling_msgs.msg
import udom_geometric_transformation.pose_extractor_utils as utils

PKG = 'udom_geometric_transformation'


class TestPoseExtractor(unittest.TestCase):
    """
    Tests the pose_extractor_utils.py module.

    """
    def test_extract_points_zero_based(self):
        """
        Tests that the function returns the correct points message based on a zero-based index.

        """
        point_1 = geometry_msgs.msg.Point()
        point_1.x = 0.5
        point_1.y = 0.3
        point_1.z = 0.9

        point_2 = geometry_msgs.msg.Point()
        point_2.x = 0.0
        point_2.y = 0.0
        point_2.z = 1.0

        point_3 = geometry_msgs.msg.Point()
        point_3.x = 0.0
        point_3.y = 1.0
        point_3.z = 0.0

        desired = [point_1, point_2, point_3]

        mesh = udom_modeling_msgs.msg.Mesh()
        p_1 = geometry_msgs.msg.Point(0.5, 0.3, 0.9)
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

        indices = [0, 1, 2]

        result = utils.extract_points(mesh, indices)
        self.assertEqual(result, desired)

    def test_extract_points_one_based(self):
        """
        Tests that the function returns the correct points message based on a one-based index.

        """
        point_1 = geometry_msgs.msg.Point()
        point_1.x = 0.5
        point_1.y = 0.3
        point_1.z = 0.9

        point_2 = geometry_msgs.msg.Point()
        point_2.x = 0.0
        point_2.y = 0.0
        point_2.z = 1.0

        point_3 = geometry_msgs.msg.Point()
        point_3.x = 0.0
        point_3.y = 1.0
        point_3.z = 0.0

        desired = [point_1, point_2, point_3]

        mesh = udom_modeling_msgs.msg.Mesh()
        p_1 = geometry_msgs.msg.Point(0.5, 0.3, 0.9)
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

        indices = [1, 2, 3]

        result = utils.extract_points(mesh, indices, zero_based=False)
        self.assertEqual(result, desired)

    def test_compute_centroid(self):
        """
        Tests that the function returns the correct centroid.

        """
        desired = geometry_msgs.msg.Point()
        desired.x = 0.1666
        desired.y = 0.4333
        desired.z = 0.6333

        point_1 = geometry_msgs.msg.Point()
        point_1.x = 0.5
        point_1.y = 0.3
        point_1.z = 0.9

        point_2 = geometry_msgs.msg.Point()
        point_2.x = 0.0
        point_2.y = 0.0
        point_2.z = 1.0

        point_3 = geometry_msgs.msg.Point()
        point_3.x = 0.0
        point_3.y = 1.0
        point_3.z = 0.0

        points = [point_1, point_2, point_3]

        result = utils.compute_centroid(points)
        self.assertAlmostEqual(result.x, desired.x, places=3)
        self.assertAlmostEqual(result.y, desired.y, places=3)
        self.assertAlmostEqual(result.z, desired.z, places=3)

    def test_compute_normal_as_quaternion(self):
        """
        Tests that the function returns the correct normal as a quaternion.

        """
        desired = geometry_msgs.msg.Quaternion()
        desired.w = 0.7071
        desired.x = 0.0
        desired.y = 0.0
        desired.z = 0.7071

        point_1 = geometry_msgs.msg.Point()
        point_1.x = 0.0
        point_1.y = 0.0
        point_1.z = 0.0

        point_2 = geometry_msgs.msg.Point()
        point_2.x = 1.0
        point_2.y = 0.0
        point_2.z = 0.0

        point_3 = geometry_msgs.msg.Point()
        point_3.x = 0.0
        point_3.y = 1.0
        point_3.z = 0.0

        points = [point_1, point_2, point_3]

        result = utils.compute_normal_as_quaternion(points)
        self.assertAlmostEqual(result.w, desired.w, places=3)
        self.assertAlmostEqual(result.x, desired.x, places=3)
        self.assertAlmostEqual(result.y, desired.y, places=3)
        self.assertAlmostEqual(result.z, desired.z, places=3)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_pose_extractor', TestPoseExtractor)
