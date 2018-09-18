#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Unittest for the quaternion_algebra.py module.

"""

import unittest
import rosunit
import geometry_msgs.msg
import udom_control_msgs.msg
import udom_geometric_transformation.quaternion_algebra as qa

PKG = 'udom_geometric_transformation'


class TestQuaternionAlgebra(unittest.TestCase):
    """
    Tests the quaternion_algebra.py module.

    """
    def test_scale_dq(self):
        """
        Tests that the function returns the correct scaled dual quaternion.

        """
        scale = 3
        q_in = udom_control_msgs.msg.DualQuaternion()
        q_in.real = geometry_msgs.msg.Quaternion(w=0.1825, x=0.3651, y=0.5477, z=0.7303)
        q_in.dual = geometry_msgs.msg.Quaternion(w=-0.6108, x=-0.4984, y=0.5786, z=0.209)

        expected = udom_control_msgs.msg.DualQuaternion()
        expected.real = geometry_msgs.msg.Quaternion(
            w=0.1825 * scale, x=0.3651 * scale, y=0.5477 * scale, z=0.7303 * scale)
        expected.dual = geometry_msgs.msg.Quaternion(
            w=-0.6108 * scale, x=-0.4984 * scale, y=0.5786 * scale, z=0.209 * scale)

        result = qa.scale_dq(q_in, scale)

        self.assertIsInstance(result, udom_control_msgs.msg.DualQuaternion)
        self.assertAlmostEqual(result.real.w, expected.real.w, places=4)
        self.assertAlmostEqual(result.real.x, expected.real.x, places=4)
        self.assertAlmostEqual(result.real.y, expected.real.y, places=4)
        self.assertAlmostEqual(result.real.z, expected.real.z, places=4)
        self.assertAlmostEqual(result.dual.w, expected.dual.w, places=4)
        self.assertAlmostEqual(result.dual.x, expected.dual.x, places=4)
        self.assertAlmostEqual(result.dual.y, expected.dual.y, places=4)
        self.assertAlmostEqual(result.dual.z, expected.dual.z, places=4)

    def test_multiply(self):
        """
        Tests that the function returns the correct product between two quaternions.

        """
        q_1 = geometry_msgs.msg.Quaternion(w=0.1825, x=0.3651, y=0.5477, z=0.7303)
        q_2 = geometry_msgs.msg.Quaternion(w=0.1761, x=0.4402, y=0.8805, z=0.0)

        expected = geometry_msgs.msg.Quaternion(w=-0.6108, x=-0.4984, y=0.5786, z=0.209)

        result = qa.multiply(q_1, q_2)

        self.assertIsInstance(result, geometry_msgs.msg.Quaternion)
        self.assertAlmostEqual(result.w, expected.w, places=4)
        self.assertAlmostEqual(result.x, expected.x, places=4)
        self.assertAlmostEqual(result.y, expected.y, places=4)
        self.assertAlmostEqual(result.z, expected.z, places=4)

    def test_pose_to_dq(self):
        """
        Tests that the function returns a dual quaternion when given a pose.

        """
        pose = geometry_msgs.msg.Pose()
        pose.orientation.w = 1.0

        expected = udom_control_msgs.msg.DualQuaternion()
        expected.real = geometry_msgs.msg.Quaternion()
        expected.real.w = 1.0
        expected.dual = geometry_msgs.msg.Quaternion()
        expected.dual.w = 0.0

        result = qa.pose_to_dq(pose)
        self.assertIsInstance(result, udom_control_msgs.msg.DualQuaternion)
        self.assertEqual(result, expected)

    def test_dq_to_pose(self):
        """
        Tests that the function returns a pose when given a dual quaternion.

        """
        dq = udom_control_msgs.msg.DualQuaternion()
        dq.real = geometry_msgs.msg.Quaternion()
        dq.real.w = 1.0
        dq.dual = geometry_msgs.msg.Quaternion()
        dq.dual.w = 0.0

        expected = geometry_msgs.msg.Pose()
        expected.orientation.w = 1.0

        result = qa.dq_to_pose(dq)
        self.assertIsInstance(result, geometry_msgs.msg.Pose)
        self.assertEqual(result, expected)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_quaternion_algebra', TestQuaternionAlgebra)
