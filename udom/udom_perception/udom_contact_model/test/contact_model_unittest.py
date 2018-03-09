#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Unittest for the contact_model.py module.

"""

import unittest
import rosunit
import geometry_msgs.msg
import udom_common_msgs.msg
import udom_perception_msgs.msg
import udom_contact_model.contact_model as contact_model

PKG = 'udom_contact_model'


class TestContactModel(unittest.TestCase):
    """
    Tests the contact_model.py module.

    """
    def test_biotac_simple_model(self):
        """
        Tests that a BioTacSimple contact model can compute the force array.

        """
        contact_info = udom_perception_msgs.msg.ContactInfo()

        # Check the result's type.
        model = contact_model.BioTacSimple()
        result = model.force_array(contact_info)

        self.assertIsInstance(result, udom_common_msgs.msg.ForceArray)

        # Check force computation.
        contact_info.contact_points = [
            geometry_msgs.msg.Point(0, 0, 0), geometry_msgs.msg.Point(1, 1, 1),
            geometry_msgs.msg.Point(2, 2, 2), geometry_msgs.msg.Point(3, 3, 3),
        ]
        contact_info.wrenches = [
            geometry_msgs.msg.Wrench(),
            geometry_msgs.msg.Wrench(),
            geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(1, 1, 1)),
            geometry_msgs.msg.Wrench()
        ]

        # The result should be the third element since it is the one with the largest force.
        desired = udom_common_msgs.msg.ForceArray()
        desired.positions = [geometry_msgs.msg.Point(2, 2, 2)]
        desired.wrenches = [geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(1, 1, 1))]

        result = model.force_array(contact_info)

        self.assertEqual(result, desired)

    def test_biotac_simple_model_filtered(self):
        """
        Tests that a BioTacSimple contact model can compute the force array with
        the filter option.

        """
        model = contact_model.BioTacSimple()
        contact_info = udom_perception_msgs.msg.ContactInfo()

        # Check force computation.
        contact_info.contact_points = [geometry_msgs.msg.Point(2, 2, 2)]
        contact_info.wrenches = [
            geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(0, 0, 1))]

        # The result should be the same since the threshold is below the largest force.
        desired = udom_common_msgs.msg.ForceArray()
        desired.positions = [geometry_msgs.msg.Point(2, 2, 2)]
        desired.wrenches = [geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(0, 0, 1))]

        result = model.force_array(contact_info, threshold=0.5)

        self.assertEqual(result, desired)

        # The result should be zero since the threshold is above the largest force.
        desired = udom_common_msgs.msg.ForceArray()
        desired.positions = [geometry_msgs.msg.Point(2, 2, 2)]
        desired.wrenches = [geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(0, 0, 0))]

        result = model.force_array(contact_info, threshold=1.5)

        self.assertEqual(result, desired)

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_contact_model', TestContactModel)
