#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Unittest for the sensor_model/tactile_utils.py module.

"""

import numpy as np
import unittest
import rosunit
import udom_sensor_model.tactile_utils as tactile_utils

PKG = 'udom_sensor_model'


class TestTactileUtils(unittest.TestCase):
    """
    Tests the sensor_model/tactile_utils.py module.

    """
    def test_electrode_simple_instantiation(self):
        """
        Tests that the Electrode class is instantiated correctly.

        """
        tactile_utils.Electrode('e1')
        tactile_utils.Electrode('e1', intensity=0.8)

    def test_electrode_simple_instantiation_errors(self):
        """
        Tests that the Electrode class throws an exception if instantiated incorrectly.

        """
        # Missing positional argument
        with self.assertRaises(TypeError) as missing_arg:
            tactile_utils.Electrode()

        # Wrong type arguments
        with self.assertRaises(AssertionError) as error_type_string:
            tactile_utils.Electrode(0)
        with self.assertRaises(AssertionError) as error_type_float:
            tactile_utils.Electrode('e1', intensity='e1')

        # Argument values outside of range.
        with self.assertRaises(AssertionError) as intensity_too_high:
            tactile_utils.Electrode('e1', intensity=1.4)
        with self.assertRaises(AssertionError) as intensity_too_low:
            tactile_utils.Electrode('e1', intensity=-0.1)

        # Incorrect number of position/normal values.
        with self.assertRaises(AssertionError) as pos_error_too_many:
            tactile_utils.Electrode('e1', position=[1, 2, 3, 4])
        with self.assertRaises(AssertionError) as pos_error_too_few:
            tactile_utils.Electrode('e1', position=[2, 3])
        with self.assertRaises(AssertionError) as normal_error_too_many:
            tactile_utils.Electrode('e1', normal=[1, 2, 3, 4])
        with self.assertRaises(AssertionError) as normal_error_too_few:
            tactile_utils.Electrode('e1', normal=[2, 3])

        self.assertIsInstance(missing_arg.exception, TypeError)
        self.assertIsInstance(error_type_string.exception, AssertionError)
        self.assertIsInstance(error_type_float.exception, AssertionError)
        self.assertIsInstance(intensity_too_high.exception, AssertionError)
        self.assertIsInstance(intensity_too_low.exception, AssertionError)
        self.assertIsInstance(pos_error_too_many.exception, AssertionError)
        self.assertIsInstance(pos_error_too_few.exception, AssertionError)
        self.assertIsInstance(normal_error_too_many.exception, AssertionError)
        self.assertIsInstance(normal_error_too_few.exception, AssertionError)

    def test_electrode_assignment(self):
        """
        Tests that the Electrode class handles inappropriate assignments.

        """
        e1 = tactile_utils.Electrode('e1')
        e1.set_intensity(10)
        self.assertTrue(e1.intensity <= 1)
        e1.set_intensity(-10)
        self.assertTrue(e1.intensity >= 0)

    def test_locate_contact(self):
        """
        Tests that the test_locate_contact functions works correctly.

        """
        electrodes = [
            tactile_utils.Electrode('e1', position=(1, 1, 1), normal=(1, 0, 0), intensity=0.5),
            tactile_utils.Electrode('e2', position=(0, 0, 0), normal=(1, 0, 0), intensity=0.6),
            tactile_utils.Electrode('e3', position=(1, 0, 1), normal=(1, 0, 0), intensity=0.8),
        ]
        contact_location = tactile_utils.locate_contact(electrodes)

        self.assertIsInstance(contact_location, np.ndarray)
        self.assertEqual(len(contact_location), 3)


if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_tactile_utils', TestTactileUtils)
