#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This module contains a collection of functions used by the analytical based
force sensor model.

"""

import numpy as np
import geometry_msgs.msg
import tf.transformations as transformations


def compute_v_g_init(g):
    """
    Computes the matrix (V_g_{init}) that considers the gravitational forces that are
    eliminated when the force-torque sensor is zeroed. Based on [1].

    [1] Kubus, Daniel, Torsten Kroger, and Friedrich M. Wahl. "On-line rigid object
    recognition and pose estimation based on inertial parameters." Intelligent Robots
    and Systems, 2007. IROS 2007.

    :param g: Gravity vector.
    :type g: geometry_msgs.msg.Vector3

    :return: Gravitational matrix
    :rtype: np.array[6, 10]

    """
    term_1 = np.array([g.x, 0.0, 0.0, 0.0])
    term_2 = np.array([g.y, 0.0, 0.0, 0.0])
    term_3 = np.array([g.z, 0.0, 0.0, 0.0])

    term_4 = np.array([0.0, 0.0, g.z, -g.y])
    term_5 = np.array([0.0, -g.z, 0.0, g.x])
    term_6 = np.array([0.0, g.y, -g.x, 0.0])

    row_1 = np.concatenate((term_1, np.zeros(6)))
    row_2 = np.concatenate((term_2, np.zeros(6)))
    row_3 = np.concatenate((term_3, np.zeros(6)))
    row_4 = np.concatenate((term_4, np.zeros(6)))
    row_5 = np.concatenate((term_5, np.zeros(6)))
    row_6 = np.concatenate((term_6, np.zeros(6)))

    return np.array([row_1, row_2, row_3, row_4, row_5, row_6])


def compute_v_matrix(acceleration, angular_velocity, gravity):
    """
    Computes the matrix (V) mapping the inertial parameters to the measured wrench.
    Based on [1].

    [1] Kubus, Daniel, Torsten Kroger, and Friedrich M. Wahl. "On-line rigid object
    recognition and pose estimation based on inertial parameters." Intelligent Robots
    and Systems, 2007. IROS 2007.

    :param acceleration: Linear and angular acceleration on the sensor frame w.r.t. the
        sensor frame.
    :type acceleration: geometry_msgs.msg.Accel

    :param angular_velocity: Angular velocity of the sensor w.r.t. the sensor frame.
    :type angular_velocity: geometry_msgs.msg.Vector3

    :param gravity: Gravity vector expressed on the sensor frame.
    :type gravity: geometry_msgs.msg.Vector3

    :return: The V matrix.
    :rtype: np.array[6, 10]

    """
    a = acceleration.linear
    alpha = acceleration.angular
    omega = angular_velocity
    g = gravity

    term_1 = np.array([
        a.x - g.x, -omega.y**2 - omega.z**2, omega.x * omega.y - alpha.z, omega.x * omega.z + alpha.y])
    term_2 = np.array([
        a.y - g.y, omega.x * omega.y + alpha.z, -omega.x**2 - omega.z**2, omega.y * omega.z - alpha.x])
    term_3 = np.array([
        a.z - g.z, omega.x * omega.z - alpha.y, omega.y * omega.z + alpha.x, -omega.y**2 - omega.x**2])

    term_4 = np.array([
        alpha.x, alpha.y - omega.x * omega.z, alpha.z + omega.x * omega.y, -omega.y * omega.z, -omega.y**2 - omega.z**2, omega.y * omega.z])
    term_5 = np.array([
        omega.x * omega.z, alpha.x + omega.y * omega.z, -omega.z**2 - omega.x**2,
        alpha.y, alpha.z - omega.x * omega.y, -omega.x * omega.z])
    term_6 = np.array([
        -omega.x * omega.y, -omega.x**2 - omega.y**2, alpha.x - omega.y * omega.z,
        omega.x * omega.y, alpha.y + omega.x * omega.z, alpha.z])

    row_1 = np.concatenate((term_1, np.zeros(6)))
    row_2 = np.concatenate((term_2, np.zeros(6)))
    row_3 = np.concatenate((term_3, np.zeros(6)))
    row_4 = np.concatenate((np.array([0, 0, a.z - g.z, g.y - a.y]), term_4))
    row_5 = np.concatenate((np.array([0, g.z - a.z, 0, a.x - g.x]), term_5))
    row_6 = np.concatenate((np.array([0, a.y - g.y, g.x - a.x, 0]), term_6))

    return np.array([row_1, row_2, row_3, row_4, row_5, row_6])


def rotate_gravity(g, orientation):
    """
    Computes the gravity vector on the same frame (inverted) of the given orientation.

    :param g: Gravity vector expressed in the same frame as the pose.
    :type g: geometry_msgs.msg.Vector3

    :param orientation: Orientation where the gravity vector is referenced from
        (in quaternion form).
    :type orientation: geometry_msgs.msg.Quaternion

    :return: The gravity vector expressed on the desired frame.
    :rtype: geometry_msgs.msg.Vector3

    """
    rot_matrix = transformations.quaternion_matrix([
        orientation.w, orientation.x, orientation.y, orientation.z])[0:3, 0:3]

    g_rotated = np.dot(rot_matrix.T, np.array([g.x, g.y, g.z]))
    return geometry_msgs.msg.Vector3(g_rotated[0], g_rotated[1], g_rotated[2])


def transform_twist(twist, transform):
    """"
    Transform a twist based on a given transformation matrix.

    :param twist: The twist to which the transform should be applied.
    :type twist: geometry_msgs.msg.Twist

    :param transform: The desired transform that should be applied.
    :type transform: numpy.matrix[4][4]

    :return: The transformed twist.
    :rtype: geometry_msgs.msg.Twist

    """
    linear_velocity = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
    angular_velocity = np.array([twist.angular.x, twist.angular.y, twist.angular.z])

    M = transform[0:3, 0:3]
    p = transform[0:3, 3]

    transformed_angular_velocity = np.dot(M, angular_velocity)
    transformed_linear_velocity = \
        np.dot(M, linear_velocity) + np.cross(p, transformed_angular_velocity)

    twist_out = geometry_msgs.msg.Twist()
    twist_out.linear.x = transformed_linear_velocity[0]
    twist_out.linear.y = transformed_linear_velocity[1]
    twist_out.linear.z = transformed_linear_velocity[2]
    twist_out.angular.x = transformed_angular_velocity[0]
    twist_out.angular.y = transformed_angular_velocity[1]
    twist_out.angular.z = transformed_angular_velocity[2]

    return twist_out
