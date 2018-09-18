#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This modules contains functions to perform operations on quaternions and dual quaternions.


"""

import numpy as np
import geometry_msgs.msg
import udom_control_msgs.msg

# Near zero constant
NEAR_ZERO = 1e-6


def multiply(q_lhs, q_rhs):
    """
    Quaternion multiplication (non commutative). Note that order matters.

    :param q_lhs: Quaternion to multiply (left hand side).
    :type q_lhs: geometry_msgs.msg.Quaternion

    :param q_rhs: Quaternion to multiply (right hand side).
    :type q_rhs: geometry_msgs.msg.Quaternion

    :return: The product of the two quaternions.
    :rtype: geometry_msgs.msg.Quaternion

    """
    s_1 = q_lhs.w
    s_2 = q_rhs.w
    v_1 = np.array([q_lhs.x, q_lhs.y, q_lhs.z])
    v_2 = np.array([q_rhs.x, q_rhs.y, q_rhs.z])

    q_out = geometry_msgs.msg.Quaternion()
    s_out = s_1 * s_2 - np.dot(v_1, v_2)
    v_out = s_1 * v_2 + s_2 * v_1 + np.cross(v_1, v_2)

    q_out.w = s_out
    q_out.x = v_out[0]
    q_out.y = v_out[1]
    q_out.z = v_out[2]

    return q_out


def add(q_lhs, q_rhs):
    """
    Quaternion addition.

    :param q_lhs: Quaternion to add (left hand side).
    :type q_lhs: geometry_msgs.msg.Quaternion

    :param q_rhs: Quaternion to add (right hand side).
    :type q_rhs: geometry_msgs.msg.Quaternion

    :return: The addition of the two quaternions.
    :rtype: geometry_msgs.msg.Quaternion

    """
    q_out = geometry_msgs.msg.Quaternion()
    q_out.x = q_lhs.x + q_rhs.x
    q_out.y = q_lhs.y + q_rhs.y
    q_out.z = q_lhs.z + q_rhs.z
    q_out.w = q_lhs.w + q_rhs.w

    return q_out


def conjugate(q):
    """
    Conjugate of a quaternion.

    :param q: Quaternion to conjugate.
    :type q: geometry_msgs.msg.Quaternion

    :return: The conjugate of the quaternion.
    :rtype: geometry_msgs.msg.Quaternion

    """
    q_out = geometry_msgs.msg.Quaternion()
    q_out.w = q.w
    q_out.x = -q.x
    q_out.y = -q.y
    q_out.z = -q.z

    return q_out


def normalize(q):
    """
    Normalization of a quaternion.

    :param q: Quaternion to normalize.
    :type q: geometry_msgs.msg.Quaternion

    :return: The normalized quaternion.
    :rtype: geometry_msgs.msg.Quaternion

    """
    q_out = geometry_msgs.msg.Quaternion()
    norm = np.linalg.norm([q.x, q.y, q.z, q.w])
    q_out.w = q.w / norm
    q_out.x = q.x / norm
    q_out.y = q.y / norm
    q_out.z = q.z / norm

    return q_out


def scale(q, s):
    """
    Multiplies a quaternion by a scalar.

    :param q: Quaternion to scale.
    :type q: geometry_msgs.msg.Quaternion

    :param s: Scale to multiply the quaternion.
    :type s: Float

    :return: The scaled quaternion.
    :rtype: geometry_msgs.msg.Quaternion

    """
    q_out = geometry_msgs.msg.Quaternion()
    q_out.w = q.w * s
    q_out.x = q.x * s
    q_out.y = q.y * s
    q_out.z = q.z * s

    return q_out


def add_dq(dq_lhs, dq_rhs):
    """
    Dual quaternion addition.

    :param dq_lhs: Dual quaternion to add (left hand side).
    :type dq_lhs: udom_control_msgs.msg.DualQuaternion

    :param dq_rhs: Dual quaternion to add (right hand side).
    :type dq_rhs: udom_control_msgs.msg.DualQuaternion

    :return: The addition of both dual quaternions.
    :rtype: udom_control_msgs.msg.DualQuaternion

    """
    dq_out = udom_control_msgs.msg.DualQuaternion()

    dq_out.real = add(dq_lhs.real, dq_rhs.real)
    dq_out.dual = add(dq_lhs.dual, dq_rhs.dual)

    return dq_out


def multiply_dq(dq_lhs, dq_rhs):
    """
    Dual quaternion multiplication (non commutative). Note that order matters.

    :param dq_lhs: Dual quaternion to multiply (left hand side).
    :type dq_lhs: udom_control_msgs.msg.DualQuaternion

    :param dq_rhs: Dual quaternion to multiply (right hand side).
    :type dq_rhs: udom_control_msgs.msg.DualQuaternion

    :return: The product of both dual quaternions.
    :rtype: udom_control_msgs.msg.DualQuaternion

    """
    dq_out = udom_control_msgs.msg.DualQuaternion()

    dq_out.real = multiply(dq_rhs.real, dq_lhs.real)
    term_1 = multiply(dq_rhs.dual, dq_lhs.real)
    term_2 = multiply(dq_rhs.real, dq_lhs.dual)
    dq_out.dual = add(term_1, term_2)

    return dq_out


def conjugate_dq(dq):
    """
    Dual quaternion conjugation.

    :param dq: Dual quaternion to conjugate.
    :type dq: udom_control_msgs.msg.DualQuaternion

    :return: The conjugation of the dual quaternion.
    :rtype: udom_control_msgs.msg.DualQuaternion

    """
    dq_out = udom_control_msgs.msg.DualQuaternion()

    dq_out.real = conjugate(dq.real)
    dq_out.dual = conjugate(dq.dual)

    return dq_out


def normalize_dq(dq):
    """
    Normalization of dual quaternion (to make it unit).

    :param dq:
    :type dq: udom_control_msgs.msg.DualQuaternion

    :return:
    :rtype: udom_control_msgs.msg.DualQuaternion

    """
    dq_out = udom_control_msgs.msg.DualQuaternion()

    real = np.array([dq.real.w, dq.real.x, dq.real.y, dq.real.z], dtype=np.float32)
    magnitude = np.linalg.norm(real)
    assert magnitude > NEAR_ZERO, "Magnitude of 'dq.real' is close to zero."

    dq_out.real = scale(dq.real, 1.0 / magnitude)
    dq_out.dual = scale(dq.dual, 1.0 / magnitude)

    return dq_out


def scale_dq(dq, s):
    """
    Multiplies a dual quaternion by a scalar.

    :param dq: Dual quaternion to scale.
    :type dq: udom_control_msgs.msg.DualQuaternion

    :param s: Scale to multiply the dual quaternion.
    :type s: Float

    :return: The scaled dual quaternion.
    :rtype: udom_control_msgs.msg.DualQuaternion

    """
    dq_out = udom_control_msgs.msg.DualQuaternion()

    dq_out.real = scale(dq.real, s)
    dq_out.dual = scale(dq.dual, s)

    return dq_out


def pose_to_dq(pose):
    """
    Converts a pose into a unit dual quaternion. It is assumed that the pose
    is expressed in the same frame as the dual quaternion.

    :param pose: Pose to convert to dual quaternion.
    :type pose: geometry_msgs.msg.Pose

    :return: Converted dual quaternion.
    :rtype: udom_control_msgs.msg.DualQuaternion

    """
    dq_out = udom_control_msgs.msg.DualQuaternion()
    dq_out.real = normalize(pose.orientation)

    translation_quaternion = geometry_msgs.msg.Quaternion()
    translation_quaternion.w = 0.0
    translation_quaternion.x = pose.position.x
    translation_quaternion.y = pose.position.y
    translation_quaternion.z = pose.position.z

    term_1 = multiply(translation_quaternion, dq_out.real)
    dq_out.dual = scale(term_1, 0.5)
    return dq_out


def dq_to_pose(dq):
    """
    Converts a unit dual quaternion into a pose. It is assumed that the dual quaternion
    is expressed in the same frame as the pose.

    :param dq: Dual quaternion to convert to pose.
    :type dq: udom_control_msgs.msg.DualQuaternion

    :return: Converted pose.
    :rtype: geometry_msgs.msg.Pose

    """
    pose = geometry_msgs.msg.Pose()
    term_1 = scale(dq.dual, 2.0)
    term_2 = conjugate(dq.real)
    translation_quaternion = multiply(term_1, term_2)
    pose.position.x = translation_quaternion.x
    pose.position.y = translation_quaternion.y
    pose.position.z = translation_quaternion.z
    pose.orientation = dq.real

    return pose


def screw_params(dq):
    """
    Retrieves the screw parameters of a unit dual quaternion into

    :param dq: Dual quaternion.
    :type dq: udom_control_msgs.msg.DualQuaternion

    :return: The screw parameters
    :rtype: list

    """
    s_r = dq.real.w
    v_r = np.array([dq.real.x, dq.real.y, dq.real.z])

    s_t = dq.dual.w
    v_t = np.array([dq.dual.x, dq.dual.y, dq.dual.z])

    theta = 2 * np.arccos(s_r)
    assert 0.0 <= theta <= 2 * np.pi, "Real part of dual quaternion must be [-1, 1]" \
                                      " is dq.real.w = {}".format(dq.real.w)

    if 0.0 < theta < 2 * np.pi:
        theta_alt = np.sin(theta / 2.0)
        d = -2 * (s_t / theta_alt)
        l = v_r / theta_alt
        m = (v_t - s_r * (d / 2.0) * l) * (1.0 / theta_alt)
    else:
        d = 2 * np.linalg.norm(v_t)
        if abs(d) > NEAR_ZERO:
            l = 2 * v_t / d
        else:
            l = 2 * v_t
        m = np.array([0, 0, 0], dtype=np.float)

    return [theta, d, l, m]


def dq_to_twist(dq):
    """
    Converts a unit dual quaternion into a twist. It is assumed that the dual quaternion
    is expressed in the same frame as the twist.

    :param dq: Dual quaternion.
    :type dq: udom_control_msgs.msg.DualQuaternion

    :return: Twist.
    :rtype: geometry_msgs.msg.Twist

    """
    twist_out = geometry_msgs.msg.Twist()

    theta, d, l, m = screw_params(dq)
    twist = theta * np.array([m, l]).flatten() + d * np.array([l, np.zeros(3)]).flatten()

    twist_out.linear.x = twist[0]
    twist_out.linear.y = twist[1]
    twist_out.linear.z = twist[2]
    twist_out.angular.x = twist[3]
    twist_out.angular.y = twist[4]
    twist_out.angular.z = twist[5]

    return twist_out
