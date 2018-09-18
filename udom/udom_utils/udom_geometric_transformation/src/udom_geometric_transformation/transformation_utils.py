#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This modules defines a class and functions to perform transformations such as
wrench transforms.


"""

import numpy as np
import rospy
import tf.transformations
import tf2_ros
import geometry_msgs.msg
# Needed for tf2
from tf2_geometry_msgs import PointStamped


class GeometryTransformer:
    """
    Subclass of TransformListener that provides transformation methods.

    """
    def __init__(self):
        """
        Initializes a TransformListener object.

        :return: A transform listener object.
        :rtype: GeometryTransformer

        """
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

    def transform_wrench(self, wrench, wrench_header, target_frame, point=None, duration=5.0):
        """
        Transform the wrench from the frame specified in its header to the target_frame
        at the time specified by the header.
        Throws an exception if the transformation failed.

        :param wrench: The wrench to be transformed.
        :type wrench: geometry_msgs.msg.Wrench

        :param wrench_header: The header of the wrench to be transformed.
        :type wrench_header: std_msgs.msg.Header

        :param target_frame: The frame into which the wrench should be transformed.
        :type target_frame: str

        :param point: The position where the wrench is applied with respect to the wrench's
            reference frame.
        :type point: tuple

        :param duration: Maximum duration to wait for the transform (in seconds).
        :type duration: float

        :return: The transformed wrench.
        :rtype: geometry_msgs.msg.Wrench

        """
        transform = self.buffer.lookup_transform(
            target_frame, wrench_header.frame_id, wrench_header.stamp,
            timeout=rospy.Duration(duration))

        transform_matrix = np.zeros((4, 4), dtype=float)
        transform_matrix[3, 3] = 1.0
        transform_matrix[0:3, 3] = [
            transform.transform.translation.x, transform.transform.translation.y,
            transform.transform.translation.z]
        # get matrix from quaternion (transformations.quaternion_matrix
        transform_matrix[0:3, 0:3] = tf.transformations.quaternion_matrix([
            transform.transform.rotation.x, transform.transform.rotation.y,
            transform.transform.rotation.z, transform.transform.rotation.w])[0:3, 0:3]

        wrench_out = transform_wrench(wrench, transform_matrix, point)

        return wrench_out

    def transform_point(self, point, point_header, target_frame, duration=5.0):
        """
        Transform the point from the frame specified in its header to the target_frame
        at the time specified by the header.
        Throws an exception if the transformation failed.

        :param point: The point to be transformed.
        :type point: geometry_msgs.msg.Point

        :param point_header: The header of the point to be transformed.
        :type point_header: std_msgs.msg.Header

        :param target_frame: The frame into which the point should be transformed.
        :type target_frame: str

        :param duration: Maximum duration to wait for the transform (in seconds).
        :type duration: float

        :return: The transformed point.
        :rtype: geometry_msgs.msg.Point

        """
        source_point = geometry_msgs.msg.PointStamped()
        source_point.header = point_header
        source_point.point = point

        point_out = self.buffer.transform(
            source_point, target_frame, timeout=rospy.Duration(duration))

        return point_out.point

    def transform_pose(self, pose, target_frame, duration=5.0):
        """
        Transform the pose to the target_frame.
        Throws an exception if the transformation failed.

        :param pose: The pose to be transformed.
        :type pose: geometry_msgs.msg.PoseStamped

        :param target_frame: The frame into which the pose should be transformed.
        :type target_frame: str

        :param duration: Maximum duration to wait for the transform (in seconds).
        :type duration: float

        :return: The transformed Pose.
        :rtype: geometry_msgs.msg.PoseStamped

        """
        point_out = self.buffer.transform(
            pose, target_frame, timeout=rospy.Duration(duration))

        return point_out

    def transform_twist(self, twist_in, target_frame='base_link', wait_for_transform=0.1):
        """
        Transforms the provided twist to the target frame.
        Raises an exception if the transformation failed.

        :param twist_in: The twist which should be transformed.
        :type twist_in: geometry_msgs.msg.TwistStamped

        :param target_frame: The frame into which the twist should be transformed.
        :type target_frame: str

        :param wait_for_transform: Amount of seconds to wait for the transform.
        :type wait_for_transform: float

        :return: The transformed twist.
        :rtype: geometry_msgs.msg.TwistStamped

        """
        twist_out = geometry_msgs.msg.TwistStamped()
        transform = self.buffer.lookup_transform(
            target_frame, twist_in.header.frame_id, twist_in.header.stamp,
            timeout=rospy.Duration(wait_for_transform))

        transform_matrix = np.zeros((4, 4), dtype=float)
        transform_matrix[3, 3] = 1.0
        transform_matrix[0:3, 3] = [
            transform.transform.translation.x, transform.transform.translation.y,
            transform.transform.translation.z]
        # get matrix from quaternion (transformations.quaternion_matrix
        transform_matrix[0:3, 0:3] = tf.transformations.quaternion_matrix([
            transform.transform.rotation.x, transform.transform.rotation.y,
            transform.transform.rotation.z, transform.transform.rotation.w])[0:3, 0:3]

        transformed_twist = transform_twist(twist_in.twist, transform_matrix)

        twist_out.twist = transformed_twist
        twist_out.header.frame_id = target_frame
        twist_out.header.stamp = twist_in.header.stamp

        return twist_out


def transform_wrench(wrench_in, transform, point=None):
    """
    Applies a transform to a wrench. It is assumed that the reference point and
    reference frame are collapsed into a single coordinate frame. (See also
    http://www.ros.org/wiki/tf/Reviews/2010-03-12_API_Review).

    :param wrench_in: The wrench to which the transform should be applied.
    :type wrench_in: geometry_msgs.msg.Wrench

    :param transform: A 4x4 transformation matrix that should be applied to the wrench.
    :type transform: numpy.array

    :param point: The position where the wrench is applied with respect to the wrench's
        reference frame.
    :type point: tuple

    :return: The transformed wrench.
    :rtype: geometry_msgs.msg.Wrench

    """
    wrench_out = geometry_msgs.msg.Wrench()

    rotation_matrix = transform[0:3, 0:3]
    translation = transform[0:3, 3]

    if point is not None:
        assert isinstance(point, tuple) and (len(point) == 3), \
            "'point' must be a three element tuple, not '{}'".format(point)
        translation += np.dot(rotation_matrix, np.array(point, dtype=np.float32))

    force_in = np.array([wrench_in.force.x, wrench_in.force.y, wrench_in.force.z])
    torque_in = np.array([wrench_in.torque.x, wrench_in.torque.y, wrench_in.torque.z])

    force_out = np.dot(rotation_matrix, force_in)
    torque_out = np.dot(rotation_matrix, torque_in) + np.cross(translation, force_out)

    wrench_out.force.x, wrench_out.force.y, wrench_out.force.z = force_out
    wrench_out.torque.x, wrench_out.torque.y, wrench_out.torque.z = torque_out

    return wrench_out


def quaternion_rotation(vector, quat):
    """
    Rotates a vector by a quaternion.
    Note: Quaternions w+ix+jy+kz are represented as [w, x, y, z].

    :param vector: Vector to rotate.
    :type vector: list

    :param quat: Quaternion.
    :type quat: geometry_msgs.msg.Quaternion

    :return: Rotated vector.
    :rtype: numpy.array

    """
    quaternion = np.array([
        [(1 - 2*quat.y**2 - 2*quat.z**2), 2*(quat.x*quat.y + quat.w*quat.z),
         2*(quat.x*quat.z - quat.w*quat.y)],
        [2*(quat.x*quat.y - quat.w*quat.z), (1 - 2*quat.x**2 - 2*quat.z**2),
         2*(quat.y*quat.z + quat.w*quat.x)],
        [2*(quat.x*quat.z + quat.w*quat.y), 2*(quat.y*quat.z - quat.w*quat.x),
         (1 - 2*quat.x**2 - 2*quat.y**2)]])

    return np.dot(quaternion, np.array(vector))


def transform_twist(twist, transform):
    """"
    Apply a transform to a twist. It is assumed that the reference point and
    reference frame are collapsed into a single coordinate frame. (See also
    http://www.ros.org/wiki/tf/Reviews/2010-03-12_API_Review)

    :param twist: The twist to which the transform should be applied.
    :type twist: geometry_msgs.msg.Twist

    :param transform: The desired transform that should be applied.
    :type transform: numpy.matrix[4][4]

    :return: The transformed twist.
    :rtype: geometry_msgs.msg.Twist

    """
    twist_out = geometry_msgs.msg.Twist()

    linear_velocity = np.array([
        twist.linear.x, twist.linear.y, twist.linear.z])
    angular_velocity = np.array([
        twist.angular.x, twist.angular.y, twist.angular.z])

    M = transform[0:3, 0:3]
    p = transform[0:3, 3]

    transformed_angular_velocity = np.dot(M, angular_velocity)
    transformed_linear_velocity = \
        np.dot(M, linear_velocity) + np.cross(p, transformed_angular_velocity)

    twist_out.linear.x = transformed_linear_velocity[0]
    twist_out.linear.y = transformed_linear_velocity[1]
    twist_out.linear.z = transformed_linear_velocity[2]
    twist_out.angular.x = transformed_angular_velocity[0]
    twist_out.angular.y = transformed_angular_velocity[1]
    twist_out.angular.z = transformed_angular_velocity[2]

    return twist_out
