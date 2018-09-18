#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This modules defines utility functions for the pose extractor node.


"""

import copy
import numpy as np
import geometry_msgs.msg
import tf
import tf.transformations as transformations


def extract_points(mesh, indices, zero_based=True):
    """
    Extracts the nodes' positions based on the mesh and indices.

    :param mesh: The mesh from which the pose will be extracted.
    :type mesh: udom_modeling_msgs.msg.Mesh

    :param indices: Indices of the node to return.
    :type indices: list of int

    :param zero_based: If true, it assumes the indices are zero-based.
        Otherwise, it assumes the indices start from one.
    :type zero_based: bool

    :return: The extracted nodes' positions, if there is an error it returns None.
    :rtype: list of geometry_msgs.msg.Point or None

    """
    try:
        if zero_based:
            return [mesh.vertices[index] for index in indices]
        else:
            return [mesh.vertices[index - 1] for index in indices]
    except IndexError:
        return None


def compute_centroid(points):
    """
    Computes the centroid of the points based on the arithmetic mean.

    :param points: The points on which the centroid will be computed.
    :type points: list of geometry_msgs.msg.Point

    :return: The centroid of the points.
    :rtype: geometry_msgs.msg.Point

    """
    centroid = geometry_msgs.msg.Point()
    points = [[point.x, point.y, point.z] for point in points]

    c = np.mean(points, axis=0)
    centroid.x = c[0]
    centroid.y = c[1]
    centroid.z = c[2]

    return centroid


def compute_normal_as_quaternion(points, rotation=90.0):
    """
    Computes the normal, as a quaternion, based on three points.

    :param points: The points on which the normal will be computed.
    :type points: list of geometry_msgs.msg.Point

    :param rotation: Rotates the pose around the normal axis by the specified angle (in degrees).
    :type rotation: float

    :return: The normal of the three points as a quaternion.
    :rtype: geometry_msgs.msg.Quaternion

    """
    quaternion = geometry_msgs.msg.Quaternion()

    p_1 = np.array([points[0].x, points[0].y, points[0].z])
    p_2 = np.array([points[1].x, points[1].y, points[1].z])
    p_3 = np.array([points[2].x, points[2].y, points[2].z])
    normal = np.cross((p_2 - p_1), (p_3 - p_1))

    quat = tf.transformations.quaternion_about_axis(np.radians(rotation), normal)
    quaternion.x = quat[0]
    quaternion.y = quat[1]
    quaternion.z = quat[2]
    quaternion.w = quat[3]

    return quaternion


def rotate_pose(pose, angle=0.0, reference_axis='z'):
    """
    Rotates the orientation of a pose, for a single rotation axis (reference_axis),
    by the specified angle.

    :param pose: The pose to be modified.
    :type pose: geometry_msgs.msg.PoseStamped

    :param angle: The angle to rotate the pose (in degrees).
    :type angle: float

    :param reference_axis: The rotation axis of the pose to be modified (e.g. x, y, z).
    :type reference_axis: str

    :return: The modified pose.
    :rtype: geometry_msgs.msg.PoseStamped

    """
    assert reference_axis.lower() in ['x', 'y', 'z'], \
        "'reference_axis' must be 'x', 'y', or 'z'."

    pose_out = copy.deepcopy(pose)
    orientation_in = (
        pose_out.pose.orientation.x, pose_out.pose.orientation.y,
        pose_out.pose.orientation.z, pose_out.pose.orientation.w)

    angles_out = np.array(transformations.euler_from_quaternion(orientation_in))

    # Ensure the offset is not more than 360 degrees
    angle %= 360

    if reference_axis.lower() == 'x':
        angles_out[0] += np.radians(angle)
    elif reference_axis.lower() == 'y':
        angles_out[1] += np.radians(angle)
    elif reference_axis.lower() == 'z':
        angles_out[2] += np.radians(angle)

    orientation_out = transformations.quaternion_from_euler(
        angles_out[0], angles_out[1], angles_out[2])

    pose_out.pose.orientation.x = orientation_out[0]
    pose_out.pose.orientation.y = orientation_out[1]
    pose_out.pose.orientation.z = orientation_out[2]
    pose_out.pose.orientation.w = orientation_out[3]

    return pose_out
