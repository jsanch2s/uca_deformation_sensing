#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This modules defines functions to compute the nodal forces on a mesh.


"""

import numpy as np
from scipy.spatial import cKDTree


def point_to_array(point):
    """
    Converts a geometry_msgs.msg.Point message into a numpy.array.

    :param point: Position of a point in free space.
    :type point: geometry_msgs.msg.Point

    :return: A three-dimensional array having the X, Y and Z coordinates of the point.
    :rtype: numpy.array

    """
    x, y, z = (point.x, point.y, point.z)
    return np.array([x, y, z], dtype=np.float32)


def neighbors(point, mesh, n=3, distance=None):
    """
    Returns the 'n' neighbors, along with their indices, from the mesh's nodes to the
    given point. The neighbors are returned as geometry_msgs.msg.Point objects.

    :param point: Position of a point in 3D space.
    :type point: geometry_msgs.msg.Point

    :param mesh: Mesh.
    :type mesh: shape_msgs.msg.Mesh

    :param n: Number of neighbors to search for.
    :type n: int

    :param distance: Maximum distance the closest neighbor should have (in meters). If the neighbor
        is farther away, then the function returns None, None.
    :type distance: float

    :return: The nodes in the mesh that are closest to the given point.
        The first list are the points representing the neighboring nodes and the second list
        are their indices as specified in the mesh.
    :rtype: list, list, bool

    """
    points = [point_to_array(pp) for pp in mesh.vertices]
    points = np.array(points).reshape((len(mesh.vertices), 3))
    p = np.array(point_to_array(point)).reshape(1, -1)

    nearest_neighbors = cKDTree(points)
    distances, indices = nearest_neighbors.query(p, k=n)

    in_collision = True
    if distance is not None:
        if abs(distance) < min(distances[0]):
            in_collision = False
    indices = indices[0]

    return np.take(mesh.vertices, indices), indices, in_collision


def triangle_area(p_1, p_2, p_3):
    """
    Computes the area formed by the three points, based on this answer [1].
    [1] http://math.stackexchange.com/a/128995/387996

    :param p_1: First point of the triangle.
    :type p_1: geometry_msgs.msg.Point

    :param p_2: Second point of the triangle.
    :type p_2: geometry_msgs.msg.Point

    :param p_3: Third point of the triangle.
    :type p_3: geometry_msgs.msg.Point

    :return: Area of the triangle.
    :rtype: float

    """
    vector_1 = np.array([p_2.x - p_1.x, p_2.y - p_1.y, p_2.z - p_1.z])
    vector_2 = np.array([p_3.x - p_1.x, p_3.y - p_1.y, p_3.z - p_1.z])

    return np.linalg.norm(np.cross(vector_1, vector_2)) / 2.0


def compute_nodal_forces(point, force, triangle):
    """
    Distributes the force applied at a point, within the triangle, among the triangle's nodes.
    The distribution of the force on each node is inversely proportional to the
    distance of the node to the point.

    :param point: Position where the force is applied.
    :type point: geometry_msgs.msg.Point

    :param force: Force applied inside the triangle.
    :type force: geometry_msgs.msg.Vector3

    :param triangle: Second point of the triangle.
    :type triangle: geometry_msgs.msg.Polygon

    :return: A 9-element list representing the force in X, Y and Z on each node:
                [F1_x, F1_y, F1_z, F2_x, F2_y, F2_z, F3_x, F3_y, F3_z]
        where F1, F2 and F3 are the forces at node 1, 2 and 3, respectively.
    :rtype: list

    """
    # Compute triangular areas.
    area_1 = triangle_area(point, triangle.points[1], triangle.points[2])
    area_2 = triangle_area(point, triangle.points[0], triangle.points[2])
    area_3 = triangle_area(point, triangle.points[0], triangle.points[1])
    area = area_1 + area_2 + area_3

    a_1 = area_1 / float(area)
    a_2 = area_2 / float(area)
    a_3 = area_3 / float(area)

    # Build shape function.
    shape_function = np.array([
        [a_1,   0,   0, a_2,   0,   0, a_3,   0,   0],
        [0,   a_1,   0,   0, a_2,   0,   0, a_3,   0],
        [0,     0, a_1,   0,   0, a_2,   0,   0, a_3],
    ], dtype=np.float32)

    # Compute nodal forces.
    return np.dot(shape_function.T, np.array([force.x, force.y, force.z]))
