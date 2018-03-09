#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This module contains a collection of functions used by the tactile sensor models.

"""

import numpy as np
import collections


class Electrode(object):
    def __init__(self, label, position=(0.0, 0.0, 0.0), normal=(0.0, 0.0, 0.0),
                 intensity=0.0):
        assert isinstance(label, str)
        assert isinstance(position, collections.Iterable) and len(position) == 3, \
            "'position' must be a 3-element tuple, not *{}*.".format(position)
        assert isinstance(normal, collections.Iterable) and len(normal) == 3, \
            "'normal' must be a 3-element tuple, not *{}*.".format(normal)
        assert isinstance(intensity, float)
        assert 0.0 <= intensity <= 1.0, \
            "'intensity' value ({}) must be within the 0-1 range.".format(intensity)
        self.label = label
        self.position = position
        self.normal = normal
        self.intensity = intensity

    def set_intensity(self, value, min_=0.0, max_=1.0):
        """
        Clip the value to be between 0 and 1 and then assigns it to the intensity attribute.

        :param value: Intensity value of the electrode.
        :type value: float

        :param min_: Maximum value allowed for the intensity.
        :type min_: float

        :param max_: Maximum value allowed for the intensity.
        :type max_: float

        """
        self.intensity = np.clip(value, min_, max_)

    def __str__(self):
        return "Label: {}, intensity: {},\nposition: {}, normal: {}".format(
            self.label, self.intensity, self.position, self.normal)


def project_on_sphere(point, center, radius):
    """
    Based on:
    http://stackoverflow.com/a/9604279/4788274

    :param point: A 3D point.
    :param center: The coordinates of the sphere's center.
    :param radius: The radius of the sphere.
    :return: The point projected on the sphere's surface

    """
    p = point - center
    q = (p / np.linalg.norm(p)) * radius

    return q + center


def project_on_finger(point, radius=0.007, center=(0.0, 0.0, 0.0)):
    """
    Takes a point inside the BioTac finger and projects it onto its surface.
    We assume the geometry of the BioTac as described in equation (2) of [1].

    [1] Loeb, Gerald E. "Estimating Point of Contact, Force and Torque in a
    Biomimetic Tactile Sensor with Deformable Skin." (2013).

    :param point: 3D position representing a point inside the finger.
    :type point: numpy.ndarray

    :param radius: Radius of the sphere used to project the point.
    :type radius: float

    :param center: A 3D position representing a point inside the finger.
    :type center: list

    :return: The projected point onto the finger's surface.
    :rtype:list

    """
    # Project to the spherical part
    if point[0] >= 0:
        projected_point = project_on_sphere(point, list(center), radius)
    # Project to the cylinder part
    else:
        # Set a sphere located at 0 in Y and Z, but use X of the point as the center.
        projected_point = project_on_sphere(point, [point[0], center[1], center[2]], radius)
    return projected_point


def perpendicular_vector(v):
    """
    Computes a perpendicular vector for 'v'.
    Obtained from:

    http://codereview.stackexchange.com/a/43940

    :param v: 3D position of the circle's center.
    :type v: list

    """
    if v[1] == 0 and v[2] == 0:
        if v[0] == 0:
            raise ValueError('zero vector')
        else:
            return np.cross(v, [0.0, 1.0, 0.0])
    return np.cross(v, [1.0, 0.0, 0.0])


def circle_gaussian(
        point, normal, intensity=0.5, min_value=0.1, max_value=1, scale=1000,
        points=20, inverted=True):
    """
    Computes a set of points that lie on a circle in 3D space with a Gaussian distribution.
    The point represents the circle's center and the normal the plane where it lies on.

    Based on:
    http://math.stackexchange.com/a/73242

    :param point: 3D position of the circle's center.
    :type point: tuple

    :param normal: Normal of the circle.
    :type normal: tuple

    :param intensity: The intensity represents the value on the diagonal of the covariance
        matrix used for the Gaussian distribution.
    :type intensity: float

    :param min_value: Minimum value to use for the diagonal in the covariance matrix.
    :type min_value: float

    :param max_value: Maximum value to use for the diagonal in the covariance matrix.
    :type max_value: float

    :param scale: Scaling factor to adjust the output of the distribution.
    :type scale: float

    :param points: Number of points to create the perimeter of the circle.
    :type points: int

    :param inverted: If true, the higher the intensity the narrower the Gaussian
    distribution will be. If false, then the intensity is proportional to the
    width of the distribution.
    :type inverted: bool

    :return: The points representing the sphere's surface.
    :rtype: list

    """
    assert np.any(normal), "'normal' can't be the zero vector."
    p = np.array(point, dtype=np.float32)
    n = np.array(normal, dtype=np.float32)
    n /= np.linalg.norm(n)

    # Compute the perpendicular vectors to the normal that lie on the plane.
    u = np.array(perpendicular_vector(n)).reshape((1, 3))
    v = np.array(np.cross(u, n)).reshape((1, 3))

    if inverted:
        intensity = 1.0 - intensity
    # Scale the intensity value to be within the range specified by the min/max values.
    c = min_value + (max_value - min_value) * intensity

    # Make sure the value of the diagonal in the covariance matrix is within the limits.
    c = min([max_value, max([min_value, c])])

    g = np.random.multivariate_normal([0.0, 0.0], [[c, 0.0], [0.0, c]], points).T / scale
    g_x = np.array(g[0]).reshape((points, 1))
    g_y = np.array(g[1]).reshape((points, 1))

    p_out = p + np.dot(g_x, u) + np.dot(g_y, v)
    p_out = np.array(p_out).T
    return [p_out[0], p_out[1], p_out[2]]


def locate_contact(electrodes, gaussian=True, inverted=False):
    """
    Computes the contact location based on the intensity values of the electrodes, see
    steps below for clarification.
    It assumes only active electrodes are given as input.

    1. Compute Gaussian distributions on the electrodes' planes. These distributions' widths
    are proportional to the intensity if the 'inverted' is set to False, otherwise the
    distributions' widths are inversely proportional to the intensity values. A single point
    from each distribution is then extracted, representing the center of the electrode.

    2. Use the point from 1. to compute the centroid of the electrodes.

    3. Compute the contact location by using vectors from the centroid to each electrode
    center, with their magnitude proportional to their intensity (after being normalized),
    and average them.

    4. Projects the contact point onto the finger's surface.

    :param electrodes: Active electrodes with their pose and intensity information.
    :type electrodes: Electrode

    :param gaussian: If True, use a Gaussian distribution to locate the center of the
        electrodes. If False, then simply use the electrodes positions.
    :type gaussian: bool

    :param inverted: If True, then the Gaussian distribution width is inversely proportional
        to the intensity value.
    :type inverted: bool

    :return: The contact location in the 3D space.
    :rtype: numpy.ndarray

    """
    if not gaussian:
        centroids = [ee.position for ee in electrodes]
    else:
        centroids = [
            circle_gaussian(
                ee.position, ee.normal, intensity=ee.intensity, points=1, inverted=inverted)
            for ee in electrodes]

    centroids = np.array(centroids).reshape((len(centroids), 3))
    centroid = centroids.mean(axis=0)

    # Use the sum of all intensities to normalize the distances vectors
    sum_intensities = float(sum((ee.intensity for ee in electrodes)))
    direction_vectors = [vector - centroid for vector in centroids]
    direction_vectors = [
        (ee.intensity / sum_intensities) * dd for ee, dd in zip(electrodes, direction_vectors)]
    direction_vectors = np.array(direction_vectors)

    contact_location = (direction_vectors.sum(axis=0) / len(centroids)) + centroid

    return project_on_finger(contact_location)
