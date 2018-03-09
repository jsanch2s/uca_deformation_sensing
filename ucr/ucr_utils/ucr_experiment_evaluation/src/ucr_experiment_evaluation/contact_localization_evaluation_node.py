#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node evaluates the performance of the contact localization node.

**Input(s):**
  * `contact_info`: Contact information (e.g. with the contact location).
    - *type:* `udom_perception_msgs/ContactInfo`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `reference_frame`: Frame to be used as reference.
  * `wait_for_transform`: Maximum duration to wait for a transform (in seconds).

"""

import numpy as np
import rospy
import tf2_ros
import std_msgs.msg
import geometry_msgs.msg
import udom_perception_msgs.msg
# Needed to do the transformation.
from tf2_geometry_msgs import PointStamped


class ContactLocalizationEvaluationNode(object):
    """
    Subscribes to a udom_perception_msgs.msg.ContactInfo topic, transforms the contact
    location into the reference frame and prints the difference between them.

    """
    def __init__(self):
        """
        Instantiates a contact localization evaluation node.

        :return: Node to evaluate the contact localization node.
        :rtype: ContactLocalizationEvaluationNode

        """
        # Params
        self.event = None
        self.gaussian_info = None
        self.inverted_gaussian_info = None
        self.non_gaussian_info = None

        # Reference frame to compute the gravity vector.
        self.reference_frame = rospy.get_param("~reference_frame", None)
        assert self.reference_frame is not None, "A reference frame must be specified."

        # Maximum duration to wait for a transform (in seconds).
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # Object to compute transformations.
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            '~gaussian', udom_perception_msgs.msg.ContactInfo, self.gaussian_cb)
        rospy.Subscriber(
            '~inverted_gaussian', udom_perception_msgs.msg.ContactInfo,
            self.inverted_gaussian_cb)
        rospy.Subscriber(
            '~non_gaussian', udom_perception_msgs.msg.ContactInfo, self.non_gaussian_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def gaussian_cb(self, msg):
        """
        Obtains the contact information input from the Gaussian estimator.

        :param msg: Contact information.
        :type msg: udom_perception_msgs.msg.ContactInfo

        """
        self.gaussian_info = msg

    def inverted_gaussian_cb(self, msg):
        """
        Obtains the contact information input from the inverted Gaussian estimator.

        :param msg: Contact information.
        :type msg: udom_perception_msgs.msg.ContactInfo

        """
        self.inverted_gaussian_info = msg

    def non_gaussian_cb(self, msg):
        """
        Obtains the contact information input from the non-Gaussian estimator.

        :param msg: Contact information.
        :type msg: udom_perception_msgs.msg.ContactInfo

        """
        self.non_gaussian_info = msg

    def start(self):
        """
        Starts the node.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event_out.publish('e_stopped')
            self.reset_component_data()
            return 'INIT'
        elif self.gaussian_info is not None and self.inverted_gaussian_info\
                is not None and self.non_gaussian_info is not None:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event_out.publish('e_stopped')
            self.reset_component_data()
            return 'INIT'
        else:
            self.transform_points()
            self.reset_component_data()
            return 'IDLE'

    def transform_points(self):
        """
        Transforms the point in the contact information into the reference frame.

        :return: The nodal force information.
        :rtype: std_msgs.msg.Float32MultiArray

        """
        gaussian_point = geometry_msgs.msg.PointStamped()
        gaussian_point.header.frame_id = self.gaussian_info.header.frame_id
        gaussian_point.header.stamp = self.gaussian_info.header.stamp
        gaussian_point.point = self.gaussian_info.contact_points[0]

        inverted_gaussian_point = geometry_msgs.msg.PointStamped()
        inverted_gaussian_point.header.frame_id = self.inverted_gaussian_info.header.frame_id
        inverted_gaussian_point.header.stamp = self.inverted_gaussian_info.header.stamp
        inverted_gaussian_point.point = self.inverted_gaussian_info.contact_points[0]

        non_gaussian_point = geometry_msgs.msg.PointStamped()
        non_gaussian_point.header.frame_id = self.non_gaussian_info.header.frame_id
        non_gaussian_point.header.stamp = self.non_gaussian_info.header.stamp
        non_gaussian_point.point = self.non_gaussian_info.contact_points[0]

        try:
            gaussian_point_out = self.buffer.transform(
                gaussian_point, self.reference_frame,
                timeout=rospy.Duration(self.wait_for_transform))
            inverted_gaussian_point_out = self.buffer.transform(
                inverted_gaussian_point, self.reference_frame,
                timeout=rospy.Duration(self.wait_for_transform))
            non_gaussian_point_out = self.buffer.transform(
                non_gaussian_point, self.reference_frame,
                timeout=rospy.Duration(self.wait_for_transform))

            gaussian_error = np.linalg.norm([
                gaussian_point_out.point.x, gaussian_point_out.point.y,
                gaussian_point_out.point.z])
            inverted_gaussian_error = np.linalg.norm([
                inverted_gaussian_point_out.point.x, inverted_gaussian_point_out.point.y,
                inverted_gaussian_point_out.point.z])
            non_gaussian_error = np.linalg.norm([
                non_gaussian_point_out.point.x, non_gaussian_point_out.point.y,
                non_gaussian_point_out.point.z])

            rospy.loginfo(
                "\nGaussian: {:.4f} ({:.4f}, {:.4f}, {:.4f})\n"
                "Inverted Gaussian: {:.4f} ({:.4f}, {:.4f}, {:.4f})\nNon Gaussian: {:.4f} "
                "({:.4f}, {:.4f}, {:.4f})\n".format(
                    gaussian_error, gaussian_point_out.point.x, gaussian_point_out.point.y,
                    gaussian_point_out.point.z, inverted_gaussian_error,
                    inverted_gaussian_point_out.point.x, inverted_gaussian_point_out.point.y,
                    inverted_gaussian_point_out.point.z, non_gaussian_error,
                    non_gaussian_point_out.point.x, non_gaussian_point_out.point.y,
                    non_gaussian_point_out.point.z))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Error while transforming the points:\n{}".format(e))

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.gaussian_info = None
        self.inverted_gaussian_info = None
        self.non_gaussian_info = None
        self.event = None


def main():
    rospy.init_node("contact_localization_evaluation_node", anonymous=True)
    contact_localization_evaluation_node = ContactLocalizationEvaluationNode()
    contact_localization_evaluation_node.start()
