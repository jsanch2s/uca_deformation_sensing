#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
Given two poses it returns their positional difference as a geometry_msgs/PointStamped object.
"""

import copy
import rospy
import udom_geometric_transformation.transformation_utils as transformation_utils
import geometry_msgs.msg


class PoseErrorCalculatorNode(object):
    """
    Subscribes to two poses and publishes their difference as a
    geometry_msgs/PointStamped message.

    """
    def __init__(self):
        """
        Instantiates a pose error calculator node.

        :return: Node to calculate the difference between two poses.
        :rtype: PoseErrorCalculator

        """
        # Params
        self.pose_1 = None
        self.pose_2 = None

        # Object to compute transformations.
        self.listener = transformation_utils.GeometryTransformer()

        # Maximum duration to wait for a transform (in seconds).
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100))

        # Publishers
        self.positional_error = rospy.Publisher(
            "~positional_error", geometry_msgs.msg.PointStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber("~pose_1", geometry_msgs.msg.PoseStamped, self.pose_1_cb)
        rospy.Subscriber('~pose_2', geometry_msgs.msg.PoseStamped, self.pose_2_cb)

    def pose_1_cb(self, msg):
        """
        Obtains the first pose.

        :param msg: First pose.
        :type msg: geometry_msgs.msg.PoseStamped

        """
        self.pose_1 = msg

    def pose_2_cb(self, msg):
        """
        Obtains the second pose.

        :param msg: Second pose.
        :type msg: geometry_msgs.msg.PoseStamped

        """
        self.pose_2 = msg

    def start(self):
        """
        Starts the node.

        """
        rospy.loginfo("Ready to start...")
        state = 'IDLE'

        while not rospy.is_shutdown():

            if state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.pose_1 is not None and self.pose_2 is not None:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """

        pose = self.pose_difference()
        if pose:
            self.positional_error.publish(pose)
        else:
            rospy.logwarn("Error while extracting the pose.")
        self.reset_component_data()
        return 'IDLE'

    def pose_difference(self):
        """
        Returns the positional error between two poses.

        :return: The positional error between the two poses.
        :rtype: geometry_msgs.msg.PointStamped

        """
        if self.pose_1.header.frame_id != self.pose_2.header.frame_id:
            pose_1 = self.listener.transform_pose(
                self.pose_1, self.pose_2.header.frame_id, self.wait_for_transform)
        else:
            pose_1 = copy.deepcopy(self.pose_1)
        error = calculate_positional_error(self.pose_2, pose_1)

        return error

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.pose_1 = None
        self.pose_2 = None


def calculate_positional_error(current_pose, target_pose):
    """
    Calculates the component-wise error between two 'PoseStamped' objects.
    It assumes that both poses are specified with respect of the same
    reference frame.

    :param current_pose: The current pose.
    :type current_pose: geometry_msgs.msg.PoseStamped

    :param target_pose: The target pose.
    :type target_pose: geometry_msgs.msg.PoseStamped

    :return: The positional error.
    :rtype: geometry_msgs.msg.PointStamped

    """
    error = geometry_msgs.msg.PointStamped()

    # calculate linear distances
    error.point.x = target_pose.pose.position.x - current_pose.pose.position.x
    error.point.y = target_pose.pose.position.y - current_pose.pose.position.y
    error.point.z = target_pose.pose.position.z - current_pose.pose.position.z

    error.header.frame_id = current_pose.header.frame_id
    error.header.stamp = rospy.Time.now()

    return error


def main():
    rospy.init_node("pose_error_calculator", anonymous=True)
    pose_error_calculator = PoseErrorCalculatorNode()
    pose_error_calculator.start()
