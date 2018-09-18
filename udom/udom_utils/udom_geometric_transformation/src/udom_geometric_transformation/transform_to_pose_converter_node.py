#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node converts a ROS-TF transform to a given frame and publishes as a ROS pose message.

**Input(s):**

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

    - *type:* `std_msgs/String`

**Output(s):**

  * `pose`: The transformed pose.
    - *type:* `geometry_msgs/PoseStamped`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `wait_for_transform`: Maximum duration to wait for a transform (in seconds).

  * `reference_frame`: Frame on which the `target_frame` will be expressed.

  * `target_frame`: This frame will be expressed with respect to the `reference_frame`.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf
import tf2_ros


class TransformToPoseConverter(object):
    """
    Converts a transform, between a target frame with respect to a specified frame,
    and publishes it as a pose.

    """
    def __init__(self):
        """
        Instantiates a transformer to pose converter node.

        :return: Node to transform a frame into a pose.
        :rtype: TransformToPoseConverter

        """
        # Params
        self.event = None

        # Frame on which the target_frame will be expressed.
        self.reference_frame = rospy.get_param('~reference_frame', None)
        assert self.reference_frame is not None, "Reference frame must be defined."

        # The frame which will be expressed with respect to the reference_frame.
        self.target_frame = rospy.get_param('~target_frame', None)
        assert self.target_frame is not None, "Target frame must be defined."

        # Object to compute transformations.
        self.listener = tf.TransformListener()

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Maximum duration to wait for a transform (in seconds).
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.converted_pose = rospy.Publisher(
            '~pose', geometry_msgs.msg.PoseStamped, queue_size=1)

        # Subscribers
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

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
        elif self.target_frame and self.reference_frame:
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
            self.publish_converted_pose()
            self.event_out.publish('e_running')
            self.reset_component_data()
            return 'IDLE'

    def publish_converted_pose(self):
        """
        Publishes the converted pose based on the transform
        between a target frame and the reference frame.

        """
        converted_pose = geometry_msgs.msg.PoseStamped()
        converted_pose.header.frame_id = self.reference_frame

        try:
            self.listener.waitForTransform(
                self.reference_frame, self.target_frame,
                rospy.Time(0), rospy.Duration(self.wait_for_transform))

            (translation, rotation) = self.listener.lookupTransform(
                self.reference_frame, self.target_frame, rospy.Time(0))

            converted_pose.pose.position.x = translation[0]
            converted_pose.pose.position.y = translation[1]
            converted_pose.pose.position.z = translation[2]
            converted_pose.pose.orientation.x = rotation[0]
            converted_pose.pose.orientation.y = rotation[1]
            converted_pose.pose.orientation.z = rotation[2]
            converted_pose.pose.orientation.w = rotation[3]

            converted_pose.header.stamp = self.listener.getLatestCommonTime(
                self.reference_frame, self.target_frame)

            self.converted_pose.publish(converted_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Error while transforming the message.\n{}".format(e))

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None


def main():
    rospy.init_node('transform_to_pose_converter', anonymous=True)
    transform_to_pose_converter = TransformToPoseConverter()
    transform_to_pose_converter.start()
