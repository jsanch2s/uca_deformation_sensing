#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node transforms every force in a udom_common_msgs.msg.ForceMultiArray into a single
message of type udom_common_msgs.msg.ForceArray with a single specified reference frame.

<sub>[**Note**: This node requires that a reference frame is specified and that it exists.]</sub>

**Input(s):**
  * `force_in`: The forces to be transformed.
    - *type:* `udom_common_msgs/ForceMultiArray`
  * `event_in`: The desired event for the node:
      `e_start`: starts the component.
      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `force_out`: An array of forces on which all elements are specified with respect to a
        single reference frame.
    - *type:* `udom_common_msgs/ForceArray`
  * `event_out`: The current event of the node.
      `e_running`: when the component is running.
      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `wait_for_transform`: Maximum duration to wait for a transform (in seconds).
  * `reference_frame`: The forces will be described with respect to this frame.

"""

import rospy
import tf2_ros
import std_msgs.msg
import udom_common_msgs.msg
import udom_geometric_transformation.transformation_utils as transformation_utils
from itertools import izip as zip


class ForceTransformerNode(object):
    """
    Subscribes to a udom_common_msgs.msg.ForceMultiArray topic and transforms each of the
    forces with respect to a single specified frame. It publishes a message of type
    udom_common_msgs.msg.ForceArray with all the forces.

    """
    def __init__(self):
        """
        Instantiates a force transformer node.

        :return: Node to transform forces into a single reference frame.
        :rtype: ForceTransformerNode

        """
        # Params
        self.event = None
        self.force_info = None

        # Object to compute transformations.
        self.listener = transformation_utils.GeometryTransformer()

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Maximum duration to wait for a transform (in seconds).
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # The forces will be described with respect to this frame.
        self.reference_frame = rospy.get_param("~reference_frame", None)
        assert self.reference_frame is not None, "A reference frame must be specified."

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.force_out = rospy.Publisher(
            "~force_out", udom_common_msgs.msg.ForceArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~force_in', udom_common_msgs.msg.ForceMultiArray, self.force_in_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def force_in_cb(self, msg):
        """
        Obtains the force information input.

        :param msg: Force information.
        :type msg: udom_common_msgs.msg.ForceMultiArray

        """
        self.force_info = msg

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
        elif self.force_info is not None:
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
            try:
                force_out = self.transform_force(self.force_info)
                self.event_out.publish('e_running')
                self.force_out.publish(force_out)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("Error while transforming the message.\n{}".format(e))
            self.reset_component_data()
            return 'IDLE'

    def transform_force(self, force_in):
        """
        Transforms the forces in the 'force_in' message to the specified reference frame.

        :param force_in: The force information to transform.
        :type force_in: udom_common_msgs.msg.ForceMultiArray

        :return: The transformed force information.
        :rtype: udom_common_msgs.msg.ForceArray

        """
        force_out = udom_common_msgs.msg.ForceArray()

        force_out.positions = [
            self.listener.transform_point(
                point, force.header, self.reference_frame, self.wait_for_transform)
            for force in force_in.forces for point in force.positions
        ]
        force_out.wrenches = [
            self.listener.transform_wrench(
                wrench, force.header, self.reference_frame,
                (point.x, point.y, point.z), self.wait_for_transform
            )
            for force in force_in.forces
            for wrench, point in zip(force.wrenches, force.positions)
        ]

        force_out.header.frame_id = self.reference_frame
        force_out.header.stamp = rospy.Time.now()
        return force_out

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.force_info = None
        self.event = None


def main():
    rospy.init_node("force_transformer_node", anonymous=True)
    force_transformer_node = ForceTransformerNode()
    force_transformer_node.start()
