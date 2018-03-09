#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node publishes the input topic at a desired rate.

**Input(s):**
  * `mesh_in`: Mesh input.
    - *type:* `udom_modeling_msgs/Mesh`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `mesh_out`: Mesh output.
    - *type:* `udom_modeling_msgs/Mesh`
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).

"""

import rospy
import std_msgs.msg
import udom_modeling_msgs.msg


class TopicThrottleNode(object):
    """
    Publishes the topic at the desired rate.

    """
    def __init__(self):
        """
        Instantiates a topic relay node.

        :return: Node to relay a topic.
        :rtype: TopicThrottleNode

        """
        # Params
        self.event = None
        self.mesh_in = None

        # Node cycle rate (in Hz).
        desired_rate = rospy.get_param('~loop_rate', 100)
        # We double it so it takes into account that one iteration occurs in the
        # running state and one in the idle state.
        desired_rate *= 2
        self.loop_rate = rospy.Rate(desired_rate)

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.mesh_out = rospy.Publisher(
            "~mesh_out", udom_modeling_msgs.msg.Mesh, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~mesh_in", udom_modeling_msgs.msg.Mesh, self.mesh_cb)
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def mesh_cb(self, msg):
        """
        Obtains the mesh data.

        :param msg: Mesh data.
        :type msg: udom_modeling_msgs.msg.Mesh

        """
        self.mesh_in = msg

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
        if self.mesh_in is not None:
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
            self.event_out.publish('e_running')
            self.mesh_out.publish(self.mesh_in)
            return 'IDLE'

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.mesh_in = None


def main():
    rospy.init_node("topic_throttle_node", anonymous=True)
    topic_throttle_node = TopicThrottleNode()
    topic_throttle_node.start()
