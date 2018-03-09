#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node combines the messages (udom_common_msgs.msg.ForceArray) from various subscribed
topics into a single udom_common_msgs.msg.ForceMultiArray topic which is then published.

**Input(s):**
  * `contact_info`: The messages, as array of forces, of each sensor to be merged.
    - *type:* `udom_common_msgs/ForceArray`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `force_multi_array`: A single message containing the array of forces for each sensor.
    - *type:* `udom_common_msgs/ForceMultiArray`
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `config_file`: Specifies which topics the node should subscribe to.
  * `loop_rate`: Node cycle rate (in Hz).

"""

import yaml
import numpy as np
import rospy
import std_msgs.msg
import udom_common_msgs.msg


class NodeSubscription(object):
    """
    Helper class to create an arbitrary number of subscribers.

    """
    def __init__(self, topic, message_type):
        """
        Creates a subscriber to the specified topic and message type.
        It stores the last message of that subscription in the 'msg' member.
        It assumes the messages has a header field of type std_msgs/Header.

        :return: A subscriber.
        :rtype: NodeSubscription

        """
        self.msg = None
        self.subscribed = False
        self.previous_sequence = None
        self.current_sequence = None
        rospy.Subscriber(topic, message_type, self.callback)

    def callback(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: Depends on the specified message_type.

        """
        if not self.subscribed:
            self.previous_sequence = msg.header.seq
        self.subscribed = True
        self.current_sequence = msg.header.seq
        self.msg = msg


class ForceMergerNode(object):
    """
    Subscribes to various topics of message type udom_common_msgs.msg.ForceArray and
    combines them into a single udom_common_msgs.msg.ForceMultiArray topic which is
    then published.

    """
    def __init__(self):
        """
        Instantiates a force merger node.

        :return: Node to merger force messages.
        :rtype: ForceMergerNode

        """
        # Params
        self.event = None

        self.config_file = rospy.get_param("~config_file", None)
        assert self.config_file is not None, "A configuration file must be specified."
        self.nodes = yaml.load(open(self.config_file, "r"))

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.force_multi_array = rospy.Publisher(
            "~force_multi_array", udom_common_msgs.msg.ForceMultiArray, queue_size=1,
            tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)

        self.force_subscribers = [
            NodeSubscription(topic, udom_common_msgs.msg.ForceArray)
            for _, topic in self.nodes.items()]

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
        elif self.has_new_data():
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
            force_info = [sub.msg for sub in self.force_subscribers]
            self.event_out.publish('e_running')
            self.force_multi_array.publish(force_info)
        self.reset_component_data()
        return 'IDLE'

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None

    def has_new_data(self):
        """
        Checks if all subscribed topics have new data.

        :return: True if all topics have new data, false otherwise.
        :rtype: bool

        """
        if not np.all([sub.subscribed for sub in self.force_subscribers]):
            return False
        updated = [
            sub.previous_sequence != sub.current_sequence for sub in self.force_subscribers
        ]
        if np.all(updated):
            for sub in self.force_subscribers:
                sub.previous_sequence = sub.current_sequence
            return True
        else:
            return False


def main():
    rospy.init_node("force_merger_node", anonymous=True)
    force_merger_node = ForceMergerNode()
    force_merger_node.start()
