#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node relays the topic after its offset has been removed.

**Input(s):**
  * `wrench_in`: Wrench input.
    - *type:* `geometry_msgs/WrenchStamped`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `wrench_out`: Wrench output.
    - *type:* `geometry_msgs/WrenchStamped`
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg


class TopicRelayNode(object):
    """
    Relays the topic with its original offset removed.

    """
    def __init__(self):
        """
        Instantiates a topic relay node.

        :return: Node to relay a topic.
        :rtype: TopicRelayNode

        """
        # Params
        self.event = None
        self.wrench_in = None
        self.offset_values = None
        self.calibrated = False

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.wrench_out = rospy.Publisher(
            "~wrench_out", geometry_msgs.msg.WrenchStamped, queue_size=10
        )

        # Subscribers
        rospy.Subscriber("~wrench_in", geometry_msgs.msg.WrenchStamped, self.wrench_data_cb)
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def wrench_data_cb(self, msg):
        """
        Obtains the wrench data.

        :param msg: Wrench data.
        :type msg: geometry_msgs.msg.WrenchStamped

        """
        self.wrench_in = msg

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
            elif state == 'CALIBRATION':
                state = self.calibration_state()
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
            self.offset_values = None
            self.calibrated = False
            return 'INIT'
        if self.wrench_in is not None and not self.calibrated:
            return 'CALIBRATION'
        if self.wrench_in is not None and self.calibrated:
            return 'RUNNING'
        else:
            return 'IDLE'

    def calibration_state(self):
        """
        Executes the CALIBRATION state of the state machine.
        It removes the offset value of the data.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event_out.publish('e_stopped')
            self.reset_component_data()
            self.offset_values = None
            self.calibrated = False
            return 'INIT'

        if self.offset_values is None:
            self.offset_values = [
                self.wrench_in.wrench.force.x, self.wrench_in.wrench.force.y,
                self.wrench_in.wrench.force.z, self.wrench_in.wrench.torque.x,
                self.wrench_in.wrench.torque.y, self.wrench_in.wrench.torque.z]
            self.calibrated = True
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
            self.offset_values = None
            self.calibrated = False
            return 'INIT'
        else:
            wrench_out = geometry_msgs.msg.WrenchStamped()
            wrench_out.wrench.force.x = self.wrench_in.wrench.force.x - self.offset_values[0]
            wrench_out.wrench.force.y = self.wrench_in.wrench.force.y - self.offset_values[1]
            wrench_out.wrench.force.z = self.wrench_in.wrench.force.z - self.offset_values[2]
            wrench_out.wrench.torque.x = self.wrench_in.wrench.torque.x - self.offset_values[3]
            wrench_out.wrench.torque.y = self.wrench_in.wrench.torque.y - self.offset_values[4]
            wrench_out.wrench.torque.z = self.wrench_in.wrench.torque.z - self.offset_values[5]
            wrench_out.header.frame_id = self.wrench_in.header.frame_id
            wrench_out.header.stamp = rospy.Time.now()
            self.event_out.publish('e_running')
            self.wrench_out.publish(wrench_out)
            self.reset_component_data()
            return 'IDLE'

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.wrench_in = None


def main():
    rospy.init_node("topic_relay_node", anonymous=True)
    topic_relay_node = TopicRelayNode()
    topic_relay_node.start()
