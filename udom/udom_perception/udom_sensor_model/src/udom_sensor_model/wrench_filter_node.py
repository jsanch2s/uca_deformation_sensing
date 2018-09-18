#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node filters a wrench using different types of filters (e.g. moving average (MA),
exponential moving average (EMA), etc.).

**Input(s):**

  * `wrench_in`: The sensor's output wrench.
    - *type:* `geometry_msgs/WrenchStamped`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

    - *type:* `std_msgs/String`

**Output(s):**

  * `wrench_out`: The filtered wrench.
    - *type:* `geometry_msgs/WrenchStamped`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `filter_type`: Name of the filter to apply.

  * `samples`: Number of samples to use by finite impulse response filters (e.g. MA).

  * `alpha`: Filter coefficient used by infinite impulse response filters (e.g. EMA).

"""

import copy
import collections
import numpy as np

import rospy
import std_msgs.msg
import geometry_msgs.msg


FILTER_TYPES = {
    'ma': 'Moving average',
    'ema': 'Exponential moving average'
}


class WrenchFilter(object):
    """
    Filters the input wrench.

    """
    def __init__(self):
        """
        Returns a wrench filter.

        :return: Wrench filter.
        :rtype: WrenchFilter

        """
        # Params
        self.event = None
        self.wrench_data = None

        # Name of the filter to apply.
        self.filter_type = rospy.get_param('~filter_type', 'ma')
        assert self.filter_type in FILTER_TYPES, "'{}' is not a valid filter. " \
            "Available filters are: {}".format(self.filter_type, FILTER_TYPES)

        # Number of samples to use by finite impulse response filters (e.g. MA).
        self.samples = rospy.get_param('~samples', 20)

        # Filter coefficient used by infinite impulse response filters (e.g. EMA).
        self.alpha = rospy.get_param('~alpha', 0.01)

        if self.filter_type == 'ema':
            self.samples = 2

        self.filtered_wrench = collections.deque([], self.samples)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 500))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.wrench_out = rospy.Publisher(
            "~wrench_out", geometry_msgs.msg.WrenchStamped, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~wrench_in", geometry_msgs.msg.WrenchStamped, self.wrench_in_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def wrench_in_cb(self, msg):
        """
        Obtains the sensor's output wrench.

        :param msg: Wrench data.
        :type msg: geometry_msgs.msg.WrenchStamped

        """
        self.wrench_data = msg

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
            rospy.logdebug("event: {0}".format(self.event))
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
            self.filtered_wrench.clear()
            self.reset_component_data()
            return 'INIT'
        elif self.wrench_data is not None:
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
            self.filtered_wrench.clear()
            self.reset_component_data()
            return 'INIT'
        else:
            self.filtered_wrench.append(self.wrench_data.wrench)
            latest_wrench = self.filtered_wrench[-1]
            wrench_out = self.filter_wrench(latest_wrench)
            if wrench_out:
                self.event_out.publish('e_running')
                self.wrench_out.publish(wrench_out)
            else:
                rospy.logwarn("Error while filtering the wrench.")
            self.reset_component_data()
            return 'IDLE'

    def filter_wrench(self, wrench_in):
        """
        Filters the wrench based on the specified filter type.

        :param wrench_in: The wrench to filter.
        :type wrench_in: geometry_msgs.msg.WrenchStamped

        :return: The filtered wrench.
        :rtype: geometry_msgs.msg.WrenchStamped

        """
        wrench_stamped = geometry_msgs.msg.WrenchStamped()
        wrench = geometry_msgs.msg.Wrench()

        if self.filter_type == 'ma':
            wrench = self.ma()
        elif self.filter_type == 'ema':
            wrench = self.ema(wrench_in)

        wrench_stamped.header.stamp = rospy.Time.now()
        wrench_stamped.header.frame_id = self.wrench_data.header.frame_id
        wrench_stamped.wrench = wrench

        return wrench_stamped

    def ma(self):
        """
        Filters a wrench using the moving average filter.

        :return: The filtered wrench.
        :rtype: geometry_msgs.msg.WrenchStamped

        """
        wrench_out = geometry_msgs.msg.Wrench()
        wrench_out.force.x = np.mean([wrench.force.x for wrench in self.filtered_wrench])
        wrench_out.force.y = np.mean([wrench.force.y for wrench in self.filtered_wrench])
        wrench_out.force.z = np.mean([wrench.force.z for wrench in self.filtered_wrench])
        wrench_out.torque.x = np.mean([wrench.torque.x for wrench in self.filtered_wrench])
        wrench_out.torque.y = np.mean([wrench.torque.y for wrench in self.filtered_wrench])
        wrench_out.torque.z = np.mean([wrench.torque.z for wrench in self.filtered_wrench])

        return wrench_out

    def ema(self, wrench_in, alpha=0.01):
        """
        Filters a wrench using the exponential moving average filter.

        :param wrench_in: The wrench to be filtered.
        :type wrench_in: geometry_msgs.msg.WrenchStamped

        :param alpha: Smoothing factor between 0 and 1. It represents the degree of weighting
        decrease, i.e. the higher it is the less important older observations become.
        :type alpha: float

        :return: The filtered wrench.
        :rtype: geometry_msgs.msg.WrenchStamped

        """
        wrench_out = geometry_msgs.msg.Wrench()
        if self.filtered_wrench:
            wrench_out.force.x = alpha * wrench_in.force.x + \
                (1 - alpha) * self.filtered_wrench[0].force.x
            wrench_out.force.y = alpha * wrench_in.force.y + \
                (1 - alpha) * self.filtered_wrench[0].force.y
            wrench_out.force.z = alpha * wrench_in.force.z + \
                (1 - alpha) * self.filtered_wrench[0].force.z
            wrench_out.torque.x = alpha * wrench_in.torque.x + \
                (1 - alpha) * self.filtered_wrench[0].torque.x
            wrench_out.torque.y = alpha * wrench_in.torque.y + \
                (1 - alpha) * self.filtered_wrench[0].torque.y
            wrench_out.torque.z = alpha * wrench_in.torque.z + \
                (1 - alpha) * self.filtered_wrench[0].torque.z
        else:
            wrench_out = copy.deepcopy(wrench_in)

        return wrench_out

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.wrench_data = None
        self.event = None


def main():
    rospy.init_node("wrench_filter", anonymous=True)
    wrench_filter = WrenchFilter()
    wrench_filter.start()
