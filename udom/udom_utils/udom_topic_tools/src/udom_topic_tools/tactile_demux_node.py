#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node selects a single general tactile data message (BioTacAll), specified
by a given sensor index, and publishes it as tactile stamped message.

**Input(s):**
  * `tactile_data_in`: The tactile data for all the sensors in the hand.
    - *type:* `sr_robot_msgs/BiotacAll`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `tactile_data_out`: A stamped tactile message for the selected sensor.
    - *type:* `udom_perception_msgs/BiotacStamped`
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `sensor_index`: Index of the sensor to be selected.
  * `sensor_frame`: Reference frame of the sensor to be selected.
  * `loop_rate`: Node cycle rate (in Hz).

"""

import rospy
import std_msgs.msg
import sr_robot_msgs.msg
import udom_perception_msgs.msg


class TactileDemux(object):
    """
    Selects, from a BiotacAll message, a Biotac message specified by the sensor index.

    """
    def __init__(self):
        """
        Instantiates a tactile demux node.

        :return: Node to select Biotac messages.
        :rtype: TactileDemux

        """
        # Params
        self.event = None
        self.tactile_data = None

        # Index of the sensor to be selected.
        self.sensor_index = rospy.get_param('~sensor_index', None)
        assert self.sensor_index is not None, "Sensor index must be specified."

        # Reference frame of the sensor to be selected.
        self.sensor_frame = rospy.get_param('~sensor_frame', None)
        assert self.sensor_frame is not None, "Sensor frame must be specified."

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.tactile_data_out = rospy.Publisher(
            "~tactile_data_out", udom_perception_msgs.msg.BiotacStamped, queue_size=1,
            tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~tactile_data_in", sr_robot_msgs.msg.BiotacAll, self.tactile_data_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs/String

        """
        self.event = msg.data

    def tactile_data_cb(self, msg):
        """
        Obtains the desired tactile data.

        :param msg: Tactile data.
        :type msg: sr_robot_msgs/BiotacAll

        """
        self.tactile_data = msg

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

        if self.tactile_data is not None:
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
            tactile_data_out = self.transform_tactile_data()
            if tactile_data_out:
                self.event_out.publish('e_running')
                self.tactile_data_out.publish(tactile_data_out)
            else:
                rospy.logwarn("Error while demuxing the tactile data.")
            self.reset_component_data()
            return 'IDLE'

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.tactile_data = None
        self.event = None

    def transform_tactile_data(self):
        """
        Transforms the tactile data, as specified by the sensor index, in a
        sr_robot_msgs/BiotacAll message  into a udom_perception_msgs/BiotacStamped message.

        :return: The selected tactile data in a BiotacStamped message.
        :rtype: udom_perception_msgs/BiotacAll

        """
        tactile_data = udom_perception_msgs.msg.BiotacStamped()
        tactile_data.header.frame_id = self.sensor_frame
        tactile_data.header.stamp = rospy.Time.now()
        biotac = self.tactile_data.tactiles[self.sensor_index]
        tactile_data.pac0 = biotac.pac0
        tactile_data.pac1 = biotac.pac1
        tactile_data.pdc = biotac.pdc
        tactile_data.tac = biotac.tac
        tactile_data.tdc = biotac.tdc
        tactile_data.electrodes = biotac.electrodes
        return tactile_data


def main():
    rospy.init_node("tactile_demux", anonymous=True)
    tactile_demux = TactileDemux()
    tactile_demux.start()
