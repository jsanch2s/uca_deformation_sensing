#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node uses a pipeline of components to demonstrate the force estimation.
The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

It uses the following nodes:
  * `udom_sensor_model/tactile_sensor_model`
  * `udom_topic_tools/tactile_demux`
  * `udom_topic_tools/topic_relay`

**Assumptions:**
  * It assumes a BioTac sensor as the input sensor.

**Input(s):**
  * `tactile_info`: The output data of a BioTac sensor.
    - *type:* `udom_perception_msgs/BiotacStamped`
  * `event_in`: The desired event for the node:
      `e_start`: starts the component.
      `e_stop`: stops the component.

**Output(s):**
  * `actual_wrench`: Actual wrench information.
    - *type:* `geometry_msgs/WrenchStamped`
  * `estimated_wrench`: Estimated wrench information for each of the five fingers.
    - *type:* `udom_perception_msgs/ContactInfo`
  * `event_out`: The current event of the node.
      `e_running`: when the component is running.
      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).

"""

import rospy
import std_msgs.msg


class Coordinator(object):
    """
    Coordinates a set of components to estimate the force based on the output of tactile
    sensors.

    """
    def __init__(self):
        """
        Instantiates a node to coordinate the components of the force estimation
        pipeline.

        """
        # Params
        self.started_components = False
        self.event = None
        self.sensor_model_ff_status = None
        self.sensor_model_mf_status = None
        self.sensor_model_rf_status = None
        self.sensor_model_lf_status = None
        self.sensor_model_th_status = None
        self.tactile_demux_ff_status = None
        self.tactile_demux_mf_status = None
        self.tactile_demux_rf_status = None
        self.tactile_demux_lf_status = None
        self.tactile_demux_th_status = None
        self.topic_relay_status = None

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.start_sensor_model_ff = rospy.Publisher(
            "~start_sensor_model_ff", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_sensor_model_mf = rospy.Publisher(
            "~start_sensor_model_mf", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_sensor_model_rf = rospy.Publisher(
            "~start_sensor_model_rf", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_sensor_model_lf = rospy.Publisher(
            "~start_sensor_model_lf", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_sensor_model_th = rospy.Publisher(
            "~start_sensor_model_th", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_tactile_demux_ff = rospy.Publisher(
            "~start_tactile_demux_ff", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_tactile_demux_mf = rospy.Publisher(
            "~start_tactile_demux_mf", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_tactile_demux_rf = rospy.Publisher(
            "~start_tactile_demux_rf", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_tactile_demux_lf = rospy.Publisher(
            "~start_tactile_demux_lf", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_tactile_demux_th = rospy.Publisher(
            "~start_tactile_demux_th", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_topic_relay = rospy.Publisher(
            "~start_topic_relay", std_msgs.msg.String, queue_size=10, latch=True
        )

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~sensor_model_ff_status", std_msgs.msg.String, self.sensor_model_ff_status_cb
        )
        rospy.Subscriber(
            "~sensor_model_mf_status", std_msgs.msg.String, self.sensor_model_mf_status_cb
        )
        rospy.Subscriber(
            "~sensor_model_rf_status", std_msgs.msg.String, self.sensor_model_rf_status_cb
        )
        rospy.Subscriber(
            "~sensor_model_lf_status", std_msgs.msg.String, self.sensor_model_lf_status_cb
        )
        rospy.Subscriber(
            "~sensor_model_th_status", std_msgs.msg.String, self.sensor_model_th_status_cb
        )
        rospy.Subscriber(
            "~tactile_demux_ff_status", std_msgs.msg.String, self.tactile_demux_ff_status_cb
        )
        rospy.Subscriber(
            "~tactile_demux_mf_status", std_msgs.msg.String, self.tactile_demux_mf_status_cb
        )
        rospy.Subscriber(
            "~tactile_demux_rf_status", std_msgs.msg.String, self.tactile_demux_rf_status_cb
        )
        rospy.Subscriber(
            "~tactile_demux_lf_status", std_msgs.msg.String, self.tactile_demux_lf_status_cb
        )
        rospy.Subscriber(
            "~tactile_demux_th_status", std_msgs.msg.String, self.tactile_demux_th_status_cb
        )
        rospy.Subscriber(
            "~topic_relay_status", std_msgs.msg.String, self.topic_relay_status_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def sensor_model_ff_status_cb(self, msg):
        """
        Obtains the status of the sensor model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.sensor_model_ff_status = msg.data

    def sensor_model_mf_status_cb(self, msg):
        """
        Obtains the status of the contact model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.sensor_model_mf_status = msg.data

    def sensor_model_rf_status_cb(self, msg):
        """
        Obtains the status of the sensor model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.sensor_model_rf_status = msg.data

    def sensor_model_lf_status_cb(self, msg):
        """
        Obtains the status of the contact model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.sensor_model_lf_status = msg.data

    def sensor_model_th_status_cb(self, msg):
        """
        Obtains the status of the contact model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.sensor_model_th_status = msg.data

    def tactile_demux_ff_status_cb(self, msg):
        """
        Obtains the status of the tactile demux (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.tactile_demux_ff_status = msg.data

    def tactile_demux_mf_status_cb(self, msg):
        """
        Obtains the status of the tactile demux (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.tactile_demux_mf_status = msg.data

    def tactile_demux_rf_status_cb(self, msg):
        """
        Obtains the status of the tactile demux (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.tactile_demux_rf_status = msg.data

    def tactile_demux_lf_status_cb(self, msg):
        """
        Obtains the status of the tactile demux (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.tactile_demux_lf_status = msg.data

    def tactile_demux_th_status_cb(self, msg):
        """
        Obtains the status of the tactile demux (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.tactile_demux_th_status = msg.data

    def topic_relay_status_cb(self, msg):
        """
        Obtains the status of the force merger (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.topic_relay_status = msg.data

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
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
            return 'RUNNING'
        else:
            return 'INIT'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        self.toggle_components(self.event)

        if self.event == 'e_stop':
            status = 'e_stopped'
            self.event_out.publish(status)
            self.reset_component_data(status)
            return 'INIT'
        else:
            return 'RUNNING'

    def toggle_components(self, event):
        """
        Starts or stops the necessary components based on the event.

        :param event: The event that determines either to start or stop the components.
        :type event: str

        """
        if event == 'e_stop':
            self.start_sensor_model_ff.publish('e_stop')
            self.start_sensor_model_mf.publish('e_stop')
            self.start_sensor_model_rf.publish('e_stop')
            self.start_sensor_model_lf.publish('e_stop')
            self.start_sensor_model_th.publish('e_stop')
            self.start_tactile_demux_ff.publish('e_stop')
            self.start_tactile_demux_mf.publish('e_stop')
            self.start_tactile_demux_rf.publish('e_stop')
            self.start_tactile_demux_lf.publish('e_stop')
            self.start_tactile_demux_th.publish('e_stop')
            self.start_topic_relay.publish('e_stop')
            self.started_components = False

        if event == 'e_start' and not self.started_components:
            self.start_sensor_model_ff.publish('e_start')
            self.start_sensor_model_mf.publish('e_start')
            self.start_sensor_model_rf.publish('e_start')
            self.start_sensor_model_lf.publish('e_start')
            self.start_sensor_model_th.publish('e_start')
            self.start_tactile_demux_ff.publish('e_start')
            self.start_tactile_demux_mf.publish('e_start')
            self.start_tactile_demux_rf.publish('e_start')
            self.start_tactile_demux_lf.publish('e_start')
            self.start_tactile_demux_th.publish('e_start')
            self.start_topic_relay.publish('e_start')
            self.started_components = True

    def reset_component_data(self, result):
        """
        Clears the data of the component.

        :param result: The result_contact_data of the component, e.g. stopped, failure, success.
        :type result: str

        """
        self.toggle_components(result)
        self.event = None
        self.sensor_model_ff_status = None
        self.sensor_model_mf_status = None
        self.topic_relay_status = None
        self.started_components = False


def main():
    rospy.init_node("coordinator", anonymous=True)
    coordinator = Coordinator()
    coordinator.start()
