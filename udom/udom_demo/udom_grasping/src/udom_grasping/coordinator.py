#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node uses a pipeline of components to perform a reactive grasp based on
the tactile data obtained from the fingertips of the Shadow hand (e.g. to apply
a desired grasp force on the object).

It uses the following nodes:


  * `udom_topic_tools/tactile_demux`

  * `udom_sensor_model/tactile_sensor_model`

  * `udom_grasp_control/reactive_grasp`

**Assumptions:**
  * It assumes five BioTac sensors and the joint_states of the Shadow hand as input.

**Input(s):**

  * `tactile_info`: The output data of five BioTac sensors.

    - *type:* `sr_robot_msgs/BiotacAll`

  * `joint_states`: The joints' state of the Shadow hand.

    - *type:* `sensor_msgs/JointState`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_reset`: resets the hand to its initial open configuration.

    - *type:* `std_msgs/String`

**Output(s):**
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `force_threshold_max`: Maximum force, if the current force of any finger exceeds this
        value, then the finger will move backwards (in N).

  * `force_threshold_min`: Minimum force, if the current force of any finger is less than
        this value, then the finger will move forwards (in N).

  * `increment`: The value of the joint position increment to move each finger (in radians).

  * `increment_thumb`: The value of the joint position increment to move the thumb
        (in radians).

  * `force_feedback`: Whether to use force feedback or not.

"""

import rospy
import std_msgs.msg


class Coordinator(object):
    """
    Coordinates a set of components to estimate the deformation of an object caused by
    an external force.

    """
    def __init__(self):
        """
        Instantiates a node to coordinate the components of the deformation sensing
        pipeline.

        """
        # Params
        self.started_components = False
        self.event = None

        self.first_finger_sensor_status = None
        self.middle_finger_sensor_status = None
        self.ring_finger_sensor_status = None
        self.little_finger_sensor_status = None
        self.thumb_sensor_status = None

        self.first_finger_sensor_model_status = None
        self.middle_finger_sensor_model_status = None
        self.ring_finger_sensor_model_status = None
        self.little_finger_sensor_model_status = None
        self.thumb_sensor_model_status = None

        self.reactive_grasp_status = None

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)

        self.start_first_finger_sensor = rospy.Publisher(
            "~start_first_finger_sensor", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_middle_finger_sensor = rospy.Publisher(
            "~start_middle_finger_sensor", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_ring_finger_sensor = rospy.Publisher(
            "~start_ring_finger_sensor", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_little_finger_sensor = rospy.Publisher(
            "~start_little_finger_sensor", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_thumb_sensor = rospy.Publisher(
            "~start_thumb_sensor", std_msgs.msg.String, queue_size=10, latch=True)

        self.start_first_finger_sensor_model = rospy.Publisher(
            "~start_first_finger_sensor_model", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_middle_finger_sensor_model = rospy.Publisher(
            "~start_middle_finger_sensor_model", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_ring_finger_sensor_model = rospy.Publisher(
            "~start_ring_finger_sensor_model", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_little_finger_sensor_model = rospy.Publisher(
            "~start_little_finger_sensor_model", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_thumb_sensor_model = rospy.Publisher(
            "~start_thumb_sensor_model", std_msgs.msg.String, queue_size=10, latch=True)

        self.start_reactive_grasp = rospy.Publisher(
            "~start_reactive_grasp", std_msgs.msg.String, queue_size=10, latch=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)

        rospy.Subscriber(
            "~first_finger_sensor_status", std_msgs.msg.String, self.ff_sensor_status_cb)
        rospy.Subscriber(
            "~middle_finger_sensor_status", std_msgs.msg.String, self.mf_sensor_status_cb)
        rospy.Subscriber(
            "~ring_finger_sensor_status", std_msgs.msg.String, self.rf_sensor_status_cb)
        rospy.Subscriber(
            "~little_finger_sensor_status", std_msgs.msg.String, self.lf_sensor_status_cb)
        rospy.Subscriber(
            "~thumb_sensor_status", std_msgs.msg.String, self.th_sensor_status_cb)

        rospy.Subscriber(
            "~first_finger_sensor_model_status", std_msgs.msg.String, self.ff_model_status_cb)
        rospy.Subscriber(
            "~middle_finger_sensor_model_status", std_msgs.msg.String, self.mf_model_status_cb)
        rospy.Subscriber(
            "~ring_finger_sensor_model_status", std_msgs.msg.String, self.rf_model_status_cb)
        rospy.Subscriber(
            "~little_finger_sensor_model_status", std_msgs.msg.String, self.lf_model_status_cb)
        rospy.Subscriber(
            "~thumb_sensor_model_status", std_msgs.msg.String, self.th_model_status_cb)

        rospy.Subscriber(
            "~reactive_grasp_status", std_msgs.msg.String, self.reactive_grasp_status_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def ff_sensor_status_cb(self, msg):
        """
        Obtains the status of the tactile sensor for the first finger (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.first_finger_sensor_status = msg.data

    def mf_sensor_status_cb(self, msg):
        """
        Obtains the status of the tactile sensor for the middle finger (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.middle_finger_sensor_status = msg.data

    def rf_sensor_status_cb(self, msg):
        """
        Obtains the status of the tactile sensor for the ring finger (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.ring_finger_sensor_status = msg.data

    def lf_sensor_status_cb(self, msg):
        """
        Obtains the status of the tactile sensor for the little finger (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.little_finger_sensor_status = msg.data

    def th_sensor_status_cb(self, msg):
        """
        Obtains the status of the tactile sensor for the thumb (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.thumb_sensor_status = msg.data

    def ff_model_status_cb(self, msg):
        """
        Obtains the status of the first finger sensor model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.first_finger_sensor_model_status = msg.data

    def mf_model_status_cb(self, msg):
        """
        Obtains the status of the middle finger sensor model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.middle_finger_sensor_model_status = msg.data

    def rf_model_status_cb(self, msg):
        """
        Obtains the status of the ring finger sensor model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.ring_finger_sensor_model_status = msg.data

    def lf_model_status_cb(self, msg):
        """
        Obtains the status of the little finger sensor model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.little_finger_sensor_model_status = msg.data

    def th_model_status_cb(self, msg):
        """
        Obtains the status of the thumb sensor model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.thumb_sensor_model_status = msg.data

    def reactive_grasp_status_cb(self, msg):
        """
        Obtains the status of the reactive grasp (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.reactive_grasp_status = msg.data

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start...")
        while not rospy.is_shutdown():

            self.toggle_components(self.event)

            if self.event == 'e_stop':
                self.event_out.publish('e_stopped')
            elif self.event == 'e_start':
                self.event_out.publish('e_started')
            elif self.event == 'e_reset':
                self.event_out.publish('e_restarted')

            self.loop_rate.sleep()

    def toggle_components(self, event):
        """
        Starts or stops the necessary components based on the event.

        :param event: The event that determines either to start or stop the components.
        :type event: str

        """
        if event == 'e_stop':
            self.start_first_finger_sensor.publish('e_stop')
            self.start_middle_finger_sensor.publish('e_stop')
            self.start_ring_finger_sensor.publish('e_stop')
            self.start_little_finger_sensor.publish('e_stop')
            self.start_thumb_sensor.publish('e_stop')

            self.start_first_finger_sensor_model.publish('e_stop')
            self.start_middle_finger_sensor_model.publish('e_stop')
            self.start_ring_finger_sensor_model.publish('e_stop')
            self.start_little_finger_sensor_model.publish('e_stop')
            self.start_thumb_sensor_model.publish('e_stop')

            self.start_reactive_grasp.publish('e_stop')
            self.started_components = False

        if event == 'e_reset':
            self.start_reactive_grasp.publish('e_reset')
            self.started_components = False

        if event == 'e_start' and not self.started_components:
            self.start_first_finger_sensor.publish('e_start')
            self.start_middle_finger_sensor.publish('e_start')
            self.start_ring_finger_sensor.publish('e_start')
            self.start_little_finger_sensor.publish('e_start')
            self.start_thumb_sensor.publish('e_start')

            self.start_first_finger_sensor_model.publish('e_start')
            self.start_middle_finger_sensor_model.publish('e_start')
            self.start_ring_finger_sensor_model.publish('e_start')
            self.start_little_finger_sensor_model.publish('e_start')
            self.start_thumb_sensor_model.publish('e_start')

            self.start_reactive_grasp.publish('e_start')
            self.started_components = True


def main():
    rospy.init_node("coordinator", anonymous=True)
    coordinator = Coordinator()
    coordinator.start()
