#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node instantiates a contact model for a tactile sensor in order to compute
a force array based on the sensor's contact information.

**Input(s):**
  * `contact_info`: The contact information (e.g. the output of the tactile sensor model).
    - *type:* `udom_perception_msgs/ContactInfo`
  * `event_in`: The desired event for the node:
      `e_start`: starts the component.
      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `force_array`: An array of forces representing the contact(s).
    - *type:* `udom_common_msgs/ForceArray`
  * `event_out`: The current event of the node.
      `e_running`: when the component is running.
      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `contact_model`:  Name of the contact model to be used. The model must be
      specified in the `udom_contact_model.contact_model` module.
  * `filter_force`: If set to true, it filters out residual forces that are below a specified
      threshold.
  * `threshold`: Threshold to remove residual forces of the sensor (in Newtons).

"""

import importlib
import rospy
import std_msgs.msg
import udom_common_msgs.msg
import udom_perception_msgs.msg

# Models will be searched in this module, and thus must be defined here.
MODEL_PKG_PATH = module_name = "udom_contact_model.contact_model"


class ContactModelNode(object):
    """
    Subscribes to a udom_perception_msgs.ContactInfo message and publishes a
    udom_common_msgs.ForceArray message.

    """
    def __init__(self, contact_model):
        """
        Class for a contact model.

        :param contact_model: The contact model to use.
        :type contact_model: ContactModel (or a derived class).

        :return: Node that represents a tactile sensor model.
        :rtype: ContactModelNode

        """
        # Params
        self.contact_model = contact_model()
        self.event = None
        self.contact_info = None

        # If set to true, it filters out residual forces that are below a specified threshold.
        self.filter_force = rospy.get_param('~filter_force', True)
        if self.filter_force:
            # Threshold to remove residual forces of the sensor (in Newtons).
            self.threshold = rospy.get_param("~threshold", 0.15)
            assert isinstance(self.threshold, float), \
                "'threshold' must be a float (e.g. 0.15), not '{}'".format(self.threshold)
        else:
            self.threshold = None

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.force_array = rospy.Publisher(
            "~force_array", udom_common_msgs.msg.ForceArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~contact_info", udom_perception_msgs.msg.ContactInfo, self.contact_info_cb
        )

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def contact_info_cb(self, msg):
        """
        Obtains the contact information input.

        :param msg: Contact information.
        :type msg: udom_perception_msgs.msg.ContactInfo

        """
        self.contact_info = msg

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

        if self.contact_info is not None:
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
            force_array = self.compute_force_array(self.contact_info)
            if force_array:
                self.event_out.publish('e_running')
                self.force_array.publish(force_array)
            else:
                rospy.logwarn("Error while computing the force array.")
        self.reset_component_data()
        return 'IDLE'

    def compute_force_array(self, contact_info):
        """
        Computes the contact information based on the contact information and the specified
        contact model.

        :param contact_info: The contact information.
        :type contact_info: udom_perception_msgs.msg.ContactInfo

        :return: The computed force array.
        :rtype: udom_common_msgs.ForceArray

        """
        if self.filter_force:
            force_array = self.contact_model.force_array(contact_info, self.threshold)
        else:
            force_array = self.contact_model.force_array(contact_info)
        force_array.header.stamp = rospy.Time.now()
        force_array.header.frame_id = contact_info.header.frame_id

        return force_array

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.contact_info = None
        self.event = None


def main():
    rospy.init_node("contact_model", anonymous=True)

    # Load model
    # Contact model to be be used (as a string).
    model = rospy.get_param('~contact_model', None)
    assert model is not None, "A contact model must be specified."

    # Load the specified contact model to be used (as a class).
    contact_model = getattr(importlib.import_module(MODEL_PKG_PATH), model, None)
    assert contact_model is not None, "'{}' is not defined as a model.".format(model)

    contact_model_node = ContactModelNode(contact_model)
    contact_model_node.start()
