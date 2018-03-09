#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node moves a specified finger of the Shadow Hand Robot to perform four exploratory
movements:

 1. Press: Begins without contact and ends with the finger pressing on the object.
 2. Hold: Keeps the finger contacting the object for a specified amount of seconds.
 3. Release: Moves the finger back its original, without contact, position.
 4. Sideways: Slides the finger side to side while making contact with the object (e.g.
  at 25-50% of the position of Press).

**Input(s):**
  * `joint_state`: Current joint position of Joint 3.
    - *type:* `std_msgs/Float64`
  * `joint_state_distal`: Current joint position of Joint 4.
    - *type:* `std_msgs/Float64`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`
  * `finger_controller`: Commands a joint position to the specified finger.
    - *type:* `std_msgs/Float64`
  * `motion`: The current exploratory motion.
    - *type:* `std_msgs/Float64`

**Parameter(s):**
  * `finger`: Name of the finger to be used in the experiments (see config file).
  * `initial position`: Joint position to start the Press motion.
  * `final position`: Joint position to end the Press motion.
  * `left position`: Joint position for the left end of the Sideways motion.
  * `right position`: Joint position for the right end of the Sideways motion.
  * `center position`: Joint position for the center end of the Sideways motion.
  * `duration`: Duration the finger should be in contact with the probe (in seconds).
  * `tolerance`: Distance error considered to check if a joint position has been reached
    (in radians).
  * `timeout`: Duration to allow a motion to be completed (in seconds).
  * `loop_rate`: Node cycle rate (in Hz).

"""

import yaml
import rospy
import std_msgs.msg
import control_msgs.msg


class FingerExperimentNode(object):
    """
    Controls the finger's motion to perform the contact experiment.

    """
    def __init__(self):
        """
        Instantiates the node.

        :return: Node to perform a finger contact experiment.
        :rtype: FingerExperimentNode

        """
        # Params
        self.event = None
        self.joint_state = None
        self.joint_state_sideways = None
        # Duration of the current motion (in seconds).
        self.start_time = rospy.Time.now()

        self.config_file = rospy.get_param("~config_file", None)
        assert self.config_file is not None, "A configuration file must be specified."
        self.config_params = yaml.load(open(self.config_file, "r"))

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 50))
        # Name of the finger to be used in the experiments (see config file).
        self.finger = rospy.get_param('~finger', 'ff')
        # Joint position to start the Press motion.
        self.initial_configuration = self.config_params[self.finger + '/initial']
        # Joint position enough to make contact with the object (for the Sideways motion).
        self.mid_configuration = self.config_params[self.finger + '/mid']
        # Joint position to end the Press motion.
        self.final_configuration = self.config_params[self.finger + '/final']
        # Joint position for the left end of the Sideways motion.
        self.left_configuration = self.config_params[self.finger + '_side/left']
        # Joint position for the right end of the Sideways motion.
        self.right_configuration = self.config_params[self.finger + '_side/right']
        # Joint position for the center end of the Sideways motion.
        self.center_configuration = self.config_params[self.finger + '_side/center']
        # Duration the finger should be in contact with the probe (in seconds).
        self.duration = rospy.get_param('~duration', 3)
        # Distance error considered to check if a joint position has been reached (in radians).
        self.tolerance = rospy.get_param('~tolerance', 0.1)
        # Duration to allow a motion to be completed (in seconds).
        self.timeout = rospy.Duration.from_sec(rospy.get_param('~timeout', 3))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.motion = rospy.Publisher("~motion", std_msgs.msg.Int16, queue_size=10)
        self.finger_controller = rospy.Publisher(
            self.config_params[self.finger], std_msgs.msg.Float64, queue_size=10)
        self.finger_controller_sideways = rospy.Publisher(
            self.config_params[self.finger + '_side'], std_msgs.msg.Float64, queue_size=10)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            self.config_params[self.finger + '_sub'],
            control_msgs.msg.JointControllerState, self.joint_state_cb)
        rospy.Subscriber(
           self.config_params[self.finger + '_sub_side'],
           control_msgs.msg.JointControllerState, self.joint_state_sideways_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def joint_state_cb(self, msg):
        """
        Obtains the joint position of Joint 3.

        :param msg: Joint position.
        :type msg: control_msgs.msg.JointControllerState

        """
        self.joint_state = msg.process_value

    def joint_state_sideways_cb(self, msg):
        """
        Obtains the joint position of Joint 4.

        :param msg: Joint position.
        :type msg: control_msgs.msg.JointControllerState

        """
        self.joint_state_sideways = msg.process_value

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
        elif self.joint_state and self.joint_state_sideways:
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
            self.run_experiment()
            self.reset_component_data()
            return 'INIT'

    def run_experiment(self):
        """
        Moves the finger in order to make contact with the probe and it maintains
        the contact for the specified duration. It also records the tactile and wrench
        data starting from before the finger moves into contact with the probe and
        ending when the finger has return to its original position (i.e. without
        contacting the probe).

        """
        rospy.loginfo("Starting experiment...")
        rospy.loginfo(
            "Moving to initial configuration {}...".format(self.initial_configuration))
        self.finger_controller.publish(
            std_msgs.msg.Float64(data=self.initial_configuration))

        self.start_time = rospy.Time.now()
        while not self.is_position_reached(self.joint_state, self.initial_configuration):
            rospy.sleep(0.01)

        rospy.loginfo(
            "[PRESS] Moving to final configuration {}...".format(
                self.final_configuration))
        self.motion.publish(std_msgs.msg.Int16(data=1))
        self.finger_controller.publish(
            std_msgs.msg.Float64(data=self.final_configuration))

        self.start_time = rospy.Time.now()
        while not self.is_position_reached(self.joint_state, self.final_configuration):
            rospy.sleep(0.01)

        rospy.loginfo(
            "[HOLDING] Keeping contact for {} seconds...".format(self.duration))
        self.motion.publish(std_msgs.msg.Int16(data=2))
        rospy.sleep(self.duration)

        rospy.loginfo(
            "[RELEASE] Moving to initial configuration {}...".format(
                self.initial_configuration))
        self.motion.publish(std_msgs.msg.Int16(data=3))
        self.finger_controller.publish(
            std_msgs.msg.Float64(data=self.initial_configuration))

        self.start_time = rospy.Time.now()
        while not self.is_position_reached(self.joint_state, self.initial_configuration):
            rospy.sleep(0.01)

        rospy.loginfo(
            "[SIDEWAYS] Moving from {} to {}...".format(
                self.left_configuration, self.right_configuration))
        # Make contact first.
        self.finger_controller.publish(std_msgs.msg.Float64(data=self.mid_configuration))
        self.start_time = rospy.Time.now()
        while not self.is_position_reached(self.joint_state_sideways, self.mid_configuration):
            rospy.sleep(0.01)

        self.motion.publish(std_msgs.msg.Int16(data=4))
        self.finger_controller_sideways.publish(std_msgs.msg.Float64(
            data=self.left_configuration))
        self.start_time = rospy.Time.now()
        while not self.is_position_reached(self.joint_state_sideways, self.left_configuration):
            rospy.sleep(0.01)

        self.finger_controller_sideways.publish(std_msgs.msg.Float64(
            data=self.right_configuration))
        self.start_time = rospy.Time.now()
        while not self.is_position_reached(self.joint_state_sideways, self.right_configuration):
            rospy.sleep(0.01)

        # Go back to the center.
        self.finger_controller_sideways.publish(std_msgs.msg.Float64(
            data=self.center_configuration))
        self.start_time = rospy.Time.now()
        while not self.is_position_reached(self.joint_state_sideways, self.center_configuration):
            rospy.sleep(0.01)

        # Go back to the initial configuration.
        self.finger_controller.publish(
            std_msgs.msg.Float64(data=self.initial_configuration))
        self.start_time = rospy.Time.now()
        while not self.is_position_reached(self.joint_state, self.initial_configuration):
            rospy.sleep(0.01)

        rospy.loginfo("Finished experiment.")
        self.motion.publish(std_msgs.msg.Int16(data=5))

    def is_position_reached(self, current, desired):
        """
        True if the joint position has been reached or a timeout has been exceeded.

        :param current: Current joint position.
        :type current: float64

        :param desired: Target joint position.
        :type desired: float64

        :return: Status of the motion.
        :rtype: bool

        """
        return abs(current - desired) <= self.tolerance \
            or (rospy.Time.now() - self.start_time) >= self.timeout

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.joint_state = None
        self.joint_state_sideways = None


def main():
    rospy.init_node("finger_experiment_node", anonymous=True)
    finger_experiment_node = FingerExperimentNode()
    finger_experiment_node.start()
