#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node controls the fingers of the Shadow hand so that they grasp an object without
exceeding a force threshold. If excessive force is applied on a finger, that finger will
move backwards in order to reduce the grasp force.

**Assumptions:**
  * It assumes five BioTac sensors as the input sensors.
  * It also assumes the joint positions of the fingers to be available.

**Input(s):**

  * `tactile_data`: Contact information of each fingertip (5x).
    - *type:* `udom_perception_msgs/ContactInfo`

  * `joint_states`: Joint position values of all five fingers.
    - *type:* `sensor_msgs/JointState`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_reset`: resets the hand to its initial open configuration.

    - *type:* `std_msgs/String`

**Output(s):**

  * `joint_info`: Joint position value for each finger (5x).
    - *type:* `std_msgs/Float64`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `force_feedback`: Whether to use force feedback or not.

  * `force_threshold_max`: Maximum force, if the current force of any finger exceeds this
    value, then the finger will move backwards (in N).

  * `force_threshold_min`: Minimum force, if the current force of any finger is less than
    this value, then the finger will move forwards (in N).

  * `increment`: The value of the joint position increment to move each finger (in radians).

  * `increment_thumb`:  The value of the joint position increment to move the thumb (in radians).

"""

import yaml
import rospy
import std_msgs.msg
import sensor_msgs.msg
import controller_manager_msgs.srv
from dynamic_reconfigure.server import Server

import udom_perception_msgs.msg
from udom_grasp_control.cfg import ReactiveGraspParamsConfig as ReactiveGraspParams


class ReactiveGrasp(object):
    """
    Controls the fingers of the Shadow hand so that they grasp an object without exceeding
    a force threshold. If excessive force is applied on a finger, that finger will move
    backwards in order to reduce the grasp force.

    """
    def __init__(self):
        """
        Returns a reactive grasp controller.

        :return: Reactive grasp controller
        :rtype: ReactiveGrasp

        """
        # Params
        self.event = None
        self.joint_states = None

        # Whether to use force feedback or not.
        self.force_feedback = rospy.get_param('~force_feedback', False)

        # Maximum force, if the current force of any finger exceeds this value, then
        # the finger will move backwards (in N).
        self.force_threshold_max = rospy.get_param('~force_threshold_max', 1.0)
        # Minimum force, if the current force of any finger is less than this value,
        # then the finger will move forwards (in N).
        self.force_threshold_min = rospy.get_param('~force_threshold_min', 0.1)

        # The value of the joint position increment to move each finger (in radians).
        self.increment = rospy.get_param('~increment', 0.05)
        # The value of the joint position increment to move the thumb (in radians).
        self.increment_thumb = rospy.get_param('~increment_thumb', 0.03)

        # Configuration file describing the limits of each joint.
        self.joint_limits_path = rospy.get_param("~joint_limits", None)
        if self.joint_limits_path is None:
            rospy.logwarn("A configuration file describing the limits of each joints "
                          "has not been specified.")
        else:
            with open(self.joint_limits_path, 'r') as ff:
                self.joint_limits = yaml.load(ff)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.pub_ffj0 = rospy.Publisher(
            self.joint_limits['rh_FFJ0']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_ffj3 = rospy.Publisher(
            self.joint_limits['rh_FFJ3']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_mfj0 = rospy.Publisher(
            self.joint_limits['rh_MFJ0']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_mfj3 = rospy.Publisher(
            self.joint_limits['rh_MFJ3']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_rfj0 = rospy.Publisher(
            self.joint_limits['rh_RFJ0']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_rfj3 = rospy.Publisher(
            self.joint_limits['rh_RFJ3']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_lfj0 = rospy.Publisher(
            self.joint_limits['rh_LFJ0']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_lfj3 = rospy.Publisher(
            self.joint_limits['rh_LFJ3']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_thj2 = rospy.Publisher(
            self.joint_limits['rh_THJ2']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_thj3 = rospy.Publisher(
            self.joint_limits['rh_THJ3']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_thj4 = rospy.Publisher(
            self.joint_limits['rh_THJ4']['topic'], std_msgs.msg.Float64, queue_size=10)
        self.pub_thj5 = rospy.Publisher(
            self.joint_limits['rh_THJ5']['topic'], std_msgs.msg.Float64, queue_size=10)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~tactile_data_ff", udom_perception_msgs.msg.ContactInfo, self.tactile_data_ff_cb)
        rospy.Subscriber(
            "~tactile_data_mf", udom_perception_msgs.msg.ContactInfo, self.tactile_data_mf_cb)
        rospy.Subscriber(
            "~tactile_data_rf", udom_perception_msgs.msg.ContactInfo, self.tactile_data_rf_cb)
        rospy.Subscriber(
            "~tactile_data_lf", udom_perception_msgs.msg.ContactInfo, self.tactile_data_lf_cb)
        rospy.Subscriber(
            "~tactile_data_th", udom_perception_msgs.msg.ContactInfo, self.tactile_data_th_cb)
        rospy.Subscriber(
            "~joint_states", sensor_msgs.msg.JointState, self.joint_states_cb)

        # Services
        controller_to_switch = rospy.get_param(
            '~switch_controller', '/controller_manager/switch_controller')
        rospy.loginfo("Waiting for service '{}'...".format(controller_to_switch))
        rospy.wait_for_service(controller_to_switch)
        self.switch_controller = rospy.ServiceProxy(
            controller_to_switch, controller_manager_msgs.srv.SwitchController)
        rospy.loginfo("Found service '{}'.".format(controller_to_switch))

        # Trajectory controller.
        self.trajectory_controller = rospy.get_param(
            '~trajectory_controller', 'rh_trajectory_controller')

        self.publishers_map = {
            'rh_FFJ0': self.pub_ffj0, 'rh_FFJ3': self.pub_ffj3, 'rh_MFJ0': self.pub_mfj0,
            'rh_MFJ3': self.pub_mfj3, 'rh_RFJ0': self.pub_rfj0, 'rh_RFJ3': self.pub_rfj3,
            'rh_LFJ0': self.pub_lfj0, 'rh_LFJ3': self.pub_lfj3, 'rh_THJ2': self.pub_thj2,
            'rh_THJ3': self.pub_thj3, 'rh_THJ4': self.pub_thj4, 'rh_THJ5': self.pub_thj5}
        self.tactile_data_map = {
             'rh_FFJ3': None, 'rh_MFJ3': None, 'rh_RFJ3': None, 'rh_LFJ3': None, 'rh_THJ5': None}

        # Dynamic Server Reconfiguration
        dynamic_reconfig_srv = Server(ReactiveGraspParams, self.dynamic_reconfig_cb)

    def dynamic_reconfig_cb(self, config, level):
        """
        Reconfigures the collision distance parameter.

        """
        self.force_threshold_max = config.force_threshold_max
        self.force_threshold_min = config.force_threshold_min
        return config

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def tactile_data_ff_cb(self, msg):
        """
        Obtains the tactile data input for the first finger.

        :param msg: Tactile data.
        :type msg: udom_perception_msgs.msg.BiotacStamped

        """
        self.tactile_data_map['rh_FFJ3'] = msg

    def tactile_data_mf_cb(self, msg):
        """
        Obtains the tactile data input for the middle finger.

        :param msg: Tactile data.
        :type msg: udom_perception_msgs.msg.BiotacStamped

        """
        self.tactile_data_map['rh_MFJ3'] = msg

    def tactile_data_rf_cb(self, msg):
        """
        Obtains the tactile data input for the ring finger.

        :param msg: Tactile data.
        :type msg: udom_perception_msgs.msg.BiotacStamped

        """
        self.tactile_data_map['rh_RFJ3'] = msg

    def tactile_data_lf_cb(self, msg):
        """
        Obtains the tactile data input for the little finger.

        :param msg: Tactile data.
        :type msg: udom_perception_msgs.msg.BiotacStamped

        """
        self.tactile_data_map['rh_LFJ3'] = msg

    def tactile_data_th_cb(self, msg):
        """
        Obtains the tactile data input for the thumb.

        :param msg: Tactile data.
        :type msg: udom_perception_msgs.msg.BiotacStamped

        """
        self.tactile_data_map['rh_THJ5'] = msg

    def joint_states_cb(self, msg):
        """
        Obtains the joint states input.

        :param msg: Joint states.
        :type msg: sensor_msgs.msg.JointState

        """
        self.joint_states = msg

    def tear_down(self):
        """

        :return:
        """
        rospy.loginfo("Shutting down reactive grasp node...")
        self.toggle_trajectory_controller('start')
        rospy.loginfo("The reactive grasp node has been successfully terminated.")

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
            elif state == 'HALT':
                state = self.halt_state()
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
            if self.toggle_trajectory_controller('stop'):
                rospy.loginfo("Opening grasp...")
                self.open_grasp()
                rospy.sleep(1.5)
                rospy.loginfo("Going to pre-grasp...")
                self.pregrasp()
                return 'IDLE'
            else:
                rospy.logerr("Unable to stop trajectory controller.")
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
            if not self.toggle_trajectory_controller('start'):
                rospy.logerr("Unable to start trajectory controller.")
            self.reset_component_data()
            return 'HALT'
        elif self.event == 'e_reset':
            self.event_out.publish('e_restarted')
            self.reset_component_data()
            return 'INIT'

        if self.joint_states is not None:
            if not self.force_feedback:
                return 'RUNNING'
            else:
                if None not in self.tactile_data_map.itervalues():
                    return 'RUNNING'
                else:
                    return 'IDLE'
        else:
            return 'IDLE'

    def halt_state(self):
        """
        Executes the HALT state of the state machine.
        It waits until the component is either reset or started again.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            self.event_out.publish('e_started')
            if not self.toggle_trajectory_controller('stop'):
                rospy.logerr("Unable to stop trajectory controller.")
            self.reset_component_data()
            return 'IDLE'
        elif self.event == 'e_reset':
            self.event_out.publish('e_restarted')
            self.reset_component_data()
            return 'INIT'
        else:
            return 'HALT'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event_out.publish('e_stopped')
            if not self.toggle_trajectory_controller('start'):
                rospy.logerr("Unable to start trajectory controller.")
            self.reset_component_data()
            return 'HALT'
        elif self.event == 'e_reset':
            self.event_out.publish('e_restarted')
            self.reset_component_data()
            return 'INIT'

        else:
            for joint in self.joint_limits['control_joints']:
                joint_index = self.joint_states.name.index(joint)
                current_joint_position = self.joint_states.position[joint_index]
                if 'TH' in joint:
                    increment = self.increment_thumb
                else:
                    increment = self.increment
                if self.force_feedback:
                    # Since the tactile sensors can't pull we ignore the negative sign.
                    current_force = abs(self.tactile_data_map[joint].wrenches[0].force.z)
                    force_scale = self.force_control(current_force)
                else:
                    force_scale = 1.0
                if self.within_joint_limits(joint, current_joint_position, (increment * force_scale)):
                    self.publishers_map[joint].publish(
                        current_joint_position + (increment * force_scale))

            self.reset_component_data()
            return 'IDLE'

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.tactile_data_map = {
             'rh_FFJ3': None, 'rh_MFJ3': None, 'rh_RFJ3': None, 'rh_LFJ3': None, 'rh_THJ5': None}
        self.joint_states = None
        self.event = None

    def force_control(self, current):
        """
        Simple force control that returns -1 if the current force exceeds the maximum
        force, 0 if the current force is between the minimum and maximum force and 1 if the
        current force is less than the minimum force.

        :param current:
        :return:
        :rtype: int
        """
        if current > self.force_threshold_max:
            return -1
        elif self.force_threshold_min <= current <= self.force_threshold_max:
            return 0
        else:
            return 1

    def within_joint_limits(self, joint_name, current, increment):
        """
        Verifies whether the increment to the current joint value is within the limits
        of the specified joint name. In case the increment will result in a joint position
        outside the limits, but will make it closer to the limits it returns True.

        :param joint_name: Name of the joint to check whether its within the limits.
        :type joint_name: str

        :param current: Current joint position value.
        :type current: float

        :param increment: Desired joint position increment.
        :type increment: float

        :return: Whether the increment to the current value is within the joint limits.
        :rtype: bool

        """
        max_value = self.joint_limits[joint_name]['max']
        min_value = self.joint_limits[joint_name]['min']
        if min_value <= current + increment <= max_value:
            return True
        if increment < 0.0 and current > min_value:
            return True
        if increment > 0.0 and current > max_value:
            return True
        else:
            return False

    def open_grasp(self):
        """
        Moves the fingers to their initial configuration.

        """
        for joint, value in self.joint_limits.iteritems():
            if joint == 'control_joints':
                continue
            self.publishers_map[joint].publish(value['init'])

    def pregrasp(self):
        """
        Moves the fingers to their initial configuration.

        """
        for joint, value in self.joint_limits.iteritems():
            if joint == 'control_joints':
                continue
            self.publishers_map[joint].publish(value['pregrasp'])

    def toggle_trajectory_controller(self, action):
        """
        Starts or stops the trajectory controller depending on the desired action.

        :param action: To 'start' or 'stop' the controller.
        :type action: str

        :return: True if it was able to toggle the controller.
        :rtype: bool

        """
        if action == 'stop':
            try:
                req = controller_manager_msgs.srv.SwitchControllerRequest()
                req.strictness = req.STRICT
                req.stop_controllers = [self.trajectory_controller]
                res = self.switch_controller(req)
                if res.ok:
                    rospy.loginfo("Successfully stopped trajectory controller.")
                    return True
                else:
                    rospy.logerr("Couldn't stop trajectory controller.")
                    return False
            except rospy.ServiceException, e:
                rospy.logwarn("Service call failed: {}".format(e))
        elif action == 'start':
            try:
                req = controller_manager_msgs.srv.SwitchControllerRequest()
                req.strictness = req.STRICT
                req.start_controllers = [self.trajectory_controller]
                res = self.switch_controller(req)
                if res.ok:
                    rospy.loginfo("Successfully started trajectory controller.")
                    return True
                else:
                    rospy.logerr("Couldn't start trajectory controller.")
                    return False
            except rospy.ServiceException, e:
                rospy.logwarn("Service call failed: {}".format(e))
        else:
            return False


def main():
    rospy.init_node("reactive_grasp", anonymous=True)
    reactive_grasp = ReactiveGrasp()
    rospy.on_shutdown(reactive_grasp.tear_down)
    reactive_grasp.start()
