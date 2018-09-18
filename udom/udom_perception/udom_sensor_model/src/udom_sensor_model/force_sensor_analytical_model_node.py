#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node instantiates a force sensor model, based on identification of
inertial parameters[1], that predicts the contact wrench at the sensor
frame by first estimating its non-contact forces using data from joint encoders
and then subtracting those non-contact forces from the sensor's output wrench.

[1] Kubus, Daniel, Torsten Kroger, and Friedrich M. Wahl. "On-line rigid object
recognition and pose estimation based on inertial parameters." Intelligent Robots
and Systems, 2007. IROS 2007.

**Input(s):**

  * `robot_data`: The data from encoders and IMUs used to predict a wrench at the robot's end effector.
    - *type:* `std_msgs/Float64MultiArray`

  * `wrench_in`: The sensor's output wrench.
    - *type:* `geometry_msgs/WrenchStamped`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

    - *type:* `std_msgs/String`

**Output(s):**

  * `force_array`: A representation of the wrench data that specifies the contact locations
    and magnitudes for (possibly) multiple forces.
    - *type:* `udom_common_msgs/ForceMultiArray`

  * `wrench_out`: The estimated wrench at the sensor's frame.
    - *type:* `geometry_msgs/WrenchStamped`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

"""

import os
import collections

import numpy as np

import rospy
import std_msgs.msg
import geometry_msgs.msg
import udom_common_msgs.msg

import identification_utils as id_utils


class ForceSensorModel(object):
    """
    Estimates the wrench based on the given robot data.

    """
    def __init__(self):
        """
        Returns a force sensor model based on the identified inertial parameters.

        :return: Force sensor model.
        :rtype: ForceSensorModel

        """
        # Params
        self.event = None
        self.robot_data = None
        self.wrench_data = None

        # Config (relative) path.
        self.config_path = 'config/force_sensor_model/identification_based/'

        # Number of samples to delay the actual wrench such that it matches the delayed model's estimation.
        self.delay_actual = rospy.get_param('~delay_actual', 1400)
        # Number of samples to delay the estimated wrench such that it matches the actual wrench.
        self.delay_predicted = rospy.get_param('~delay_predicted', 1)

        # Inertial parameters.
        self.inertial_parameters = np.load(
            os.path.join(
                os.path.dirname(__file__), self.config_path + 'inertial_parameters.npy'))

        # Transform between the end-effector and the sensor.
        transform = rospy.get_param('~ee_to_sensor_transform', 'transform_ee_s')
        self.ee_to_sensor_transform = np.array(rospy.get_param('~' + transform))

        # Gravity vector based on the robot's base frame.
        g_vector = rospy.get_param('~gravity_vector', 'gravity_base')
        self.gravity_vector = geometry_msgs.msg.Vector3(*rospy.get_param('~' + g_vector))

        # Initial gravity matrix (to compensate for zeroing the sensor).
        self.v_g_init = id_utils.compute_v_g_init(self.gravity_vector)

        # Inputs to the model.
        self.orientation_in = geometry_msgs.msg.Quaternion()
        self.twist_in = geometry_msgs.msg.Twist()
        # Used to compute the numerical differentiation.
        self.twist_queue = collections.deque([], 2)

        # Store 'delay_actual' timesteps of the actual wrench to synchronize it with the model's estimation.
        self.wrench_actual_queue = collections.deque([], self.delay_actual)
        # Store 'delay_predicted' timesteps of the estimated wrench to synchronize it with the actual wrench.
        self.wrench_estimated_queue = collections.deque([], self.delay_predicted)

        # Used to apply an Exponential Moving Average filter.
        self.filtered_wrench = collections.deque([], 2)

        # Filter coefficient for the estimated signal.
        self.alpha = 0.0095

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 500))

        # To compute the time difference between to time steps.
        self.dt = rospy.Time.now()
        self.previous_time = rospy.Time.now()
        self.now = rospy.Time.now()

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.wrench_out = rospy.Publisher(
            "~wrench_out", geometry_msgs.msg.WrenchStamped, queue_size=1, tcp_nodelay=True)
        self.force_array = rospy.Publisher(
            "~force_array", udom_common_msgs.msg.ForceMultiArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~robot_data", std_msgs.msg.Float64MultiArray, self.robot_data_cb)
        rospy.Subscriber(
            "~wrench_in", geometry_msgs.msg.WrenchStamped, self.wrench_in_cb)

        # Mapping robot data to the specific features.
        self.inputs_mapping = {
            'accel_angular_x': 17,
            'accel_angular_y': 18,
            'accel_angular_z': 19,
            'accel_linear_x': 14,
            'accel_linear_y': 15,
            'accel_linear_z': 16,
            'force_angular_x': 23,
            'force_angular_y': 24,
            'force_angular_z': 25,
            'force_linear_x': 20,
            'force_linear_y': 21,
            'force_linear_z': 22,
            'joint_pos_0': 26,
            'joint_pos_1': 27,
            'joint_pos_2': 28,
            'joint_pos_3': 29,
            'joint_pos_4': 30,
            'joint_pos_5': 31,
            'joint_pos_6': 32,
            'joint_torque_0': 40,
            'joint_torque_1': 41,
            'joint_torque_2': 42,
            'joint_torque_3': 43,
            'joint_torque_4': 44,
            'joint_torque_5': 45,
            'joint_torque_6': 46,
            'joint_vel_0': 33,
            'joint_vel_1': 34,
            'joint_vel_2': 35,
            'joint_vel_3': 36,
            'joint_vel_4': 37,
            'joint_vel_5': 38,
            'joint_vel_6': 39,
            'orientation_w': 7,
            'orientation_x': 4,
            'orientation_y': 5,
            'orientation_z': 6,
            'position_x': 1,
            'position_y': 2,
            'position_z': 3,
            'timestep': 0,
            'twist_angular_x': 11,
            'twist_angular_y': 12,
            'twist_angular_z': 13,
            'twist_linear_x': 8,
            'twist_linear_y': 9,
            'twist_linear_z': 10}

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def robot_data_cb(self, msg):
        """
        Obtains the robot data input.

        :param msg: Robot data.
        :type msg: std_msgs.msg.Float64MultiArray

        """
        self.robot_data = msg
        self.orientation_in.w = msg.data[self.inputs_mapping['orientation_w']]
        self.orientation_in.x = msg.data[self.inputs_mapping['orientation_x']]
        self.orientation_in.y = msg.data[self.inputs_mapping['orientation_y']]
        self.orientation_in.z = msg.data[self.inputs_mapping['orientation_z']]

        self.twist_in.linear.x = msg.data[self.inputs_mapping['twist_linear_x']]
        self.twist_in.linear.y = msg.data[self.inputs_mapping['twist_linear_y']]
        self.twist_in.linear.z = msg.data[self.inputs_mapping['twist_linear_z']]
        self.twist_in.angular.x = msg.data[self.inputs_mapping['twist_linear_x']]
        self.twist_in.angular.y = msg.data[self.inputs_mapping['twist_linear_y']]
        self.twist_in.angular.z = msg.data[self.inputs_mapping['twist_linear_z']]

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
            self.previous_time = rospy.Time.now()
            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            rospy.logdebug("event: {0}".format(self.event))
            self.loop_rate.sleep()
            self.dt = rospy.Time.now() - self.previous_time

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
            self.reset_queues()
            self.reset_component_data()
            return 'INIT'
        elif self.robot_data is not None and self.wrench_data is not None:
            self.wrench_actual_queue.append(self.wrench_data.wrench)
            while len(self.wrench_actual_queue) < self.delay_actual:
                self.wrench_actual_queue.append(self.wrench_data.wrench)
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
            self.reset_queues()
            self.reset_component_data()
            return 'INIT'
        else:
            force_array, wrench_out = self.compute_contact_info(
                self.orientation_in, self.twist_in)
            if force_array:
                self.event_out.publish('e_running')
                self.wrench_out.publish(wrench_out)
                self.force_array.publish(force_array)
            else:
                rospy.logwarn("Error while computing contact information.")
            self.reset_component_data()
            return 'IDLE'

    def compute_contact_info(self, orientation, twist):
        """
        Computes the contact information based on the robot data and the specified sensor model.

        :param orientation: Rotation of the end-effector with respect to the base frame.
        :type orientation: geometry_msgs.msg.Quaternion

        :param twist: End-effector twist expressed with respect to the base frame.
        :type twist: geometry_msgs.msg.Twist

        :return: The computed contact information and the estimated wrench.
        :rtype: udom_common_msgs.msg.ForceMultiArray, geometry_msgs.msg.WrenchStamped

        """
        force_array = udom_common_msgs.msg.ForceMultiArray()
        forces = udom_common_msgs.msg.ForceArray()
        force_array.forces = [forces]
        wrench_stamped = geometry_msgs.msg.WrenchStamped()
        wrench = geometry_msgs.msg.Wrench()

        # Rotate gravity vector to be expressed on the sensor frame.
        gravity_sensor = id_utils.rotate_gravity(self.gravity_vector, orientation)

        # Transform twist to be expressed on the sensor frame.
        twist_sensor = id_utils.transform_twist(twist, self.ee_to_sensor_transform)

        # Compute acceleration by differentiating the twist.
        self.twist_queue.append(twist_sensor)
        acceleration = self.differentiate_twist(twist_sensor, self.twist_queue[0])

        # Compute V matrix.
        v_matrix = id_utils.compute_v_matrix(
            acceleration, twist_sensor.angular, gravity_sensor)

        # Compute non-contact wrench.
        y = np.dot(v_matrix + self.v_g_init, self.inertial_parameters)

        wrench.force.x = y[0]
        wrench.force.y = y[1]
        wrench.force.z = y[2]
        wrench.torque.x = y[3]
        wrench.torque.y = y[4]
        wrench.torque.z = y[5]

        self.wrench_estimated_queue.append(wrench)
        estimated_wrench = self.filter_wrench(self.wrench_estimated_queue[0], alpha=self.alpha)

        # This is a hack, since we don't guarantee that we use a 'delay' until the queue
        # has been filled.
        delayed_wrench = self.wrench_actual_queue[0]
        wrench_difference = subtract_wrenches(delayed_wrench, estimated_wrench)

        # ToDo: estimate contact location
        point = geometry_msgs.msg.Point()

        force_array.forces[0].wrenches = [wrench_difference]
        force_array.forces[0].positions = [point]
        force_array.forces[0].header.stamp = rospy.Time.now()
        force_array.forces[0].header.frame_id = self.wrench_data.header.frame_id

        wrench_stamped.header.stamp = rospy.Time.now()
        wrench_stamped.header.frame_id = self.wrench_data.header.frame_id
        wrench_stamped.wrench = wrench_difference

        return force_array, wrench_stamped

    def filter_wrench(self, wrench_in, alpha=0.01):
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
            wrench_out = wrench_in
        self.filtered_wrench.append(wrench_out)
        return wrench_out

    def differentiate_twist(self, twist, twist_previous, min_dt=1e-3):
        """
        Computes a numerical difference of a twist, based on the current and previous one,
        to estimate the linear and angular acceleration.

        :param twist: The current twist.
        :type twist: geometry_msgs.msg.Twist

        :param twist_previous: The previous twist.
        :type twist_previous: geometry_msgs.msg.Twist

        :param min_dt: Minimum time step to avoid near zero division (in seconds).
        :type min_dt: float

        :return: Acceleration.
        :rtype: geometry_msgs.msg.Accel

        """
        acceleration = geometry_msgs.msg.Accel()
        dt = max(self.dt.to_sec(), min_dt)

        acceleration.linear.x = np.gradient(
            [twist_previous.linear.x, twist.linear.x], dt)[-1]
        acceleration.linear.y = np.gradient(
            [twist_previous.linear.y, twist.linear.y], dt)[-1]
        acceleration.linear.z = np.gradient(
            [twist_previous.linear.z, twist.linear.z], dt)[-1]

        acceleration.angular.x = np.gradient(
            [twist_previous.angular.x, twist.angular.x], dt)[-1]
        acceleration.angular.z = np.gradient(
            [twist_previous.angular.y, twist.angular.y], dt)[-1]
        acceleration.angular.z = np.gradient(
            [twist_previous.angular.z, twist.angular.z], dt)[-1]

        return acceleration

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.robot_data = None
        self.wrench_data = None
        self.event = None

    def reset_queues(self):
        """
        Clears the queues of the component.

        """
        self.wrench_actual_queue.clear()
        self.wrench_estimated_queue.clear()


def subtract_wrenches(wrench_1, wrench_2):
    """
    Returns the difference between two wrenches. Both wrenches are assumed to have
    the same reference frame.

    :param wrench_1: The wrench from which wrench_2 will be subtracted.
    :type wrench_1: geometry_msgs.msg.Wrench

    :param wrench_2: The wrench which will be subtracted from wrench_1.
    :type wrench_2: geometry_msgs.msg.Wrench

    :return: The difference between wrench_1 and wrench_2
    :rtype: geometry_msgs.msg.Wrench

    """
    wrench_out = geometry_msgs.msg.Wrench()

    wrench_out.force.x = wrench_1.force.x - wrench_2.force.x
    wrench_out.force.y = wrench_1.force.y - wrench_2.force.y
    wrench_out.force.z = wrench_1.force.z - wrench_2.force.z

    wrench_out.torque.x = wrench_1.torque.x - wrench_2.torque.x
    wrench_out.torque.y = wrench_1.torque.y - wrench_2.torque.y
    wrench_out.torque.z = wrench_1.torque.z - wrench_2.torque.z

    return wrench_out


def main():
    rospy.init_node("force_sensor_model", anonymous=True)
    force_sensor_model = ForceSensorModel()
    force_sensor_model.start()
