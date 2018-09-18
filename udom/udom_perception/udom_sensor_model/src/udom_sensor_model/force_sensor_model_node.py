#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node instantiates a force sensor model that predicts the contact wrench at the
sensor's frame by first estimating its non-contact forces using data from joint encoders
and inertial measurement units (IMUs), e.g. pose, twist and accelerations of the robot;
and then subtracting those non-contact forces from the sensor's output wrench.

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

  * `sequence_length`: Number of samples representing the sequence length that was used to learn the model.

  * `model_name`: Name of the model for the trained RNN.

"""

import os
import copy
import collections

import numpy as np
import tflearn

import rospy
import std_msgs.msg
import geometry_msgs.msg
import udom_common_msgs.msg


class ForceSensorModel(object):
    """
    Estimates the wrench based on the given robot data.

    """
    def __init__(self):
        """
        Returns a force sensor model learned with Recurrent Neural Networks.

        :return: Force sensor model.
        :rtype: ForceSensorModel

        """
        # Params
        self.event = None
        self.robot_data = None
        self.wrench_data = None
        self.zero_wrench = geometry_msgs.msg.Wrench()

        # Number of samples representing the sequence length that was used to learn the model.
        self.sequence_length = rospy.get_param('~sequence_length', 20)

        # Number of samples to delay the actual wrench such that it matches the delayed model's estimation.
        self.delay_actual = rospy.get_param('~delay_actual', 50)
        # Number of samples to delay the estimated wrench such that it matches the actual wrench.
        self.delay_predicted = rospy.get_param('~delay_predicted', 1)

        # Number of samples to filter the estimated non-contact wrench.
        self.filter_samples = rospy.get_param('~filter_samples', 50)

        # Name of the model for the trained RNN.
        model_name = rospy.get_param('~model_name', 'linear_accel_orientation_twist')
        self.features_in = model_name

        # Input features for the RNN model.
        self.input_features = rospy.get_param('~input_features', 13)

        # Output features for the RNN model.
        self.output_features = rospy.get_param('~output_features', 6)

        # Number of hidden units for each hidden layer.
        self.hidden_units = rospy.get_param('~hidden_units', [15, 10])

        # Relative path (ending with a slash '/') where the model for the RNN has been saved.
        self.model_path = rospy.get_param(
            '~model_path', 'config/force_sensor_model/rnn_based/')

        self.model_name = model_name + '_' + str(self.sequence_length)

        # Build and load the neural network.
        self.model = self.build_network()
        self.model.load(
            os.path.join(os.path.dirname(__file__), self.model_path + self.model_name + '/' +
                         self.model_name + ".tfl"))

        # Robot data values (e.g. twist, pose) sequence.
        self.sequence = collections.deque([], self.sequence_length)

        # Store 'delay_actual' timesteps of the actual wrench to synchronize it with the model's estimation.
        self.wrench_actual_queue = collections.deque([], self.delay_actual)
        # Store 'delay_predicted' timesteps of the estimated wrench to synchronize it with the actual wrench.
        self.wrench_estimated_queue = collections.deque([], self.delay_predicted)

        # Used to apply an Exponential Moving Average filter.
        # self.filtered_wrench = collections.deque([], 2)
        # Used to apply a Moving Average filter.
        self.filtered_wrench = collections.deque([], self.filter_samples)

        # Filter coefficient for the estimated signal.
        self.alpha = rospy.get_param('~alpha', 0.02)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 500))

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

        # Mapping a set of features to the specific features.
        self.features_mapping = {
            'linear_accel_orientation_twist': [
                'accel_linear_x', 'accel_linear_y', 'accel_linear_z', 'orientation_x', 'orientation_y',
                'orientation_z', 'orientation_w', 'twist_linear_x', 'twist_linear_y', 'twist_linear_z',
                'twist_angular_x', 'twist_angular_y', 'twist_angular_z'],
            'accel': [
                'accel_linear_x', 'accel_linear_y', 'accel_linear_z', 'accel_angular_x',
                'accel_angular_y', 'accel_angular_z'],
            'orientation_twist': [
                'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w', 'twist_linear_x',
                'twist_linear_y', 'twist_linear_z', 'twist_angular_x', 'twist_angular_y',
                'twist_angular_z']}

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

    def build_network(self):
        """
        Builds a recurrent neural network model.

        :return: Neural network model.
        :rtype: tflearn.DNN

        """
        input_layer = tflearn.input_data(
            shape=[None, self.sequence_length, self.input_features], name='input_layer')
        layer_1 = tflearn.lstm(
            input_layer, self.hidden_units[0], return_seq=True, activation='tanh', name='layer1')
        layer_2 = tflearn.lstm(
            layer_1, self.hidden_units[1], return_seq=False, activation='tanh', name='layer2')
        output_layer = tflearn.fully_connected(
            layer_2, self.output_features, activation='linear', name='output_layer')

        regression_layer = tflearn.regression(
            output_layer, optimizer='sgd', loss='mean_square', metric='R2', learning_rate=0.01)

        return tflearn.DNN(
            regression_layer, tensorboard_verbose=0,
            checkpoint_path='{}/{}.tfl.ckpt'.format(os.path.join(
                os.path.dirname(__file__), self.model_path + self.model_name + '/'), self.model_name))

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
i
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
            self.sequence.append(self.get_robot_values())
            self.wrench_actual_queue.append(self.wrench_data.wrench)
            while (len(self.sequence) < self.sequence_length) or \
                    (len(self.wrench_actual_queue) < self.delay_actual):
                self.sequence.append(self.get_robot_values())
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
            # Adding an extra dimension to make it compatible with the learned model.
            robot_data = np.array(self.sequence).reshape(
                (1, self.sequence_length, self.input_features))
            force_array, wrench_out = self.compute_contact_info(robot_data)
            if force_array:
                self.event_out.publish('e_running')
                self.wrench_out.publish(wrench_out)
                self.force_array.publish(force_array)
            else:
                rospy.logwarn("Error while computing contact information.")
            self.reset_component_data()
            return 'IDLE'

    def compute_contact_info(self, robot_data):
        """
        Computes the contact information based on the robot data and the specified sensor model.

        :param robot_data: The robot data sequence (e.g. pose, twist of the required sequence length).
        :type robot_data: np.array of shape (1, sequence_length, input_features)

        :return: The computed contact information and the estimated wrench.
        :rtype: udom_common_msgs.msg.ForceMultiArray, geometry_msgs.msg.WrenchStamped

        """
        force_array = udom_common_msgs.msg.ForceMultiArray()
        forces = udom_common_msgs.msg.ForceArray()
        force_array.forces = [forces]
        y = self.model.predict(robot_data)
        wrench_stamped = geometry_msgs.msg.WrenchStamped()
        non_contact_wrench = geometry_msgs.msg.Wrench()

        non_contact_wrench.force.x = y[0][0]
        non_contact_wrench.force.y = y[0][1]
        non_contact_wrench.force.z = y[0][2]
        non_contact_wrench.torque.x = y[0][3]
        non_contact_wrench.torque.y = y[0][4]
        non_contact_wrench.torque.z = y[0][5]

        self.wrench_estimated_queue.append(non_contact_wrench)
        non_contact_wrench = self.wrench_estimated_queue[0]

        # This is a hack, since we don't guarantee that we use a 'delay' until the queue
        # has been filled.
        delayed_wrench = self.wrench_actual_queue[0]
        wrench_difference = subtract_wrenches(delayed_wrench, non_contact_wrench)

        estimated_wrench = self.filter_wrench_ma(wrench_difference)
        # estimated_wrench = self.filter_wrench(self.wrench_estimated_queue[0], alpha=self.alpha)

        if self.event == 'e_zero':
            self.zero_wrench = copy.deepcopy(estimated_wrench)

        estimated_wrench = subtract_wrenches(estimated_wrench, self.zero_wrench)

        # ToDo: estimate contact location
        point = geometry_msgs.msg.Point()

        force_array.forces[0].wrenches = [estimated_wrench]
        force_array.forces[0].positions = [point]
        force_array.forces[0].header.stamp = rospy.Time.now()
        force_array.forces[0].header.frame_id = self.wrench_data.header.frame_id

        wrench_stamped.header.stamp = rospy.Time.now()
        wrench_stamped.header.frame_id = self.wrench_data.header.frame_id
        wrench_stamped.wrench = estimated_wrench

        return force_array, wrench_stamped

    def filter_wrench_ma(self, wrench_in):
        """
        Filters a wrench using the moving average filter.

        :param wrench_in: The wrench to be filtered.
        :type wrench_in: geometry_msgs.msg.WrenchStamped

        :return: The filtered wrench.
        :rtype: geometry_msgs.msg.WrenchStamped

        """
        self.filtered_wrench.append(wrench_in)

        wrench_out = geometry_msgs.msg.Wrench()
        wrench_out.force.x = np.mean([wrench.force.x for wrench in self.filtered_wrench])
        wrench_out.force.y = np.mean([wrench.force.y for wrench in self.filtered_wrench])
        wrench_out.force.z = np.mean([wrench.force.z for wrench in self.filtered_wrench])
        wrench_out.torque.x = np.mean([wrench.torque.x for wrench in self.filtered_wrench])
        wrench_out.torque.y = np.mean([wrench.torque.y for wrench in self.filtered_wrench])
        wrench_out.torque.z = np.mean([wrench.torque.z for wrench in self.filtered_wrench])

        return wrench_out

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

    def get_robot_values(self):
        """
        Gets the robot values based on the specified features.

        :return: Specified robot values (e.g. acceleration, pose).
        :rtype: np.array

        """
        idx = [self.inputs_mapping[ff] for ff in self.features_mapping[self.features_in]]
        idx.sort()
        return np.array([self.robot_data.data[ii] for ii in idx])

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
        self.sequence.clear()
        self.wrench_actual_queue.clear()
        self.wrench_estimated_queue.clear()
        self.zero_wrench = geometry_msgs.msg.Wrench()


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
