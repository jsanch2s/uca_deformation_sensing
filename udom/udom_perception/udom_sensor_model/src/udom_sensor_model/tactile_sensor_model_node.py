#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node instantiates a tactile sensor model in order to map the readings of the
sensor to contact information.

**Input(s):**
  * `tactile_data`: The tactile sensor's output.
    - *type:* `udom_perception_msgs/BiotacStamped`
  * `event_in`: The desired event for the node:
      `e_start`: starts the component.
      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `contact_info`: A representation of the tactile data that specifies the contact
      locations and magnitudes.
    - *type:* `udom_perception_msgs/ContactInfo`
  * `event_out`: The current event of the node.
      `e_running`: when the component is running.
      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `max_intensity_value`: Maximum value that an electrode might reach.
  * `intensity_threshold`: Threshold used to decide whether or not an electrode is active.
      Since the electrode value decreases the more its surface is pressed, a negative value
      thus, means higher intensity. The values above this threshold are then removed.
  * `gaussian`: If True, it uses a Gaussian distribution to find the electrodes' centers.
      Otherwise, it assumes the center as specified in the electrodes config file.
  * `inverted`: If False, it assumes that the intensity value of each electrode is
      proportional to the Gaussian distribution's width. Otherwise, the width is inversely
      proportional to the intensity.
  * `calibration_samples`: Number of samples to calculate the resting impedance values,
      these values are then used to subtract them from the current impedance values
      to obtain the impedance changes.
  * `sequence_length`: Number of samples representing the sequence length that was
      used to learn the model.

"""

import os
import collections
import csv

import numpy as np
import tflearn
import yaml

import rospy
import std_msgs.msg
import geometry_msgs.msg
import udom_perception_msgs.msg

import udom_sensor_model.tactile_utils as tactile_utils


class TactileSensorModel(object):
    """
    Subscribes to a specific output of a tactile sensor and publishes a
    udom_perception_msgs.ContactInfo message.

    """
    def __init__(self):
        """
        Returns a BioTac sensor model learned with Recurrent Neural Networks.

        :return: BioTac sensor model.
        :rtype: TactileSensorModel

        """
        # Params
        self.event = None
        self.tactile_data = None
        self.calibrated = False

        # Both of these values, max_intensity_value and intensity_threshold, assume
        # the resting value from the electrodes has already been subtracted.
        # Maximum value that an electrode might reach.
        self.max_intensity_value = rospy.get_param('~max_intensity_value', 200)
        # Threshold used to decide whether or not an electrode is active.
        # Since the BioTac outputs a lower value the more its surface is pressed,
        # here a negative value means more intensity. Thus the values above this
        # threshold would be cutoff.
        self.intensity_threshold = rospy.get_param('~intensity_threshold', 0)

        # Both of these values, gaussian and inverted, are used for the contact
        # localization algorithm.
        # If True, it uses a Gaussian distribution to find the electrodes' centers.
        # Otherwise, it assumes the center as specified in the electrodes config file.
        self.gaussian = rospy.get_param('~gaussian', True)
        # If False, it assumes that the intensity value of each electrode is proportional
        # to the Gaussian distribution's width. Otherwise, the width is inversely
        # proportional to the intensity.
        self.inverted = rospy.get_param('~inverted', False)

        # Input features for the RNN model.
        self.input_features = rospy.get_param('~input_features', 21)

        # Output features for the RNN model.
        self.output_features = rospy.get_param('~output_features', 4)

        # Number of samples representing the sequence length that was used to
        # learn the model.
        self.sequence_length = rospy.get_param('~sequence_length', 50)

        # Number of hidden units for the hidden layers.
        self.hidden_units = rospy.get_param('~hidden_units', 20)

        # Path where the model for the RNN has been saved.
        self.model_path = rospy.get_param('~model_path', 'config/learned_rnn_model')

        # Build and load the neural network.
        self.model = self.build_network()
        self.model.load(
            os.path.join(os.path.dirname(__file__), self.model_path + "/biotac.tfl"))

        # File with the labels, positions and normals of the electrodes.
        self.electrodes_config_file = rospy.get_param("~electrodes_config_file", None)
        assert self.electrodes_config_file is not None, \
            "A configuration file of the electrodes must be specified."

        # Set up the electrodes' information (e.g. their labels, positions and normals)
        with open(self.electrodes_config_file, 'r') as f:
            data_iter = csv.DictReader(f, delimiter=' ')
            self.electrodes_info = [data for data in data_iter]
        self.electrodes = [
            tactile_utils.Electrode(
                ee['electrode'],
                position=(float(ee['pos_x']) / 1000, float(ee['pos_y']) / 1000, float(ee['pos_z']) / 1000),
                normal=(float(ee['normal_x']), float(ee['normal_y']), float(ee['normal_z'])))
            for ee in self.electrodes_info if ee['electrode'].startswith('E')]

        # Mapping relating the areas, of each finger, to their centroid in X, Y and Z.
        self.config_file = rospy.get_param("~config_file", None)
        assert self.config_file is not None, "A configuration file must be specified."
        self.finger_locations = yaml.load(open(self.config_file, "r"))['finger_locations']

        # Number of samples to calculate the resting impedance values, these values
        # are then used to subtract them from the current impedance values to obtain
        # the impedance changes.
        self.calibration_samples = rospy.get_param('~calibration_samples', 1)

        # Average tactile values (e.g. impedance and pressure).
        self.average_tactile_values = None

        # Tactile values (e.g. impedance and pressure) sequence.
        self.sequence = collections.deque([], self.sequence_length)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.contact_info = rospy.Publisher(
            "~contact_info", udom_perception_msgs.msg.ContactInfo, queue_size=1,
            tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~tactile_data", udom_perception_msgs.msg.BiotacStamped, self.tactile_data_cb)

    def build_network(self):
        """
        Builds a recurrent neural network model.

        :return: Neural network model.
        :rtype: tflearn.DNN

        """
        input_layer = tflearn.input_data(
            shape=[None, self.sequence_length, self.input_features], name='input_layer')
        layer_1 = tflearn.lstm(
            input_layer, self.hidden_units, return_seq=True, name='layer1')
        layer_2 = tflearn.lstm(layer_1, self.hidden_units, name='layer2')
        output_layer = tflearn.fully_connected(
            layer_2, self.output_features, name='output_layer')

        regression_layer = tflearn.regression(
            output_layer, optimizer='sgd', loss='mean_square',
            metric='R2', learning_rate=0.01)

        return tflearn.DNN(
            regression_layer, tensorboard_verbose=2,
            checkpoint_path='{}/biotac.tfl.ckpt'.format(
                os.path.join(os.path.dirname(__file__), self.model_path)))

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def tactile_data_cb(self, msg):
        """
        Obtains the tactile data input.

        :param msg: Tactile data.
        :type msg: udom_perception_msgs.msg.BiotacStamped

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
            elif state == 'CALIBRATION':
                state = self.calibration_state()
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
            self.reset_component_data()
            self.calibrated = False
            self.average_tactile_values = None
            return 'INIT'
        if self.tactile_data is not None and not self.calibrated:
            return 'CALIBRATION'
        if self.tactile_data is not None and self.calibrated:
            try:
                self.sequence.popleft()
            except IndexError:
                pass
            while len(self.sequence) < self.sequence_length:
                self.sequence.append(self.get_tactile_values() - self.average_tactile_values)
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
            self.calibrated = False
            self.average_tactile_values = None
            return 'INIT'

        if self.average_tactile_values is None:
            samples = collections.deque([], self.calibration_samples)
            while len(samples) < self.calibration_samples:
                samples.append(self.get_tactile_values())
            self.average_tactile_values = np.mean(samples, axis=0)
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
            self.calibrated = False
            self.average_tactile_values = None
            return 'INIT'
        else:
            # Adding an extra dimension to make it compatible with the learned model.
            tactile_data = np.array(self.sequence).reshape(
                (1, self.sequence_length, self.input_features))
            contact_info = self.compute_contact_info(
                tactile_data, self.tactile_data.header.frame_id)
            if contact_info:
                self.event_out.publish('e_running')
                self.contact_info.publish(contact_info)
            else:
                rospy.logwarn("Error while computing contact information.")
            self.reset_component_data()
            return 'IDLE'

    def compute_contact_info(self, tactile_values, frame_id):
        """
        Computes the contact information based on the tactile data and the specified
        sensor model.

        :param tactile_values: The tactile data sequence (e.g. impedance and pressure values
        of the required sequence length).
        :type tactile_values: np.array of shape (1, sequence_length, input_features)

        :param frame_id: Frame id of the tactile sensor.
        :type frame_id: str

        :return: The computed contact information.
        :rtype: udom_perception_msgs.msg.ContactInfo

        """
        contact_info = udom_perception_msgs.msg.ContactInfo()
        y = self.model.predict(tactile_values)
        wrench = geometry_msgs.msg.Wrench()
        wrench.force.x = y[0][0]
        wrench.force.y = y[0][1]
        wrench.force.z = y[0][2]
        # ToDo: Add torque info

        point = geometry_msgs.msg.Point()
        # Use the most recent value of the electrodes.
        electrodes_values = tactile_values[:, -1, :len(self.electrodes)].flatten()
        active_electrodes = []
        for ee, vv in zip(self.electrodes, electrodes_values):
            if vv < self.intensity_threshold:
                ee.set_intensity(abs(vv / float(self.max_intensity_value)))
                active_electrodes.append(ee)

        if active_electrodes:
            location = tactile_utils.locate_contact(
                active_electrodes, gaussian=self.gaussian, inverted=self.inverted)
            point.x = location[0]
            point.y = location[1]
            point.z = location[2]

        contact_info.wrenches = [wrench]
        contact_info.contact_points = [point]
        contact_info.header.stamp = rospy.Time.now()
        contact_info.header.frame_id = frame_id

        return contact_info

    def get_tactile_values(self):
        """
        Gets the required tactile values in a list.

        :return: Required tactile values (e.g. electrodes, pressure).
        :rtype: np.array

        """
        tactile_values = list(self.tactile_data.electrodes)
        tactile_values.append(self.tactile_data.pac0)
        tactile_values.append(self.tactile_data.pdc)
        return np.array(tactile_values)

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.tactile_data = None
        self.event = None


def main():
    rospy.init_node("sensor_model", anonymous=True)

    tactile_sensor_model = TactileSensorModel()
    tactile_sensor_model.start()
