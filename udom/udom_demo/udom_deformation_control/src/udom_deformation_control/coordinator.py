#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node uses a pipeline of components to perform deformation control using a
a KUKA arm and a force/torque sensor. The component serves as a configurator/coordinator,
i.e. it sets the required parameters for all the components and starts/stops them accordingly.

It uses the following nodes:

  * `udom_sensor_model/force_sensor_model`
  * `udom_geometric_transformation/force_transformer`
  * `udom_geometric_transformation/nodal_force_calculator`
  * `udom_deformation_modeling/deformation_model`
  * `udom_geometric_transformation/pose_extractor`
  * `udom_pose_control/pose_controller`

**Assumptions:**

  * It assumes a force/torque sensor as input sensor.

**Input(s):**

  * `wrench_in`: The output data of the force sensor.
    - *type:* `geometry_msgs/WrenchStamped`
  * `robot_data`: The data from encoders and IMUs used to predict a wrench at the robot's end effector.
    - *type:* `std_msgs/Float64MultiArray`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_reset`: resets the mesh to its original undeformed state.

    - *type:* `std_msgs/String`

**Output(s):**

  * `mesh`: A mesh representation of the object with the updated nodes' position based
        on  the deformation.
    - *type:* `udom_modeling_msgs/Mesh`
  * `twist`: Twist command to the robot controller.
    - *type:* `geometry_msgs/TwistStamped`
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).
  * `mesh_filename`: Filename of the volumetric mesh in a .veg format. **Note:** This file
        should be located in the config directory of the `deformation_sensing` package.
  * `object_frame`: Reference frame of the object.
  * `twist_frame`: The twist will be described with respect to this frame (e.g. end-effector frame).
  * `pose_nodes`: Nodes used to extract a pose from the mesh.
  * `constrained_nodes`: Constrained vertices of the mesh. Each constrained node must
        specify its three degrees of freedom. E.g., to constrain vertices 4, 10 and 14 the
        constrained_nodes should be [12, 13, 14, 30, 31, 32, 42, 43, 44].

"""

import rospy
import std_msgs.msg


class Coordinator(object):
    """
    Coordinates a set of components to control the deformation of an object.

    """
    def __init__(self):
        """
        Instantiates a node to coordinate the components of the deformation control pipeline.

        """
        # Params
        self.started_components = False
        self.event = None

        self.sensor_model_status = None
        self.pose_extractor_status = None
        self.pose_controller_status = None
        self.force_transformer_status = None
        self.nodal_force_calculator_status = None
        self.deformation_model_status = None

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 500))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)

        self.start_sensor_model = rospy.Publisher(
            "~start_sensor_model", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_pose_extractor = rospy.Publisher(
            "~start_pose_extractor", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_pose_controller = rospy.Publisher(
            "~start_pose_controller", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_force_transformer = rospy.Publisher(
            "~start_force_transformer", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_nodal_force_calculator = rospy.Publisher(
            "~start_nodal_force_calculator", std_msgs.msg.String, queue_size=10, latch=True)
        self.start_deformation_model = rospy.Publisher(
            "~start_deformation_model", std_msgs.msg.String, queue_size=10, latch=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)

        rospy.Subscriber(
            "~sensor_model_status", std_msgs.msg.String, self.sensor_model_status_cb)
        rospy.Subscriber(
            "~pose_extractor_status", std_msgs.msg.String, self.pose_extractor_status_cb)
        rospy.Subscriber(
            "~pose_controller_status", std_msgs.msg.String, self.pose_controller_status_cb)
        rospy.Subscriber(
            "~force_transformer_status", std_msgs.msg.String, self.force_transformer_status_cb)
        rospy.Subscriber(
            "~nodal_force_calculator_status", std_msgs.msg.String,
            self.nodal_force_calculator_status_cb)
        rospy.Subscriber(
            "~deformation_model_status", std_msgs.msg.String, self.deformation_model_status_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the component.

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def sensor_model_status_cb(self, msg):
        """
        Obtains the status of the component (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.sensor_model_status = msg.data

    def pose_extractor_status_cb(self, msg):
        """
        Obtains the status of the component (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.pose_extractor_status = msg.data

    def pose_controller_status_cb(self, msg):
        """
        Obtains the status of the component (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.pose_controller_status = msg.data

    def force_transformer_status_cb(self, msg):
        """
        Obtains the status of the component (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.force_transformer_status = msg.data

    def nodal_force_calculator_status_cb(self, msg):
        """
        Obtains the status of the component (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.nodal_force_calculator_status = msg.data

    def deformation_model_status_cb(self, msg):
        """
        Obtains the status of the component (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.deformation_model_status = msg.data

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
        if self.event in ['e_start', 'e_reset']:
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
            self.start_sensor_model.publish('e_stop')
            self.start_pose_extractor.publish('e_stop')
            self.start_pose_controller.publish('e_stop')
            self.start_force_transformer.publish('e_stop')
            self.start_nodal_force_calculator.publish('e_stop')
            self.start_deformation_model.publish('e_stop')
            self.started_components = False

        if event == 'e_reset':
            self.start_deformation_model.publish('e_reset')
            self.event = 'e_start'
            self.started_components = False

        if event == 'e_start' and not self.started_components:
            self.start_sensor_model.publish('e_start')
            self.start_pose_extractor.publish('e_start')
            self.start_pose_controller.publish('e_start')
            self.start_force_transformer.publish('e_start')
            self.start_nodal_force_calculator.publish('e_start')
            self.start_deformation_model.publish('e_start')
            self.started_components = True

    def reset_component_data(self, result):
        """
        Clears the data of the component.

        :param result: The result_mesh of the component, e.g. stopped, failure, success.
        :type result: str

        """
        self.toggle_components(result)
        self.event = None

        self.sensor_model_status = None
        self.pose_extractor_status = None
        self.pose_controller_status = None
        self.force_transformer_status = None
        self.nodal_force_calculator_status = None
        self.deformation_model_status = None
        self.started_components = False


def main():
    rospy.init_node("coordinator", anonymous=True)
    coordinator = Coordinator()
    coordinator.start()
