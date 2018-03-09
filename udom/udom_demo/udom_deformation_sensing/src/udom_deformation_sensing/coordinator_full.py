#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node uses a pipeline of components to perform deformation sensing using a
a Shadow hand with five BioTac sensors.
The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

It uses the following nodes:
  * `udom_topic_tools/tactile_demux`
  * `udom_sensor_model/tactile_sensor_model`
  * `udom_contact_model/contact_model`
  * `udom_topic_tools/force_merger`
  * `udom_geometric_transformation/force_transformer`
  * `udom_geometric_transformation/nodal_force_calculator`
  * `udom_topic_tools/topic_throttle`
  * `udom_deformation_modeling/deformation_model`

**Assumptions:**
  * It assumes five BioTac sensors as the input sensors.

**Input(s):**
  * `tactile_info`: The output data of five BioTac sensors.
    - *type:* `sr_robot_msgs/BiotacAll`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_reset`: resets the mesh to its original undeformed state.
    - *type:* `std_msgs/String`

**Output(s):**
  * `mesh`: A mesh representation of the object with the updated nodes' position based
        on  the deformation.
    - *type:* `udom_modeling_msgs/Mesh`
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `mesh_filename`: Filename of the volumetric mesh in a .veg format. **Note:** This file
        should be located in the config directory of the `deformation_sensing` package.
  * `reference_frame`: Reference frame of the object.
  * `constrained_nodes`: Constrained vertices of the mesh. Each constrained node must
        specify its three degrees of freedom. E.g., to constrain vertices 4, 10 and 14 the
        constrained_nodes should be [12, 13, 14, 30, 31, 32, 42, 43, 44].

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

        self.first_finger_contact_model_status = None
        self.middle_finger_contact_model_status = None
        self.ring_finger_contact_model_status = None
        self.little_finger_contact_model_status = None
        self.thumb_contact_model_status = None

        self.force_merger_status = None
        self.force_transformer_status = None
        self.nodal_force_calculator_status = None
        self.topic_throttle_status = None
        self.deformation_model_status = None

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)

        self.start_first_finger_sensor = rospy.Publisher(
            "~start_first_finger_sensor", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_middle_finger_sensor = rospy.Publisher(
            "~start_middle_finger_sensor", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_ring_finger_sensor = rospy.Publisher(
            "~start_ring_finger_sensor", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_little_finger_sensor = rospy.Publisher(
            "~start_little_finger_sensor", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_thumb_sensor = rospy.Publisher(
            "~start_thumb_sensor", std_msgs.msg.String, queue_size=10, latch=True
        )

        self.start_first_finger_sensor_model = rospy.Publisher(
            "~start_first_finger_sensor_model", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_middle_finger_sensor_model = rospy.Publisher(
            "~start_middle_finger_sensor_model", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_ring_finger_sensor_model = rospy.Publisher(
            "~start_ring_finger_sensor_model", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_little_finger_sensor_model = rospy.Publisher(
            "~start_little_finger_sensor_model", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_thumb_sensor_model = rospy.Publisher(
            "~start_thumb_sensor_model", std_msgs.msg.String, queue_size=10, latch=True
        )

        self.start_first_finger_contact_model = rospy.Publisher(
            "~start_first_finger_contact_model", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_middle_finger_contact_model = rospy.Publisher(
            "~start_middle_finger_contact_model", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_ring_finger_contact_model = rospy.Publisher(
            "~start_ring_finger_contact_model", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_little_finger_contact_model = rospy.Publisher(
            "~start_little_finger_contact_model", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_thumb_contact_model = rospy.Publisher(
            "~start_thumb_contact_model", std_msgs.msg.String, queue_size=10, latch=True
        )

        self.start_force_merger = rospy.Publisher(
            "~start_force_merger", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_force_transformer = rospy.Publisher(
            "~start_force_transformer", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_nodal_force_calculator = rospy.Publisher(
            "~start_nodal_force_calculator", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_topic_throttle = rospy.Publisher(
            "~start_topic_throttle", std_msgs.msg.String, queue_size=10, latch=True
        )
        self.start_deformation_model = rospy.Publisher(
            "~start_deformation_model", std_msgs.msg.String, queue_size=10, latch=True
        )

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)

        rospy.Subscriber(
            "~first_finger_sensor_status", std_msgs.msg.String, self.ff_sensor_status_cb
        )
        rospy.Subscriber(
            "~middle_finger_sensor_status", std_msgs.msg.String, self.mf_sensor_status_cb
        )
        rospy.Subscriber(
            "~ring_finger_sensor_status", std_msgs.msg.String, self.rf_sensor_status_cb
        )
        rospy.Subscriber(
            "~little_finger_sensor_status", std_msgs.msg.String, self.lf_sensor_status_cb
        )
        rospy.Subscriber(
            "~thumb_sensor_status", std_msgs.msg.String, self.th_sensor_status_cb
        )

        rospy.Subscriber(
            "~first_finger_sensor_model_status", std_msgs.msg.String, self.ff_model_status_cb
        )
        rospy.Subscriber(
            "~middle_finger_sensor_model_status", std_msgs.msg.String, self.mf_model_status_cb
        )
        rospy.Subscriber(
            "~ring_finger_sensor_model_status", std_msgs.msg.String, self.rf_model_status_cb
        )
        rospy.Subscriber(
            "~little_finger_sensor_model_status", std_msgs.msg.String, self.lf_model_status_cb
        )
        rospy.Subscriber(
            "~thumb_sensor_model_status", std_msgs.msg.String, self.th_model_status_cb
        )

        rospy.Subscriber(
            "~first_finger_contact_model_status", std_msgs.msg.String,
            self.ff_contact_model_status_cb
        )
        rospy.Subscriber(
            "~middle_finger_contact_model_status", std_msgs.msg.String,
            self.mf_contact_model_status_cb
        )
        rospy.Subscriber(
            "~ring_finger_contact_model_status", std_msgs.msg.String,
            self.rf_contact_model_status_cb
        )
        rospy.Subscriber(
            "~little_finger_contact_model_status", std_msgs.msg.String,
            self.lf_contact_model_status_cb
        )
        rospy.Subscriber(
            "~thumb_sensor_contact_status", std_msgs.msg.String,
            self.th_contact_model_status_cb
        )

        rospy.Subscriber(
            "~force_merger_status", std_msgs.msg.String, self.force_merger_status_cb
        )
        rospy.Subscriber(
            "~force_transformer_status", std_msgs.msg.String,
            self.force_transformer_status_cb
        )
        rospy.Subscriber(
            "~nodal_force_calculator_status", std_msgs.msg.String,
            self.nodal_force_calculator_status_cb
        )
        rospy.Subscriber(
            "~topic_throttle_status", std_msgs.msg.String,
            self.topic_throttle_status_cb
        )
        rospy.Subscriber(
            "~deformation_model_status", std_msgs.msg.String,
            self.deformation_model_status_cb
        )

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

    def ff_contact_model_status_cb(self, msg):
        """
        Obtains the status of the first finger contact model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.first_finger_contact_model_status = msg.data

    def mf_contact_model_status_cb(self, msg):
        """
        Obtains the status of the middle finger contact model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.middle_finger_contact_model_status = msg.data

    def rf_contact_model_status_cb(self, msg):
        """
        Obtains the status of the ring finger contact model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.ring_finger_contact_model_status = msg.data

    def lf_contact_model_status_cb(self, msg):
        """
        Obtains the status of the little finger contact model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.little_finger_contact_model_status = msg.data

    def th_contact_model_status_cb(self, msg):
        """
        Obtains the status of the thumb contact model (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.thumb_contact_model_status = msg.data

    def force_merger_status_cb(self, msg):
        """
        Obtains the status of the force merger (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.force_merger_status = msg.data

    def force_transformer_status_cb(self, msg):
        """
        Obtains the status of the force transformer (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.force_transformer_status = msg.data

    def nodal_force_calculator_status_cb(self, msg):
        """
        Obtains the status of the nodal force calculator (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.nodal_force_calculator_status = msg.data

    def topic_throttle_status_cb(self, msg):
        """
        Obtains the status of the topic throttle node (as an event).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.topic_throttle_status = msg.data

    def deformation_model_status_cb(self, msg):
        """
        Obtains the status of the deformation model (as an event).

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

            self.start_first_finger_contact_model.publish('e_stop')
            self.start_middle_finger_contact_model.publish('e_stop')
            self.start_ring_finger_contact_model.publish('e_stop')
            self.start_little_finger_contact_model.publish('e_stop')
            self.start_thumb_contact_model.publish('e_stop')

            self.start_force_merger.publish('e_stop')
            self.start_force_transformer.publish('e_stop')
            self.start_nodal_force_calculator.publish('e_stop')
            self.start_topic_throttle.publish('e_stop')
            self.start_deformation_model.publish('e_stop')
            self.started_components = False

        if event == 'e_reset':
            self.start_deformation_model.publish('e_reset')
            self.event = 'e_start'
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

            self.start_first_finger_contact_model.publish('e_start')
            self.start_middle_finger_contact_model.publish('e_start')
            self.start_ring_finger_contact_model.publish('e_start')
            self.start_little_finger_contact_model.publish('e_start')
            self.start_thumb_contact_model.publish('e_start')

            self.start_force_merger.publish('e_start')
            self.start_force_transformer.publish('e_start')
            self.start_nodal_force_calculator.publish('e_start')
            self.start_topic_throttle.publish('e_start')
            self.start_deformation_model.publish('e_start')
            self.started_components = True

    def reset_component_data(self, result):
        """
        Clears the data of the component.

        :param result: The result of the component, e.g. stopped, failure, success.
        :type result: str

        """
        self.toggle_components(result)
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

        self.first_finger_contact_model_status = None
        self.middle_finger_contact_model_status = None
        self.ring_finger_contact_model_status = None
        self.little_finger_contact_model_status = None
        self.thumb_contact_model_status = None

        self.force_merger_status = None
        self.force_transformer_status = None
        self.nodal_force_calculator_status = None
        self.topic_throttle_status = None
        self.deformation_model_status = None
        self.started_components = False


def main():
    rospy.init_node("coordinator", anonymous=True)
    coordinator = Coordinator()
    coordinator.start()
