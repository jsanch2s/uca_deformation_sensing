#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node transforms the forces in a udom_common_msgs.msg.ForceArray message
to loads that are applied on each node of the specified volumetric mesh. The output
is an array of a three-element force vector for each node. By setting `add_gravity`
to True, it computes the gravity vector with respect to the `reference_frame`, it
requires that the user sets the gravity vector, e.g. (0, 0, -9.81) would mean the
gravity acts downwards if the Z axis of the reference frame is pointing upwards.

<sub>[**Note:** The mesh nodes are assumed to be on the same frame reference as the forces.]</sub>

**Input(s):**
  * `force_info`: The forces applied on a mesh.
    - *type:* `udom_common_msgs/ForceArray`
  * `mesh`: Volumetric mesh on which the forces are applied.
    - *type:* `udom_modeling_msgs/Mesh`
  * `event_in`: The desired event for the node:
      `e_start`: starts the component.
      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `force_out`: Nodal forces. The linear forces (in X, Y and Z) applied on each node
        of the mesh.
    - *type:* `std_msgs/Float32MultiArray`
  * `event_out`: The current event of the node.
      `e_running`: when the component is running.
      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `collision_distance`: Distance to determine if a collision is occurring between the mesh and the finger.
  * `add_gravity`: If true, it adds the gravity force with respect to the reference frame.
  * `gravity_vector`: The gravity vector, defaults to [0, 0, -9.805665] (in m/s^2).
  * `reference_frame`: Reference frame to compute the gravity vector.
  * `object_frame`: Object frame to compute the gravity vector.
  * `wait_for_transform`: Maximum duration to wait for a transform (in seconds).
  * `mass`: Object's mass (in Kg).

"""

import rospy
import tf2_ros
import std_msgs.msg
import geometry_msgs.msg
import udom_common_msgs.msg
import udom_modeling_msgs.msg
import numpy as np
from itertools import izip as zip
import udom_geometric_transformation.transformation_utils as utils
import udom_geometric_transformation.nodal_force_computation as nodal_force_computation


class NodalForceCalculatorNode(object):
    """
    Subscribes to a udom_common_msgs.msg.ForceArray topic and transforms each of the
    forces into nodal loads on the specified mesh. It publishes a message of type
    std_msgs.msg.Float32MultiArray specifying a force vector on each vertex of the mesh.

    """
    def __init__(self):
        """
        Instantiates a nodal force calculator node.

        :return: Node to compute the nodal forces of a mesh.
        :rtype: NodalForceCalculatorNode

        """
        # Params
        self.event = None
        self.force_info = None
        self.mesh = None
        self.gravity_computed = False
        self.g_force = [0.0, 0.0, 0.0]

        # Distance to determine if a collision is occurring between the mesh and the finger.
        self.collision_distance = rospy.get_param('~collision_distance', 0.001)

        # Enable gravity.
        self.add_gravity = rospy.get_param('~add_gravity', True)
        if self.add_gravity:
            # Gravity vector (in m/s^2).
            self.gravity_vector = rospy.get_param("~gravity_vector", [0, 0, -9.805665])
            assert isinstance(self.gravity_vector, list) and (len(self.gravity_vector) == 3), \
                "'gravity_vector' must be a three element list (e.g. [0, 0, -9.805665])," \
                " not '{}'".format(self.gravity_vector)
            # Reference frame to compute the gravity vector.
            self.reference_frame = rospy.get_param("~reference_frame", None)
            assert self.reference_frame is not None, "If `add_gravity` is set, then a " \
                                                     "reference frame must be specified."
            # Object frame to compute the gravity vector.
            self.object_frame = rospy.get_param("~object_frame", None)
            assert self.object_frame is not None, "If `add_gravity` is set, then a " \
                                                  "object frame must be specified."
            # Object's mass (in Kg).
            self.mass = rospy.get_param("~mass", None)
            assert self.mass is not None, "If `add_gravity` is set, then a  mass must " \
                                          "be specified."
            # Maximum duration to wait for a transform (in seconds).
            self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

            # Object to compute transformations.
            self.buffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buffer)
        else:
            self.gravity_vector = None
            self.reference_frame = None
            self.mass = None

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.force_out = rospy.Publisher(
            "~force_out", std_msgs.msg.Float32MultiArray, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~force_in', udom_common_msgs.msg.ForceArray, self.force_in_cb)
        rospy.Subscriber('~mesh', udom_modeling_msgs.msg.Mesh, self.mesh_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def force_in_cb(self, msg):
        """
        Obtains the force information input.

        :param msg: Force information.
        :type msg: udom_common_msgs.msg.ForceArray

        """
        self.force_info = msg

    def mesh_cb(self, msg):
        """
        Obtains the mesh.

        :param msg: Mesh.
        :type msg: udom_modeling_msgs.msg.Mesh

        """
        self.mesh = msg

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
        elif self.force_info is not None and self.mesh is not None:
            if self.add_gravity:
                self.compute_gravity()
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
            force_out = self.compute_nodal_forces(self.force_info)
            if force_out:
                self.event_out.publish('e_running')
                self.force_out.publish(force_out)
            else:
                rospy.logwarn("Error while computing the forces.")
            self.reset_component_data()
            return 'IDLE'

    def compute_nodal_forces(self, force_in):
        """
        Computes the nodal forces of the specified mesh. It distributes the force
        among the three nodes of the mesh element where the force is applied.
        The distribution of force is inversely proportional to the distance from
        the force to the node.

        :param force_in: The force information.
        :type force_in: udom_common_msgs.msg.ForceArray

        :return: The nodal force information.
        :rtype: std_msgs.msg.Float32MultiArray

        """
        force_out = std_msgs.msg.Float32MultiArray()
        force_out.data = np.zeros(3 * len(self.mesh.vertices)).tolist()

        for wrench, point in zip(force_in.wrenches, force_in.positions):
            points, idx, in_collision = nodal_force_computation.neighbors(
                point, self.mesh, n=3, distance=self.collision_distance)
            if not in_collision:
                for index, node in enumerate(idx):
                    nn = node * 3
                    force_out.data[nn] += self.g_force[0]
                    force_out.data[nn+1] += self.g_force[1]
                    force_out.data[nn+2] += self.g_force[2]
                return force_out

            triangle = geometry_msgs.msg.Polygon()
            triangle.points = points
            nodal_forces = nodal_force_computation.compute_nodal_forces(
                point, wrench.force, triangle)

            # For each of the three nodes where the force is closer to,
            # compute the force components in X, Y and Z.
            for index, node in enumerate(idx):
                nn = node * 3
                ii = index * 3
                force_out.data[nn] += nodal_forces[ii] + self.g_force[0]
                force_out.data[nn+1] += nodal_forces[ii+1] + self.g_force[1]
                force_out.data[nn+2] += nodal_forces[ii+2] + self.g_force[2]

        return force_out

    def compute_gravity(self):
        """
        Computes the gravity force based on the mass and the gravity vector.

        """
        try:
            trans = self.buffer.lookup_transform(
                self.object_frame, self.reference_frame, rospy.Time.now(),
                timeout=rospy.Duration(self.wait_for_transform))
            gravity_vector = utils.quaternion_rotation(
                self.gravity_vector, trans.transform.rotation)
            self.g_force = (float(self.mass) / len(self.mesh.vertices)) * gravity_vector
            self.gravity_computed = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Error while transforming frames:\n{}".format(e))

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.force_info = None
        self.mesh = None
        self.event = None
        self.gravity_computed = False


def main():
    rospy.init_node("nodal_force_calculator_node", anonymous=True)
    nodal_force_calculator_node = NodalForceCalculatorNode()
    nodal_force_calculator_node.start()
