#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node extracts a pose from a mesh given the indices of three nodes.
The position of the pose is the centroid of those three points and its
orientation its the normal defined by the plane of the points.

**Input(s):**

  * `mesh`: The mesh from which the pose will be extracted.
    - *type:* `udom_modeling_msgs/Mesh`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

    - *type:* `std_msgs/String`

**Output(s):**

  * `pose`: The extracted pose.
    - *type:* `geometry_msgs/PoseStamped`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `reference_frame`: Reference frame of the mesh.

  * `nodes`: Indices for three contiguous nodes on the mesh.

  * `zero_based`: Whether the indices are specified on zero-based (True) or one-based (False).

  * `flip_pose`: Inverts the the normal of the pose if True.

  * `rotate_pose`: Rotate the pose around the specified axes in the given order (in degrees) when
    an 'e_rotate' event is received.

  * `rotation_axes`: Axes on which the rotation will be applied.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import udom_modeling_msgs.msg
import udom_geometric_transformation.pose_extractor_utils as utils


class PoseExtractorNode(object):
    """
    Subscribes to a udom_modeling_msgs.msg.Mesh topic and extracts a pose based on
    the specified nodes. It publishes a PoseStamped.

    """
    def __init__(self):
        """
        Instantiates a pose extractor node.

        :return: Node to extract a pose from a mesh.
        :rtype: PoseExtractorNode

        """
        # Params
        self.event = None
        self.mesh = None

        # Flag to either apply a rotation or not.
        self.keep_rotation = False

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Whether the indices are specified on zero-based or one-based.
        self.zero_based = rospy.get_param('~zero_based', True)

        # Inverts the the normal of the pose if True.
        self.flip_pose = rospy.get_param('~flip_pose', True)

        # Rotate the pose around the specified axes in the given order (in degrees) when
        # an 'e_rotate' event is received.
        self.rotate_pose = rospy.get_param('~rotate_pose', None)
        assert self.rotate_pose is not None, "'rotate_pose' frame must be specified."
        assert isinstance(self.rotate_pose, list), "'rotate_pose' must be a list, " \
                                                   "not '{}'.".format(type(self.rotate_pose))
        assert len(self.rotate_pose) == 3, "'rotate_pose' must be a list with 3 integers, " \
                                           "not {} elements.".format(len(self.rotate_pose))
        # Axes on which the rotation will be applied.
        self.rotation_axes = rospy.get_param('~rotation_axes', 'xyz')
        assert isinstance(self.rotation_axes, str), "'rotation_axes' must be a string."
        assert len(self.rotation_axes) == 3,  "'rotation_axes' must be a string with " \
                                              "3 characters."

        # Reference frame of the mesh.
        self.reference_frame = rospy.get_param("~reference_frame", None)
        assert self.reference_frame is not None, "A reference frame must be specified."

        self.nodes = rospy.get_param('~nodes', None)
        assert self.nodes is not None, "'nodes' must be specified."
        assert isinstance(self.nodes, list), "'nodes' must be a list, " \
                                             "not '{}'.".format(type(self.nodes))
        assert len(self.nodes) == 3, "'nodes' must be a list with 3 integers, " \
                                     "not {} elements.".format(len(self.nodes))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.pose = rospy.Publisher(
            "~pose", geometry_msgs.msg.PoseStamped, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~mesh', udom_modeling_msgs.msg.Mesh, self.mesh_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def mesh_cb(self, msg):
        """
        Obtains the mesh.

        :param msg: Force information.
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
        elif self.mesh is not None:
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
            pose = self.extract_pose(self.mesh)
            if pose:
                self.event_out.publish('e_running')
                self.pose.publish(pose)
            else:
                rospy.logwarn("Error while extracting the pose.")
            self.reset_component_data()
            return 'IDLE'

    def extract_pose(self, mesh):
        """
        Extracts the pose from the mesh based on the specified nodes.

        :param mesh: The mesh from which the pose will be extracted.
        :type mesh: udom_modeling_msgs.msg.Mesh

        :return: The extracted pose, if there's an error it outputs None.
        :rtype: geometry_msgs.msg.PoseStamped or None.

        """
        pose = geometry_msgs.msg.PoseStamped()

        points = utils.extract_points(mesh, self.nodes, self.zero_based)
        if points is None:
            return None

        centroid = utils.compute_centroid(points)
        normal = utils.compute_normal_as_quaternion(points, self.flip_pose)

        pose.pose.position = centroid
        pose.pose.orientation = normal

        if self.event == 'e_rotate':
            self.keep_rotation = not self.keep_rotation

        if self.keep_rotation:
            pose = utils.rotate_pose(pose, self.rotate_pose[0], self.rotation_axes[0])
            pose = utils.rotate_pose(pose, self.rotate_pose[1], self.rotation_axes[1])
            pose = utils.rotate_pose(pose, self.rotate_pose[2], self.rotation_axes[2])

        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.reference_frame

        return pose

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.mesh = None
        self.event = None


def main():
    rospy.init_node("pose_extractor_node", anonymous=True)
    pose_extractor_node = PoseExtractorNode()
    pose_extractor_node.start()
