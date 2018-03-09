#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node converts a Mesh (udom_modeling_msgs/Mesh) into a point cloud
(sensor_msgs/PointCloud2). If a set of nodes are specified, then only those
nodes are converted (e.g. to extract only the surface).

**Input(s):**
  * `mesh_in`: Mesh to convert.
    - *type:* `udom_modeling_msgs/Mesh`

  * `nodes`: Nodes to extract
    - *type:* `std_msgs/Int32MultiArray`

**Output(s):**
  * `point_cloud_out`: Mesh as a point cloud message.
    - *type:* `sensor_msgs/PointCloud2`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `nodes`: Nodes to convert.

"""

import rospy
import std_msgs.msg
import sensor_msgs.msg
import sensor_msgs.point_cloud2 as point_cloud2
import udom_modeling_msgs.msg


class MeshToPointCloudNode(object):
    """
    Subscribes to a udom_modeling_msgs.msg.Mesh topic, converts it into a
    message sensor_msgs.msg.PointCloud2 and publishes it.

    """
    def __init__(self):
        """
        Instantiates a MeshToPointCloud node.

        :return: Node to convert a mesh into a point cloud.
        :rtype: MeshToPointCloudNode

        """
        # Params
        self.mesh = None
        self.nodes = []

        # Reference frame to assign to the converted point cloud.
        self.reference_frame = rospy.get_param('~reference_frame', 'map')

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.point_cloud_pub = rospy.Publisher(
            "~point_cloud_out", sensor_msgs.msg.PointCloud2, queue_size=10)

        # Subscribers
        rospy.Subscriber('~mesh_in', udom_modeling_msgs.msg.Mesh, self.mesh_in_cb)
        rospy.Subscriber('~nodes', std_msgs.msg.Int32MultiArray, self.nodes_cb)

        self.header = std_msgs.msg.Header()
        self.header.frame_id = self.reference_frame

    def mesh_in_cb(self, msg):
        """
        Obtains the mesh input.

        :param msg: Mesh to convert.
        :type msg: udom_modeling_msgs.msg.Mesh

        """
        self.mesh = msg

    def nodes_cb(self, msg):
        """
        Obtains the nodes to extract.

        :param msg: Nodes to extract
        :type msg: std_msgs.msg.Int32MultiArray

        """
        self.nodes = msg.data

    def start(self):
        """
        Starts the node.

        """
        rospy.loginfo("Ready to start...")
        state = 'IDLE'

        while not rospy.is_shutdown():

            if state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.mesh is not None:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        self.convert_to_point_cloud(self.mesh)
        self.reset_component_data()
        return 'IDLE'

    def convert_to_point_cloud(self, mesh):
        """
        Converts a mesh into a point cloud and publishes it.

        :param mesh: Mesh to be converted.
        :type mesh: udom_modeling_msgs.msg.Mesh


        """
        self.header.stamp = rospy.Time.now()

        points = self.generate_points(mesh.vertices, self.nodes)
        point_cloud = point_cloud2.create_cloud_xyz32(self.header, points)
        self.point_cloud_pub.publish(point_cloud)

    @staticmethod
    def generate_points(vertices, nodes):
        """
        Converts a list of points (geometry_msgs.msg.Point) into a list of points.
        If nodes are specified, only those nodes will be converted.

        :param vertices: Array of vertices describing the mesh.
        :type vertices: geometry_msgs.msg.Point[]

        :param nodes: Indices of the nodes to be extracted.
        :type nodes: list

        :return: A list of points.
        :rtype: list

        """
        if nodes:
            vertices_to_extract = [vertices[nn] for nn in nodes]
            points = [(pp.x, pp.y, pp.z) for pp in vertices_to_extract]
        else:
            points = [(pp.x, pp.y, pp.z) for pp in vertices]

        return points

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.mesh = None
        self.nodes = []


def main():
    rospy.init_node("mesh_to_point_cloud_node", anonymous=True)
    mesh_to_point_cloud_node = MeshToPointCloudNode()
    mesh_to_point_cloud_node.start()
