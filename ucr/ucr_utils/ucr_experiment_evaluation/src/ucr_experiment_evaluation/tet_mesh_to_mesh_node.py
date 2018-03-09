#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node converts a tet mesh (udom_modeling_msgs/Mesh) into a triangular mesh
(shape_msgs/Mesh) based on the nodes that are on the surface.

**Input(s):**
  * `mesh_in`: Tet mesh to convert.
    - *type:* `udom_modeling_msgs/Mesh`

**Output(s):**
  * `mesh_out`: Surface (triangular) Mesh.
    - *type:* `shape_msgs/Mesh`
  * `mesh_visualization`: Surface (triangular) Mesh for visualization.
    - *type:* `visualization_msgs/Marker`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `reference_frame`: Reference frame of the mesh.
  * `surface_nodes`: Nodes to extract.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import visualization_msgs.msg
import udom_modeling_msgs.msg


class TetMeshToMeshNode(object):
    """
    Subscribes to a udom_modeling_msgs.msg.Mesh topic, converts it into a
    message sensor_msgs.msg.PointCloud2 and publishes it.

    """
    def __init__(self):
        """
        Instantiates a MeshToPointCloud node.

        :return: Node to convert a mesh into a point cloud.
        :rtype: TetMeshToMeshNode

        """
        # Params
        self.mesh_in = None

        # Reference frame of the mesh.
        self.reference_frame = rospy.get_param('~reference_frame', 'object')
        # Surface nodes to extract.
        self.surface_nodes = rospy.get_param('~surface_nodes', [])

        # Scale to visualize the triangles in the mesh.
        self.scale = rospy.get_param('~scale', 1.0)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.mesh_out_pub = rospy.Publisher("~mesh_out", shape_msgs.msg.Mesh, queue_size=10)
        self.mesh_vis_pub = rospy.Publisher(
            "~mesh_visualization", visualization_msgs.msg.Marker, queue_size=10)

        # Subscribers
        rospy.Subscriber('~mesh_in', udom_modeling_msgs.msg.Mesh, self.mesh_in_cb)

        self.mesh_out = shape_msgs.msg.Mesh()
        self.mesh_vis = visualization_msgs.msg.Marker()
        self.mesh_vis.header.frame_id = self.reference_frame
        self.mesh_vis.type = visualization_msgs.msg.Marker.TRIANGLE_LIST
        self.mesh_vis.scale = geometry_msgs.msg.Vector3(self.scale, self.scale, self.scale)
        self.mesh_vis.color = std_msgs.msg.ColorRGBA(0.93, 0.82, 0.59, 1.0)

    def mesh_in_cb(self, msg):
        """
        Obtains the mesh input.

        :param msg: Mesh to convert.
        :type msg: udom_modeling_msgs.msg.Mesh

        """
        self.mesh_in = msg

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
        if self.mesh_in is not None:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        mesh_out = self.convert_to_triangular_mesh()
        if mesh_out:
            self.mesh_out_pub.publish(mesh_out)
            mesh_vis = self.convert_to_visual_mesh(mesh_out)
            if mesh_vis:
                self.mesh_vis_pub.publish(mesh_vis)

        self.reset_component_data()
        return 'IDLE'

    def convert_to_triangular_mesh(self):
        """
        Converts a mesh into a point cloud and publishes it.

        :return: Converted triangular mesh
        :rtype: shape_msgs.msg.Mesh

        """
        self.mesh_out.triangles = []
        self.mesh_out.vertices = []
        self.mesh_out.vertices = [vv for vv in self.mesh_in.vertices]
        vertices_idx = range(len(self.mesh_out.vertices))

        self.mesh_out.triangles = []
        for ii, tet in enumerate(self.mesh_in.tetrahedra):
            if tet:
                triangle_idx = [vi for vi in tet.vertex_indices if vi in vertices_idx]
                triangle_1 = shape_msgs.msg.MeshTriangle(
                    (triangle_idx[0], triangle_idx[1], triangle_idx[2]))
                triangle_2 = shape_msgs.msg.MeshTriangle(
                    (triangle_idx[1], triangle_idx[2], triangle_idx[3]))
                triangle_3 = shape_msgs.msg.MeshTriangle(
                    (triangle_idx[2], triangle_idx[3], triangle_idx[0]))
                triangle_4 = shape_msgs.msg.MeshTriangle(
                    (triangle_idx[3], triangle_idx[0], triangle_idx[1]))

                self.mesh_out.triangles.append(triangle_1)
                self.mesh_out.triangles.append(triangle_2)
                self.mesh_out.triangles.append(triangle_3)
                self.mesh_out.triangles.append(triangle_4)

        return self.mesh_out

    def convert_to_visual_mesh(self, mesh_in):
        """
        Converts a mesh into a visual marker.

        :param mesh_in: Triangular mesh.
        :type mesh_in: shape_msgs.msg.Mesh

        :return: Converted triangular mesh for visualization
        :rtype: visualization_msgs.msg.Marker

        """
        self.mesh_vis.header.stamp = rospy.Time.now()
        self.mesh_vis.points = []

        indices = []
        for triangle in mesh_in.triangles:
            indices.extend(triangle.vertex_indices)
        vertices = [mesh_in.vertices[ii] for ii in indices]
        for vv in vertices:
            self.mesh_vis.points.append(vv)
        return self.mesh_vis

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.mesh_in = None


def main():
    rospy.init_node("tet_mesh_to_mesh_node", anonymous=True)
    tet_mesh_to_mesh_node = TetMeshToMeshNode()
    tet_mesh_to_mesh_node.start()
