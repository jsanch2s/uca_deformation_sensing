#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node converts a Mesh into a Marker message in order to be visualized (e.g. in RViz).

**Input(s):**
  * `mesh`: Volumetric mesh to be visualized
    - *type:* `udom_modeling_msgs/Mesh`

**Output(s):**
  * `points`: The vertices of the mesh represented as a Marker.
    - *type:* `visualization_msgs/Marker`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `mesh_frame`: Reference frame of the mesh to be converted.
  * `scale`: Marker's scale for the X, Y and Z axes.
  * `color`: Marker's color values for the red, green, blue and alpha.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
import udom_modeling_msgs.msg


class MeshVisualizerNode(object):
    """
    Subscribes to a udom_modeling_msgs/Mesh topic, converts its message into a
    visualization_msgs/Marker message to publish it for visualization.

    """
    def __init__(self):
        """
        Instantiates a mesh visualizer node.

        :return: Node to transform a mesh into a marker for visualization.
        :rtype: MeshVisualizerNode

        """
        # Params
        self.mesh = None
        self.marker = visualization_msgs.msg.Marker()
        self.marker.type = self.marker.POINTS

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Reference frame of the mesh to be converted.
        self.reference_frame = rospy.get_param('~mesh_frame')
        assert self.reference_frame is not None, "'mesh_frame' must be specified."

        # Marker's scale for the X, Y and Z axes.
        self.scale = rospy.get_param('~scale', [0.05, 0.05, 0.05])
        assert isinstance(self.scale, list), "'scale' must be a list, " \
                                             "not '{}'.".format(type(self.scale))
        assert len(self.scale) == 3, "'scale' must be a list with 3 elements, " \
                                     "not {} elements.".format(len(self.scale))

        # Marker's color values for the red, green, blue and alpha.
        color = rospy.get_param('~color', [1.0, 0.0, 0.0, 0.5])
        assert isinstance(color, list), "'color' must be a list, " \
                                        "not '{}'.".format(type(color))
        assert len(color) == 4, "'color' must be a list with 4 elements, " \
                                "not {} elements.".format(len(color))
        self.color = std_msgs.msg.ColorRGBA(*color)

        # Fill the marker's constant information.
        self.marker.header.frame_id = self.reference_frame
        self.marker.scale = geometry_msgs.msg.Vector3(*self.scale)
        self.marker.color = self.color

        # Publishers
        self.points = rospy.Publisher(
            "~points", visualization_msgs.msg.Marker, queue_size=10
        )

        # Subscribers
        rospy.Subscriber('~mesh', udom_modeling_msgs.msg.Mesh, self.mesh_cb)

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

        while not rospy.is_shutdown():
            if self.mesh is not None:
                self.marker.header.stamp = rospy.Time.now()
                self.marker.points = self.mesh.vertices
                for _ in self.marker.points:
                    self.marker.colors.append(self.color)
                self.points.publish(self.marker)
                self.mesh = None

            self.loop_rate.sleep()


def main():
    rospy.init_node("mesh_visualizer_node", anonymous=True)
    mesh_visualizer_node = MeshVisualizerNode()
    mesh_visualizer_node.start()
