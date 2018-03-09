#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node fills a Marker message in order to be visualized (e.g. in RViz).

**Input(s):**
  * `points`: Points to be visualized.
    - *type:* `visualization_msgs/Marker`

**Output(s):**
  * `points`: Points to be visualized with their marker_out message correctly filled.
    - *type:* `visualization_msgs/Marker`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `points_frame`: Reference frame of the marker_out message to be filled.
  * `scale`: Marker's scale for the X, Y and Z axes.
  * `color`: Marker's color values for the red, green, blue and alpha.

"""

import rospy
import std_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg


class PointsVisualizerNode(object):
    """
    Subscribes to a visualization_msgs/Marker topic, fills its message appropriately and
    publishes it for visualization.

    """
    def __init__(self):
        """
        Instantiates a points visualizer node.

        :return: Node to fills a Marker message for visualization.
        :rtype: PointsVisualizerNode

        """
        # Params
        self.marker_in = None
        self.marker_out = visualization_msgs.msg.Marker()
        self.marker_out.type = self.marker_out.POINTS

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Reference frame of the points to be converted.
        self.reference_frame = rospy.get_param('~points_frame')
        assert self.reference_frame is not None, "'points_frame' must be specified."

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

        # Fill the marker_out's constant information.
        self.marker_out.header.frame_id = self.reference_frame
        self.marker_out.scale = geometry_msgs.msg.Vector3(*self.scale)
        self.marker_out.color = self.color

        # Publishers
        self.points = rospy.Publisher(
            "~points_out", visualization_msgs.msg.Marker, queue_size=10
        )

        # Subscribers
        rospy.Subscriber('~points_in', visualization_msgs.msg.Marker, self.points_cb)

    def points_cb(self, msg):
        """
        Obtains the points.

        :param msg: Points to be visualized.
        :type msg: visualization_msgs.msg.Marker

        """
        self.marker_in = msg

    def start(self):
        """
        Starts the node.

        """
        rospy.loginfo("Ready to start...")

        while not rospy.is_shutdown():
            if self.marker_in is not None:
                self.marker_out.header.stamp = rospy.Time.now()
                self.marker_out.points = self.marker_in.points
                for _ in self.marker_out.points:
                    self.marker_out.colors.append(self.color)
                self.points.publish(self.marker_out)
                self.marker_in = None

            self.loop_rate.sleep()


def main():
    rospy.init_node("points_visualizer_node", anonymous=True)
    points_visualizer_node = PointsVisualizerNode()
    points_visualizer_node.start()
