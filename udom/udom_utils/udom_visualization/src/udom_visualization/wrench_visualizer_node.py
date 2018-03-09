#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node converts a ContactInfo message into a PointStamped and a WrenchStamped messages,
it also adds their header information in order to be visualized (e.g. in RViz).

**Input(s):**
  * `contact_info`: Contact information to be visualized.
    - *type:* `udom_perception_msgs/ContactInfo`

**Output(s):**
  * `point`: Point to be visualized with their marker_out message correctly filled.
    - *type:* `geometry_msgs/PointStamped`
  * `wrench`: Wrench to be visualized with their marker_out message correctly filled.
    - *type:* `geometry_msgs/WrenchStamped`


**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).

"""

import rospy
import geometry_msgs.msg
import udom_perception_msgs.msg


class WrenchVisualizerNode(object):
    """
    Subscribes to a udom_perception_msgs/ContactInfo topic, converts and fill its message
    appropriately to be published as geometry_msgs/PointStamped and geometry_msgs/WrenchStamped
    messages for visualization.

    """
    def __init__(self):
        """
        Instantiates a wrench visualizer node.

        :return: Node to fills a Marker message for visualization.
        :rtype: WrenchVisualizerNode

        """
        # Params
        self.contact_info = None

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.point = rospy.Publisher(
            "~point", geometry_msgs.msg.PointStamped, queue_size=10)
        self.wrench = rospy.Publisher(
            "~wrench", geometry_msgs.msg.WrenchStamped, queue_size=10)

        # Subscribers
        rospy.Subscriber(
            '~contact_info', udom_perception_msgs.msg.ContactInfo, self.contact_info_cb)

    def contact_info_cb(self, msg):
        """
        Obtains the contact information.

        :param msg: Contact information.
        :type msg: udom_perception_msgs.msg.ContactInfo

        """
        self.contact_info = msg

    def start(self):
        """
        Starts the node.

        """
        rospy.loginfo("Ready to start...")

        while not rospy.is_shutdown():
            if self.contact_info is not None:
                point = geometry_msgs.msg.PointStamped()
                wrench = geometry_msgs.msg.WrenchStamped()
                point.header.stamp = rospy.Time.now()
                wrench.header.stamp = rospy.Time.now()
                point.header.frame_id = self.contact_info.header.frame_id
                wrench.header.frame_id = self.contact_info.header.frame_id

                point.point = self.contact_info.contact_points[0]
                wrench.wrench = self.contact_info.wrenches[0]
                self.point.publish(point)
                self.wrench.publish(wrench)
                self.contact_info = None

            self.loop_rate.sleep()


def main():
    rospy.init_node("wrench_visualizer_node", anonymous=True)
    wrench_visualizer_node = WrenchVisualizerNode()
    wrench_visualizer_node.start()
