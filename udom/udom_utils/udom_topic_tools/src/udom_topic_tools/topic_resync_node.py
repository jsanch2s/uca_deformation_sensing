#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node publishes the input topic with an updated time stamp (e.g. the current one).

**Input(s):**
  * `topic_in`: Topic input.
    - *type:* `tf2_msgs/TFMessage`

**Output(s):**
  * `topic_out`: Topic output.
    - *type:* `tf2_msgs/TFMessage`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).

"""

import rospy
import tf2_msgs.msg


class TopicResyncNode(object):
    """
    Publishes the topic with an updated time stamp.

    """
    def __init__(self):
        """
        Instantiates a topic resync node.

        :return: Node to resync a topic.
        :rtype: TopicResyncNode

        """
        # Params
        self.event = None
        self.topic_in = None

        # Name of input topic.
        input_topic = rospy.get_param('~input_topic', 'tf_old')

        # Name of output topic.
        output_topic = rospy.get_param('~output_topic', 'tf')

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100))

        # Publishers
        self.topic_out = rospy.Publisher(
            output_topic, tf2_msgs.msg.TFMessage, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber(input_topic, tf2_msgs.msg.TFMessage, self.topic_cb)

    def topic_cb(self, msg):
        """
        Obtains the topic data.

        :param msg: Topic data.
        :type msg: tf2_msgs.msg.TFMessage

        """
        self.topic_in = msg

    def start(self):
        """
        Starts the node.

        """
        rospy.loginfo("Ready to start...")

        while not rospy.is_shutdown():

            if self.topic_in is not None:
                for msg in self.topic_in.transforms:
                    msg.header.stamp = rospy.Time.now()
                self.topic_out.publish(self.topic_in)
            self.topic_in = None
            self.loop_rate.sleep()


def main():
    rospy.init_node("topic_resync_node", anonymous=True)
    topic_resync_node = TopicResyncNode()
    topic_resync_node.start()
