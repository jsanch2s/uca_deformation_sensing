#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node publishes a pose message based on the controls of a GUI.

**Output(s):**
  * `mock_up_pose`: A pose message.
    - *type:* `geometry_msgs/PoseStamped`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `reference_frame`: Reference frame of the pose to be published.

"""

import math
import threading
import Tkinter
import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf.transformations as transformations


# In meters.
LINEAR_RESOLUTION = 0.005
MAX_POS_X = 1.0
MIN_POS_X = -1.0
MAX_POS_Y = 1.0
MIN_POS_Y = -1.0
MAX_POS_Z = 2.0
MIN_POS_Z = 0.0

# In degrees.
ANGULAR_RESOLUTION = 1
MAX_ANGLE = 180
MIN_ANGLE = -180


roll = std_msgs.msg.Float32()
pitch = std_msgs.msg.Float32()
yaw = std_msgs.msg.Float32()
pose = geometry_msgs.msg.PoseStamped()

global lock
lock = threading.Lock()


def create_window():
    """
    Creates a GUI window to publish a pose.

    """
    master = Tkinter.Tk()

    scale_x = Tkinter.Scale(
        master, command=update_position_x, from_=MIN_POS_X, to=MAX_POS_X,
        resolution=LINEAR_RESOLUTION, label="position_x")
    scale_y = Tkinter.Scale(
        master, command=update_position_y, from_=MIN_POS_Y, to=MAX_POS_Y,
        resolution=LINEAR_RESOLUTION, label="position_y")
    scale_z = Tkinter.Scale(
        master, command=update_position_z, from_=MIN_POS_Z, to=MAX_POS_Z,
        resolution=LINEAR_RESOLUTION, label="position_z")

    scale_x.grid(row=0, column=0)
    scale_y.grid(row=0, column=1)
    scale_z.grid(row=0, column=2)

    scale_roll = Tkinter.Scale(
        master, command=update_roll, from_=MIN_ANGLE, to=MAX_ANGLE,
        resolution=ANGULAR_RESOLUTION, label="roll")
    scale_pitch = Tkinter.Scale(
        master, command=update_pitch, from_=MIN_ANGLE, to=MAX_ANGLE,
        resolution=ANGULAR_RESOLUTION, label="pitch")
    scale_yaw = Tkinter.Scale(
        master, command=update_yaw, from_=MIN_ANGLE, to=MAX_ANGLE,
        resolution=ANGULAR_RESOLUTION, label="yaw")

    scale_roll.grid(row=1, column=0)
    scale_pitch.grid(row=1, column=1)
    scale_yaw.grid(row=1, column=2)

    master.title("Pose mock-up")
    master.mainloop()
    rospy.signal_shutdown("GUI closed")


def update_position_x(slider):
    """
    Sets slider as the position magnitude in the X axis.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    pose.pose.position.x = float(slider)
    lock.release()


def update_position_y(slider):
    """
    Sets slider as the position magnitude in the X axis.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    pose.pose.position.y = float(slider)
    lock.release()


def update_position_z(slider):
    """
    Sets slider as the position in the Z axis.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    pose.pose.position.z = float(slider)
    lock.release()


def update_roll(slider):
    """
    Sets slider as the roll.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    roll.data = math.radians(float(slider))
    x, y, z, w = transformations.quaternion_from_euler(roll.data, pitch.data, yaw.data)
    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    lock.release()


def update_pitch(slider):
    """
    Sets slider as the pitch.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    pitch.data = math.radians(float(slider))
    x, y, z, w = transformations.quaternion_from_euler(roll.data, pitch.data, yaw.data)
    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    lock.release()


def update_yaw(slider):
    """
    Sets slider as the yaw.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    yaw.data = math.radians(float(slider))
    x, y, z, w = transformations.quaternion_from_euler(roll.data, pitch.data, yaw.data)
    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    lock.release()


def publish_pose():
    """
    Publishes the pose.

    """
    # Node cycle rate (in Hz).
    loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

    # Reference frame of the pose.
    reference_frame = rospy.get_param('~reference_frame', 'base_frame')

    # Publishers
    pub_pose = rospy.Publisher(
        '~mock_up_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

    pose.header.frame_id = reference_frame

    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        pub_pose.publish(pose)
        loop_rate.sleep()


def main():
    rospy.init_node('pose_mock_up')

    import thread
    try:
        thread.start_new_thread(create_window, tuple())

        publish_pose()
    except rospy.ROSInterruptException:
        pass
