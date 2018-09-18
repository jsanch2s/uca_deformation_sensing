#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node publishes a wrench message based on the controls of a GUI.

**Output(s):**
  * `mock_up_wrench`: A wrench message.
    - *type:* `geometry_msgs/WrenchStamped`
  * `force_array`: The wrench message converted to a force array message.
    - *type:* `udom_common_msgs/ForceArray`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `reference_frame`: Reference frame of the wrench to be published.

"""

import threading
import Tkinter
import numpy as np
import rospy
import tf.transformations as transformations
import std_msgs.msg
import geometry_msgs.msg
import udom_common_msgs.msg

# In Newtons
FORCE_RESOLUTION = 0.1
FORCE_SCALE = 1
MAX_FORCE = 1
MIN_FORCE = -1

# In meters.
POSITION_RESOLUTION = 0.005
MAX_POS_X = 2.0
MIN_POS_X = -2.0
MAX_POS_Y = 2.0
MIN_POS_Y = -2.0
MAX_POS_Z = 2.0
MIN_POS_Z = -2.0

wrench = geometry_msgs.msg.WrenchStamped()
pose = geometry_msgs.msg.PoseStamped()
position = geometry_msgs.msg.Point()
force_array = udom_common_msgs.msg.ForceArray()
event = std_msgs.msg.String()

global lock
lock = threading.Lock()


def create_window():
    """
    Creates a GUI window to publish a pose.

    """
    master = Tkinter.Tk()

    # Force magnitude.
    force_x = Tkinter.Scale(
        master, command=update_force_x, from_=MAX_FORCE, to=MIN_FORCE,
        resolution=FORCE_RESOLUTION, label="Force X"
    )
    force_y = Tkinter.Scale(
        master, command=update_force_y, from_=MAX_FORCE, to=MIN_FORCE,
        resolution=FORCE_RESOLUTION, label="Force Y"
    )
    force_z = Tkinter.Scale(
        master, command=update_force_z, from_=MAX_FORCE, to=MIN_FORCE,
        resolution=FORCE_RESOLUTION, label="Force Z"
    )
    force_x.grid(row=0, column=0)
    force_y.grid(row=0, column=1)
    force_z.grid(row=0, column=2)

    # Force position.
    position_x = Tkinter.Scale(
        master, command=update_position_x, from_=MAX_POS_X, to=MIN_POS_X,
        resolution=POSITION_RESOLUTION, label="Position X"
    )
    position_y = Tkinter.Scale(
        master, command=update_position_y, from_=MAX_POS_Y, to=MIN_POS_Y,
        resolution=POSITION_RESOLUTION, label="Position Y"
    )
    position_z = Tkinter.Scale(
        master, command=update_position_z, from_=MAX_POS_Z, to=MIN_POS_Z,
        resolution=POSITION_RESOLUTION, label="Position Z"
    )
    position_x.set((MAX_POS_X - MIN_POS_X) / 2.0)
    position_y.set((MAX_POS_Y - MIN_POS_Y) / 2.0)
    position_z.set((MAX_POS_Z - MIN_POS_Z) / 2.0)
    position_x.grid(row=1, column=0)
    position_y.grid(row=1, column=1)
    position_z.grid(row=1, column=2)

    # Event buttons.
    start_button = Tkinter.Button(master, command=start_cb, text="START")
    stop_button = Tkinter.Button(master, command=stop_cb, text="STOP")
    reset_button = Tkinter.Button(master, command=reset_cb, text="RESET")
    start_button.grid(row=2, column=0)
    stop_button.grid(row=2, column=1)
    reset_button.grid(row=2, column=2)

    master.title("Wrench mock-up")
    master.mainloop()
    rospy.signal_shutdown("GUI closed")


def update_force_x(slider):
    """
    Sets slider as the force magnitude in the X axis.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    wrench.wrench.force.x = float(slider) * FORCE_SCALE
    lock.release()


def update_force_y(slider):
    """
    Sets slider as the force magnitude in the Y axis.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    wrench.wrench.force.y = float(slider) * FORCE_SCALE
    lock.release()


def update_force_z(slider):
    """
    Sets slider as the force magnitude in the Z axis.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    wrench.wrench.force.z = float(slider) * FORCE_SCALE
    lock.release()


def update_position_x(slider):
    """
    Sets slider as the force position in the X axis.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    position.x = float(slider)
    lock.release()


def update_position_y(slider):
    """
    Sets slider as the force position in the Y axis.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    position.y = float(slider)
    lock.release()


def update_position_z(slider):
    """
    Sets slider as the force position in the Z axis.

    :param slider: Value from the slider in the GUI.
    :type slider: str

    """
    global lock
    lock.acquire()
    position.z = float(slider)
    lock.release()


def start_cb():
    """
    Updates the event to 'e_start'.

    """
    global lock
    lock.acquire()
    event.data = 'e_start'
    lock.release()


def stop_cb():
    """
    Updates the event to 'e_stop'.

    """
    global lock
    lock.acquire()
    event.data = 'e_stop'
    lock.release()


def reset_cb():
    """
    Updates the event to 'e_reset'.

    """
    global lock
    lock.acquire()
    event.data = 'e_reset'
    lock.release()


def add_bounds(x, lower, upper):
    """
    Puts x between the lower and upper bound. It assumes all the values
    in x are already in normalized between -1 and 1.

    :param x: Values to bound.
    :type x: list

    :param lower: Lower bound of range (inclusive).
    :type lower: float

    :param upper: Upper bound of range (inclusive).
    :type upper: float

    :return: The bounded value.
    :rtype: float

    """
    return np.interp(x, [-1, 1], [lower, upper])


def normalize_force(force, lower=0.0, upper=1.0):
    """
    Returns a normalized list from a force within a specified range.

    :param force: Force to normalize.
    :type force: geometry_msgs.msg.Vector3

    :param lower: Lower bound of range (inclusive).
    :type lower: float

    :param upper: Upper bound of range (inclusive).
    :type upper: float

    :return: The normalized forces as a list.
    :rtype: list

    """
    total = float(abs(force.x) + abs(force.y) + abs(force.z))
    if total:
        x = force.x / total
        y = force.y / total
        z = force.z / total
        return add_bounds([x, y, z], lower, upper)
    else:
        return [0.0, 0.0, 0.0]


def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts a list having roll, pitch and yaw angles (in radians) to a quaternion message.

    :param roll: Roll angle (in radians).
    :type roll: float.

    :param pitch: Pitch angle (in radians).
    :type pitch: float.

    :param yaw: Yaw angle (in radians).
    :type yaw: float.

    :return: The orientation as quaternion.
    :rtype: geometry_msgs.msg.Quaternion

    """
    x, y, z, w = transformations.quaternion_from_euler(roll, pitch, yaw)
    return geometry_msgs.msg.Quaternion(x, y, z, w)


def publish_wrench():
    """
    Publishes the wrench.

    """
    # Node cycle rate (in Hz).
    loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

    # Reference frame of the wrench.
    reference_frame = rospy.get_param('~reference_frame', 'map')

    # Publishers
    pub_wrench = rospy.Publisher(
        '~mock_up_wrench', geometry_msgs.msg.WrenchStamped, queue_size=10)
    pub_force_array = rospy.Publisher(
        '~force_array', udom_common_msgs.msg.ForceArray, queue_size=10)
    pub_force_pose = rospy.Publisher(
        '~force_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
    pub_event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=10)

    wrench.header.frame_id = reference_frame

    force_array.header.frame_id = reference_frame
    force_array.wrenches = [wrench.wrench]
    force_array.positions = [position]

    pose.header = wrench.header
    pose.pose.position = position

    while not rospy.is_shutdown():
        roll, yaw, pitch = normalize_force(wrench.wrench.force, lower=-np.pi/2, upper=np.pi/2)
        # Change sign of force on Z axis to match visualization.
        pose.pose.orientation = euler_to_quaternion(roll, -pitch, yaw)
        wrench.header.stamp = rospy.Time.now()
        force_array.header.stamp = wrench.header.stamp

        pub_wrench.publish(wrench)
        pub_force_array.publish(force_array)
        pub_force_pose.publish(pose)
        if event.data:
            pub_event_out.publish(event)
        event.data = ""
        loop_rate.sleep()


def main():
    rospy.init_node('wrench_mock_up')

    import thread
    try:
        thread.start_new_thread(create_window, tuple())

        publish_wrench()
    except rospy.ROSInterruptException:
        pass
