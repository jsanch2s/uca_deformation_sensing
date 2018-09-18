#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node enables a GUI to send commands (e.g. start/stop) to the reactive grasp demo.

**Output(s):**
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).

"""

import threading
import Tkinter
import rospy
import std_msgs.msg

event = std_msgs.msg.String()

global lock
lock = threading.Lock()


def create_window():
    """
    Creates a GUI to control the Shadow hand.

    """
    master = Tkinter.Tk()

    # Event buttons.
    start_button = Tkinter.Button(master, command=start_cb, text="START")
    reset_button = Tkinter.Button(master, command=reset_cb, text="RESET")
    stop_button = Tkinter.Button(master, command=stop_cb, text="STOP")
    start_button.grid(row=0, column=0)
    reset_button.grid(row=0, column=1)
    stop_button.grid(row=0, column=2)

    master.title("Reactive grasp controller")
    master.mainloop()
    rospy.signal_shutdown("GUI closed")


def start_cb():
    """
    Updates the event to 'e_start'.

    """
    global lock
    lock.acquire()
    event.data = 'e_start'
    lock.release()


def reset_cb():
    """
    Updates the event to 'e_reset'.

    """
    global lock
    lock.acquire()
    event.data = 'e_reset'
    lock.release()


def stop_cb():
    """
    Updates the event to 'e_stop'.

    """
    global lock
    lock.acquire()
    event.data = 'e_stop'
    lock.release()


def publish_events():
    """
    Publishes the joint positions.

    """
    # Node cycle rate (in Hz).
    loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100))

    # Publishers
    pub_event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=10)

    while not rospy.is_shutdown():
        pub_event_out.publish(event)
        event.data = ""
        loop_rate.sleep()


def main():
    rospy.init_node('reactive_grasp_gui')

    import thread
    try:
        thread.start_new_thread(create_window, tuple())
        publish_events()
    except rospy.ROSInterruptException:
        pass
