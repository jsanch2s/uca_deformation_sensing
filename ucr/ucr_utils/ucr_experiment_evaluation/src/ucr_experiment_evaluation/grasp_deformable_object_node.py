#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node controls the Shadow hand to open/close the desired fingers.

**Input(s):**
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `joint_positions`: Commands a joint joint_positions to the specified finger(s).
    - *type:* `std_msgs/Float64`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).

"""

import threading
import Tkinter
import rospy
import std_msgs.msg
import sensor_msgs.msg
import controller_manager_msgs.srv


joint_positions = std_msgs.msg.Float64()
up_joint_position = std_msgs.msg.Bool()
down_joint_position = std_msgs.msg.Bool()
thumb_position = std_msgs.msg.Float64()
event = std_msgs.msg.String()
joint_states = sensor_msgs.msg.JointState()

joint_vars = []
joints_to_move = {
     'rh_FFJ2': False, 'rh_MFJ2': False, 'rh_RFJ2': False, 'rh_LFJ2': False,
     'rh_FFJ3': False, 'rh_MFJ3': False, 'rh_RFJ3': False, 'rh_LFJ3': False,
     'rh_THJ2': False, 'rh_THJ4': False, 'rh_THJ5': False}

global lock
lock = threading.Lock()


def create_window():
    """
    Creates a GUI to control the Shadow hand.

    """
    master = Tkinter.Tk()
    # Fingers to move
    joint_vars.extend([
        Tkinter.IntVar(), Tkinter.IntVar(), Tkinter.IntVar(), Tkinter.IntVar(),
        Tkinter.IntVar(), Tkinter.IntVar(), Tkinter.IntVar(), Tkinter.IntVar(),
        Tkinter.IntVar(), Tkinter.IntVar(), Tkinter.IntVar()])

    ff_j2 = Tkinter.Checkbutton(
        master, text="First finger (J2)", variable=joint_vars[0], command=ff_j2_cb)
    mf_j2 = Tkinter.Checkbutton(
        master, text="Middle finger (J2)", variable=joint_vars[1], command=mf_j2_cb)
    rf_j2 = Tkinter.Checkbutton(
        master, text="Ring finger (J2)", variable=joint_vars[2], command=rf_j2_cb)
    lf_j2 = Tkinter.Checkbutton(
        master, text="Little finger (J2)", variable=joint_vars[3], command=lf_j2_cb)

    ff_j3 = Tkinter.Checkbutton(
        master, text="First finger (J3)", variable=joint_vars[4], command=ff_j3_cb)
    mf_j3 = Tkinter.Checkbutton(
        master, text="Middle finger (J3)", variable=joint_vars[5], command=mf_j3_cb)
    rf_j3 = Tkinter.Checkbutton(
        master, text="Ring finger (J3)", variable=joint_vars[6], command=rf_j3_cb)
    lf_j3 = Tkinter.Checkbutton(
        master, text="Little finger (J3)", variable=joint_vars[7], command=lf_j3_cb)

    thumb_j2 = Tkinter.Checkbutton(
        master, text="Thumb (J2)", variable=joint_vars[8], command=th_j2_cb)
    thumb_j4 = Tkinter.Checkbutton(
        master, text="Thumb (J4)", variable=joint_vars[9], command=th_j4_cb)
    thumb_j5 = Tkinter.Checkbutton(
        master, text="Thumb (J5)", variable=joint_vars[10], command=th_j5_cb)

    ff_j2.grid(row=0, column=0)
    mf_j2.grid(row=1, column=0)
    rf_j2.grid(row=2, column=0)
    lf_j2.grid(row=3, column=0)

    ff_j3.grid(row=0, column=1)
    mf_j3.grid(row=1, column=1)
    rf_j3.grid(row=2, column=1)
    lf_j3.grid(row=3, column=1)

    thumb_j2.grid(row=0, column=2)
    thumb_j4.grid(row=1, column=2)
    thumb_j5.grid(row=2, column=2)

    increase_j2_fingers = Tkinter.Button(
        master, command=increase_finger_j2, text="+")
    increase_j2_fingers.grid(row=4, column=5)
    decrease_j2_fingers = Tkinter.Button(
        master, command=decrease_finger_j2, text="-")
    decrease_j2_fingers.grid(row=4, column=6)

    # Event buttons.
    start_button = Tkinter.Button(master, command=start_cb, text="START")
    stop_button = Tkinter.Button(master, command=stop_cb, text="STOP")
    start_button.grid(row=1, column=6)
    stop_button.grid(row=2, column=6)

    master.title("Hand controller")
    master.mainloop()
    rospy.signal_shutdown("GUI closed")


def increase_finger_j2():
    """
    Increases the joint_positions value.

    """
    global lock
    lock.acquire()
    up_joint_position.data = True
    lock.release()


def decrease_finger_j2():
    """
    Decreases the joint_positions value.

    """
    global lock
    lock.acquire()
    down_joint_position.data = True
    lock.release()


def ff_j2_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_FFJ2'] = bool(joint_vars[0].get())
    lock.release()


def mf_j2_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_MFJ2'] = bool(joint_vars[1].get())
    lock.release()


def rf_j2_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_RFJ2'] = bool(joint_vars[2].get())
    lock.release()


def lf_j2_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_LFJ2'] = bool(joint_vars[3].get())
    lock.release()


def ff_j3_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_FFJ3'] = bool(joint_vars[4].get())
    lock.release()


def mf_j3_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_MFJ3'] = bool(joint_vars[5].get())
    lock.release()


def rf_j3_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_RFJ3'] = bool(joint_vars[6].get())
    lock.release()


def lf_j3_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_LFJ3'] = bool(joint_vars[7].get())
    lock.release()


def th_j2_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_THJ2'] = bool(joint_vars[8].get())
    lock.release()


def th_j4_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_THJ4'] = bool(joint_vars[9].get())
    lock.release()


def th_j5_cb():
    """
    Sets the joint to move or not.

    """
    global lock
    lock.acquire()
    joints_to_move['rh_THJ5'] = bool(joint_vars[10].get())
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


def joint_states_cb(msg):
    """
    Obtains the joint states of the hand.

    :param msg: Joint state of the hand.
    :type msg: sensor_msgs.msg.JointState

    """
    joint_states.name = msg.name
    joint_states.position = msg.position


def get_joint_position(joint_states_in, joint_name):
    """
    Returns the joint position for the given joint name.

    :param joint_states_in: Joint state of the hand.
    :type joint_states_in: sensor_msgs.msg.JointState

    :param joint_name: Name to retrieve the joint position.
    :type joint_name: str

    :return: The current joint position for the specified joint name.
    :rtype: float

    """
    try:
        idx = joint_states_in.name.index(joint_name)
        return joint_states_in.position[idx]
    except KeyError:
        rospy.logwarn("Joint state '{}' not found.".format(joint_name))
        return None


def publish_joint_positions():
    """
    Publishes the joint positions.

    """
    # Joint increment/decrement (in radians).
    joint_increments = rospy.get_param('~joint_increments', 0.07)
    # Use simulation.
    use_sim = rospy.get_param('~use_sim', False)
    # Node cycle rate (in Hz).
    loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

    # Publishers
    pub_ffj2 = rospy.Publisher('~ff_j2', std_msgs.msg.Float64, queue_size=10)
    pub_mfj2 = rospy.Publisher('~mf_j2', std_msgs.msg.Float64, queue_size=10)
    pub_rfj2 = rospy.Publisher('~rf_j2', std_msgs.msg.Float64, queue_size=10)
    pub_lfj2 = rospy.Publisher('~lf_j2', std_msgs.msg.Float64, queue_size=10)
    pub_ffj3 = rospy.Publisher('~ff_j3', std_msgs.msg.Float64, queue_size=10)
    pub_mfj3 = rospy.Publisher('~mf_j3', std_msgs.msg.Float64, queue_size=10)
    pub_rfj3 = rospy.Publisher('~rf_j3', std_msgs.msg.Float64, queue_size=10)
    pub_lfj3 = rospy.Publisher('~lf_j3', std_msgs.msg.Float64, queue_size=10)
    pub_thj2 = rospy.Publisher('~th_j2', std_msgs.msg.Float64, queue_size=10)
    pub_thj4 = rospy.Publisher('~th_j4', std_msgs.msg.Float64, queue_size=10)
    pub_thj5 = rospy.Publisher('~th_j5', std_msgs.msg.Float64, queue_size=10)
    pub_event_out = rospy.Publisher('~event_out', std_msgs.msg.String, queue_size=10)

    # Subscribers
    rospy.Subscriber('~joint_states', sensor_msgs.msg.JointState, joint_states_cb)

    # Service clients
    if not use_sim:
        controller_to_switch = rospy.get_param('~switch_controller', '/controller_manager/switch_controller')
        rospy.loginfo("Waiting for service '{}'...".format(controller_to_switch))
        rospy.wait_for_service(controller_to_switch)
        switch_controller = rospy.ServiceProxy(
            controller_to_switch, controller_manager_msgs.srv.SwitchController)
        rospy.loginfo("Found service '{}'.".format(controller_to_switch))

        # Trajectory controller.
        trajectory_controller = rospy.get_param('~trajectory_controller', 'rh_trajectory_controller')

    running = False

    publishers_map = {
        'rh_FFJ2': pub_ffj2, 'rh_MFJ2': pub_mfj2, 'rh_RFJ2': pub_rfj2, 'rh_LFJ2': pub_lfj2,
        'rh_FFJ3': pub_ffj3, 'rh_MFJ3': pub_mfj3, 'rh_RFJ3': pub_rfj3, 'rh_LFJ3': pub_lfj3,
        'rh_THJ2': pub_thj2, 'rh_THJ4': pub_thj4, 'rh_THJ5': pub_thj5}

    while not rospy.is_shutdown():
        selected_joints = [kk for kk, vv in joints_to_move.iteritems() if vv]

        if running:
            for joint in selected_joints:
                joint_position = get_joint_position(joint_states, joint)
                if joint_positions is None:
                    pass
                if up_joint_position.data:
                    publishers_map[joint].publish(joint_position + joint_increments)
                if down_joint_position.data:
                    publishers_map[joint].publish(joint_position - joint_increments)
            up_joint_position.data = False
            down_joint_position.data = False

        if event.data == 'e_start':
            if use_sim:
                running = True
            else:
                try:
                    req = controller_manager_msgs.srv.SwitchControllerRequest()
                    req.strictness = req.STRICT
                    req.stop_controllers = [trajectory_controller]
                    res = switch_controller(req)
                    if res.ok:
                        rospy.loginfo("Successfully stopped trajectory controller.")
                        running = True
                    else:
                        rospy.logerr("Couldn't stop trajectory controller.")
                except rospy.ServiceException, e:
                    rospy.logwarn("Service call failed: {}".format(e))

        elif event.data == 'e_stop':
            if use_sim:
                running = False
            else:
                try:
                    req = controller_manager_msgs.srv.SwitchControllerRequest()
                    req.strictness = req.STRICT
                    req.start_controllers = [trajectory_controller]
                    res = switch_controller(req)
                    if res.ok:
                        rospy.loginfo("Successfully started trajectory controller.")
                        running = False
                    else:
                        rospy.logerr("Couldn't start trajectory controller.")
                except rospy.ServiceException, e:
                    rospy.logwarn("Service call failed: {}".format(e))

        pub_event_out.publish(event)
        event.data = ""
        loop_rate.sleep()


def main():
    rospy.init_node('grasp_deformable_object')

    import thread
    try:
        thread.start_new_thread(create_window, tuple())
        publish_joint_positions()
    except rospy.ROSInterruptException:
        pass
