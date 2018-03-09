#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node moves a specified finger of the Shadow Hand Robot down N times with
M different configurations (e.g. to apply different force_profiles) to make contact with
a probe mounted on a force-torque sensor. It moves the finger down N * M times and
then it waits for the user to input a new location to move the finger again N * M times.
Additionally, it records the output of the tactile sensor as well as the output of
the force-torque sensor as .bag file with a filename based on the finger, probe,
contact location, force profile and trial number. The recording starts before contact
and ends after contact.

**Input(s):**
  * `tactile_data`: The tactile data for all the sensors in the hand.
    - *type:* `sr_robot_msgs/BiotacAll`
  * `wrench_data`: The output of the force-torque sensor.
    - *type:* `geometry_msgs/WrenchStamped`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`
  * `finger_controller`: Commands a joint position to the specified finger.
    - *type:* `std_msgs/Float64`

**Parameter(s):**
  * `finger`: Name of the finger to be used in the experiments (see config file).
  * `probe`: Name of the probe to be used in the experiments (see config file).
  * `number_of_trials`: Times the finger would be moved into contact for the same location
    and force profile (e.g. low, mid and high).
  * `duration`: Duration the finger should be in contact with the probe (in seconds).
  * `file_path`: File path where recording is going to be saved.
  * `topics`: Topics to be recorded (see config file).
  * `loop_rate`: Node cycle rate (in Hz).

"""

import os
import os.path
import signal
import subprocess
import yaml
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sr_robot_msgs.msg


class FingerExperimentNode(object):
    """
    Records the tactile and force-torque data and controls the finger's motion
    to perform the contact experiment.

    """
    def __init__(self):
        """
        Instantiates the node.

        :return: Node to perform a finger contact experiment.
        :rtype: FingerExperimentNode

        """
        # Params
        self.event = None
        self.tactile_data = None
        self.wrench_data = None

        self.config_file = rospy.get_param("~config_file", None)
        assert self.config_file is not None, "A configuration file must be specified."
        self.config_params = yaml.load(open(self.config_file, "r"))

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 50))
        # If true, it records the desired topics.
        self.logging = rospy.get_param('~logging', False)
        # Name of the finger to be used in the experiments (see config file).
        self.finger = rospy.get_param('~finger', 'ff')
        # Name of the probe to be used in the experiments (see config file).
        probe = rospy.get_param('~probe', 'probe_flat_3')
        self.probe = self.config_params[probe]
        # Times the finger would be moved into contact for the same location and
        #  force_profile.
        self.number_of_trials = rospy.get_param('~number_of_trials', 5)
        # Duration the finger should be in contact with the probe (in seconds).
        self.duration = rospy.get_param('~duration', 3)
        # File path where recording is going to be saved.
        self.file_path = rospy.get_param('~file_path', None)
        assert self.file_path is not None, "A file path must be specified."
        # Topics to be recorded (see config file).
        topics_file = rospy.get_param('~topics')
        stream = open(topics_file, 'r')
        self.topics = yaml.load(stream)
        # The process executing the rosbag recording.
        self.rosbag_process = None
        # These profiles serve to relate a joint configuration to a specific
        # force contact (i.e. the further a joint moves, towards the probe, the
        # higher the force will be).
        self.force_profiles = ['low', 'mid', 'high']

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.finger_controller = rospy.Publisher(
            self.config_params[self.finger], std_msgs.msg.Float64, queue_size=10)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber("~tactile_data", sr_robot_msgs.msg.BiotacAll, self.tactile_data_cb)
        rospy.Subscriber("~wrench_data", geometry_msgs.msg.WrenchStamped, self.wrench_data_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def tactile_data_cb(self, msg):
        """
        Obtains the tactile data from the tactile sensors.

        :param msg: Tactile data.
        :type msg: sr_robot_msgs.msg.BiotacAll

        """
        self.tactile_data = msg

    def wrench_data_cb(self, msg):
        """
        Obtains the wrench data from the force-torque sensor.

        :param msg: Wrench data.
        :type msg: geometry_msgs.msg.WrenchStamped

        """
        self.wrench_data = msg

    def start(self):
        """
        Starts the node.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():
            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'IDLE'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event_out.publish('e_stopped')
            self.reset_component_data()
            return 'INIT'
        elif self.tactile_data and self.wrench_data:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.event_out.publish('e_stopped')
            self.reset_component_data()
            return 'INIT'
        else:
            self.run_experiment()
            self.reset_component_data()
            return 'IDLE'

    def run_experiment(self):
        """
        Moves the finger in order to make contact with the probe and it maintains
        the contact for the specified duration. It also records the tactile and wrench
        data starting from before the finger moves into contact with the probe and
        ending when the finger has return to its original position (i.e. without
        contacting the probe).

        """
        while not (rospy.is_shutdown() or self.event == 'e_stop'):
            location = raw_input("Enter location (or 'n' to stop): ")
            if location.lower() == 'n':
                rospy.loginfo("Finished experiment.")
                self.reset_component_data()
                break
            raw_input("Ready to start with trial *{0}_{1}_loc-{2}*\n(press enter)? ".format(
                self.finger, self.probe, location))
            for force in self.force_profiles:
                for trial in range(1, self.number_of_trials + 1):
                    self.event_out.publish('e_running')
                    filename = "{0}_{1}_loc-{2}_force-{3}_trial-{4}".format(
                        self.finger, self.probe, location, force, trial)
                    rospy.loginfo("\nExecuting trial: {}".format(filename))
                    if self.logging:
                        self.start_recording(filename)

                    joint_value = self.config_params[self.finger + '/' + force]
                    self.finger_controller.publish(std_msgs.msg.Float64(data=joint_value))
                    rospy.loginfo(
                        "Keeping contact for {} seconds...".format(self.duration))
                    rospy.sleep(self.duration)
                    joint_value = self.config_params[self.finger + '/zero']
                    self.finger_controller.publish(std_msgs.msg.Float64(data=joint_value))
                    # Hack to allow enough time for the finger to move back.
                    rospy.sleep(1.0)

                    if self.logging:
                        self.stop_recording()
                        rospy.logwarn("Stopped recording.")

    def start_recording(self, filename):
        """
        Starts the recording of the topics in a .bag file.

        :param filename: Name to record the rosbag file.
        :type filename: str

        """
        filename += '.bag'
        if not os.path.isdir(os.path.expanduser(self.file_path)):
            os.mkdir(os.path.expanduser(self.file_path))
        full_path = os.path.expanduser(os.path.join(self.file_path, filename))
        rosbag_command = 'rosbag record ' + self.topics + ' -O ' + full_path
        self.rosbag_process = subprocess.Popen(rosbag_command.split(' '))

    def stop_recording(self):
        """
        Stops the rosbag recording.

        """
        if self.rosbag_process:
            self.rosbag_process.send_signal(subprocess.signal.SIGINT)
            self.terminate_process(self.rosbag_process)
            self.rosbag_process = None
            rospy.loginfo("Killed rosbag process.")

    @staticmethod
    def terminate_process(p):
        """
        Kills the process 'p' and its children (sub-processes).

        Code by Sergey Alexandrov:
        http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/

        :param p: Process to terminate.
        :type p: subprocess.Popen

        """
        ps_command = subprocess.Popen(
            "ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
        ps_output = ps_command.stdout.read()
        return_code = ps_command.wait()
        assert return_code == 0, "ps command returned %d" % return_code
        for pid_str in ps_output.split("\n")[:-1]:
                os.kill(int(pid_str), signal.SIGINT)
        p.terminate()

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.tactile_data = None
        self.wrench_data = None


def main():
    rospy.init_node("finger_experiment_node", anonymous=True)
    finger_experiment_node = FingerExperimentNode()
    finger_experiment_node.start()
