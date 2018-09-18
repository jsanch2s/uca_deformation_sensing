#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
This node outputs a twist such that the distance between two poses is minimized.
The following type of controllers are available:

 * `simple`: This controller uses the difference between two poses and then applies
 linear and angular proportional gains to generate a twist.

 * `dual_quaternion`: Dual quaternion control based on [1].

[1] Özgür, Erol, and Youcef Mezouar. "Kinematic modeling and control of a robot arm
using unit dual quaternions." Robotics and Autonomous Systems 77 (2016): 66-73.

**Input(s):**
  * `current_pose`: Current pose to be moved towards the target pose.
    - *type:* `geometry_msgs/PoseStamped`

  * `target_pose`: Target pose to move the current pose towards.
    - *type:* `geometry_msgs/PoseStamped`

  * `event_in`: The desired event for the node:
      `e_start`: starts the component.
      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `twist_out`: The twist that minimizes the distance between the two poses.
    - *type:* `geometry_msgs/TwistStamped`

  * `event_out`: The current event of the node.
      `e_running`: when the component is running.
      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).

  * `wait_for_transform`:Maximum duration to wait for a transform (in seconds).

  * `controller_type`: The type of controller to use: {simple, dual_quaternion}.

  * `reference_frame`: The twist will be described with respect to this frame.

  * `gains`: A six-dimensional vector with the proportional gains of each dimension.

  * `limits`: The velocity limits for each dimension.

  * `linear_tolerance`: Linear tolerance for all dimensions (in meters).

  * `angular_tolerance`: Angular tolerance for all dimensions (in degrees).

  * `sync`: If true, it will synchronize all velocities such that they reach the target
  at the same time.

"""

import copy
import numpy as np
import rospy
import std_msgs.msg
import geometry_msgs.msg
import tf
import tf2_ros
import udom_geometric_transformation.transformation_utils as transformation_utils
import udom_geometric_transformation.quaternion_algebra as quat_algebra


class PoseController(object):
    """
    Outputs a twist such that the distance between two poses is minimized.

    """
    def __init__(self):
        """
        Returns a pose controller.

        :return: Pose controller
        :rtype: PoseController

        """
        # Params
        self.available_controllers = ['simple', 'dual_quaternion']

        self.event = None
        self.current_pose = None
        self.target_pose = None

        # Object to compute transformations.
        self.listener = transformation_utils.GeometryTransformer()

        # Maximum duration to wait for a transform (in seconds).
        self.wait_for_transform = rospy.get_param('~wait_for_transform', 0.1)

        # The type of controller to use.
        self.controller_type = rospy.get_param("~controller_type", 'simple')
        assert self.controller_type in \
            self.available_controllers, "{} is not an available controller. " \
            "Available controllers are: {}".format(
                self.controller_type, self.available_controllers)

        # The twist will be described with respect to this frame.
        self.reference_frame = rospy.get_param("~reference_frame", None)
        assert self.reference_frame is not None, "A reference frame must be specified."

        # Proportional gains for each of the six dimensions.
        self.gains = rospy.get_param('~gains', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

        # Velocity limits for each of the six dimensions (in m/s, rad/s).
        self.limits = rospy.get_param('~limits', [1.0, 1.0, 1.0, 0.2, 0.2, 0.2])

        # Whether to synchronize all velocities such that they reach the target
        # at the same time.
        self.sync = rospy.get_param('~sync', False)

        # Linear tolerance (in meters).
        self.linear_tolerance = rospy.get_param('~linear_tolerance', 0.01)

        # Angular tolerance (in degrees).
        self.angular_tolerance = rospy.get_param('~angular_tolerance', 3.0)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 100))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=10)
        self.twist_out = rospy.Publisher(
            "~twist_out", geometry_msgs.msg.TwistStamped, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "~current_pose", geometry_msgs.msg.PoseStamped, self.current_pose_cb)
        rospy.Subscriber("~target_pose", geometry_msgs.msg.PoseStamped, self.target_pose_cb)

    def event_in_cb(self, msg):
        """
        Obtains the event for the node (e.g. start, stop).

        :param msg: Event message for the node.
        :type msg: std_msgs.msg.String

        """
        self.event = msg.data

    def current_pose_cb(self, msg):
        """
        Obtains the current pose.

        :param msg: Current pose.
        :type msg: geometry_msgs.msg.PoseStamped

        """
        self.current_pose = msg

    def target_pose_cb(self, msg):
        """
        Obtains the target pose.

        :param msg: Target pose.
        :type msg: geometry_msgs.msg.PoseStamped

        """
        self.target_pose = msg

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
            rospy.logdebug("event: {0}".format(self.event))
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
            self.twist_out.publish(self.zero_twist())
            return 'INIT'
        elif (self.current_pose is not None) and (self.target_pose is not None):
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
            self.twist_out.publish(self.zero_twist())
            return 'INIT'
        else:
            if self.target_pose.header.frame_id != self.reference_frame:
                target_pose = self.listener.transform_pose(
                    self.target_pose, self.reference_frame, self.wait_for_transform)
            else:
                target_pose = copy.deepcopy(self.target_pose)
            if self.current_pose.header.frame_id != self.reference_frame:
                current_pose = self.listener.transform_pose(
                    self.current_pose, self.reference_frame, self.wait_for_transform)
            else:
                current_pose = copy.deepcopy(self.current_pose)

            if self.controller_type == 'dual_quaternion':
                try:
                    twist_out, error = self.compute_twist_dq(target_pose, current_pose)
                except AssertionError, e:
                    rospy.logerr("Error while computing twist:\n{}".format(e.message))
                    twist_out = self.zero_twist()
                    error = np.zeros(6)
            else:
                twist_out, error = self.compute_twist(target_pose, current_pose)
            if self.within_threshold(error):
                twist_out = self.zero_twist()
            transformed_twist = self.transform_twist(twist_out, error)
            if transformed_twist:
                self.event_out.publish('e_running')
                self.twist_out.publish(transformed_twist)
            else:
                rospy.logwarn("Error while computing controlled twist.")
            self.reset_component_data()
            return 'IDLE'

    def compute_twist(self, target_pose, current_pose):
        """
        Computes the twist required to minimize the distance between the target and
        current poses. It also returns the Cartesian error (linear and angular) as
        a list. Note that the poses must be in the same frame.

        :param target_pose: The target pose.
        :type target_pose: geometry_msgs.msg.PoseStamped

        :param current_pose: The current pose.
        :type current_pose: geometry_msgs.msg.PoseStamped

        :return: The computed twist and the Cartesian error (in meters and radians).
        :rtype: geometry_msgs.msg.TwistStamped or None in case of an error, list

        """
        twist_out = geometry_msgs.msg.TwistStamped()

        # Compute Cartesian error.
        error = compute_cartesian_error(target_pose, current_pose)

        twist_out.twist.linear.x = error[0] * self.gains[0]
        twist_out.twist.linear.y = error[1] * self.gains[1]
        twist_out.twist.linear.z = error[2] * self.gains[2]
        twist_out.twist.angular.x = error[3] * self.gains[3]
        twist_out.twist.angular.y = error[4] * self.gains[4]
        twist_out.twist.angular.z = error[5] * self.gains[5]

        twist_out.header.stamp = rospy.Time.now()
        twist_out.header.frame_id = target_pose.header.frame_id

        return twist_out, error

    def compute_twist_dq(self, target_pose, current_pose):
        """
        Computes the twist required to minimize the distance between the target and
        current poses using dual quaternions as described in [1]. It also returns the
        Cartesian error (linear and angular) as a list. Note that the poses must be
        in the same frame.

        [1] Özgür, Erol, and Youcef Mezouar. "Kinematic modeling and control of a robot arm
        using unit dual quaternions." Robotics and Autonomous Systems 77 (2016): 66-73.

        :param target_pose: The target pose.
        :type target_pose: geometry_msgs.msg.PoseStamped

        :param current_pose: The current pose.
        :type current_pose: geometry_msgs.msg.PoseStamped

        :return: The computed twist and the Cartesian error (in meters and radians).
        :rtype: geometry_msgs.msg.TwistStamped or None in case of an error, list

        """
        twist_out = geometry_msgs.msg.TwistStamped()

        # Transform poses into dual quaternions.
        current_pose_dq = quat_algebra.pose_to_dq(current_pose.pose)
        target_pose_dq = quat_algebra.pose_to_dq(target_pose.pose)

        # Compute error in dual quaternion.
        error_dq = quat_algebra.multiply_dq(
            current_pose_dq, quat_algebra.conjugate_dq(target_pose_dq))

        twist_out.twist = quat_algebra.dq_to_twist(error_dq)
        twist_out.twist.linear.x *= -self.gains[0]
        twist_out.twist.linear.y *= -self.gains[1]
        twist_out.twist.linear.z *= -self.gains[2]
        twist_out.twist.angular.x *= -self.gains[3]
        twist_out.twist.angular.y *= -self.gains[4]
        twist_out.twist.angular.z *= -self.gains[5]

        twist_out.header.stamp = rospy.Time.now()
        twist_out.header.frame_id = target_pose.header.frame_id

        error = compute_cartesian_error(target_pose, current_pose)

        return twist_out, error

    def transform_twist(self, twist_in, error):
        """
        Transforms the twist to be expressed with respect to the reference frame and
        limits it values for safety reasons. Optionally, it can synchronize the twist
        such that all dimensions reach the target at the same time.

        :param twist_in: The twist to transform and limit.
        :type twist_in: geometry_msgs.msg.TwistStamped

        :param error: The Cartesian error (linear and angular) in meters and radians.
        :type error: list

        :return: The computed twist.
        :rtype: geometry_msgs.msg.TwistStamped or None in case of an error.

        """
        twist_out = geometry_msgs.msg.TwistStamped()

        try:
            twist_transformed = self.listener.transform_twist(
                twist_in, self.reference_frame, self.wait_for_transform)

            v_x = limit_value(twist_transformed.twist.linear.x, self.limits[0])
            v_y = limit_value(twist_transformed.twist.linear.y, self.limits[1])
            v_z = limit_value(twist_transformed.twist.linear.z, self.limits[2])
            w_x = limit_value(twist_transformed.twist.angular.x, self.limits[3])
            w_y = limit_value(twist_transformed.twist.angular.y, self.limits[4])
            w_z = limit_value(twist_transformed.twist.angular.z, self.limits[5])

            if self.sync:
                v_x, v_y, v_z, w_x, w_y, w_z = self.synchronize_velocities(
                    error, [v_x, v_y, v_z, w_x, w_y, w_z])

            twist_out.twist.linear.x = v_x
            twist_out.twist.linear.y = v_y
            twist_out.twist.linear.z = v_z
            twist_out.twist.angular.x = w_x
            twist_out.twist.angular.y = w_y
            twist_out.twist.angular.z = w_z

            twist_out.header.stamp = rospy.Time.now()
            twist_out.header.frame_id = twist_transformed.header.frame_id

            return twist_out
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Error while transforming frames:\n{}".format(e))
            return None

    def zero_twist(self):
        """
        Returns a zero twist to stop the arm's motion.

        :return: Zero twist
        :rtype: geometry_msgs.msg.TwistStamped

        """
        twist_out = geometry_msgs.msg.TwistStamped()
        twist_out.twist.linear.x = 0.0
        twist_out.twist.linear.y = 0.0
        twist_out.twist.linear.z = 0.0
        twist_out.twist.angular.x = 0.0
        twist_out.twist.angular.y = 0.0
        twist_out.twist.angular.z = 0.0

        twist_out.header.stamp = rospy.Time.now()
        twist_out.header.frame_id = self.reference_frame
        return twist_out

    def within_threshold(self, error):
        """
        Limits a value if it is greater than the limit, or lower than the negative limit.

        :param error: Cartesian error (linear and angular) in meters and radians error.
        :type error: list

        :return: True, if both linear and angular error are within the tolerances.
        :rtype: bool

        """
        within_linear = np.allclose(error[:3], np.zeros(3), atol=self.linear_tolerance)
        within_angular = np.allclose(
            error[3:], np.zeros(3), atol=np.radians(self.angular_tolerance))
        if within_linear and within_angular:
            return True
        else:
            return False

    def synchronize_velocities(self, error, velocity):
        """
        Synchronizes each of the linear and angular velocities such that they reach their
        target at the same time.

        :param error: The linear and angular error.
        :type error: list

        :param velocity: The linear and angular velocities.
        :type velocity: list

        :return: The synchronized velocities.
        :rtype: list

        """
        durations = [
            calculate_duration(ee, vv, self.linear_tolerance)
            for ee, vv in zip(error[:3], velocity[:3])]
        angular_durations = [
            calculate_duration(ee, vv, np.radians(self.angular_tolerance))
            for ee, vv in zip(error[3:], velocity[3:])]

        durations.extend(angular_durations)
        max_time = max(durations)

        synced_velocities = [
            synchronize_velocity(ee, vv, max_time, self.linear_tolerance)
            for ee, vv in zip(error[:3], velocity[:3])]
        synced_angular = [
            synchronize_velocity(ee, vv, max_time, np.radians(self.angular_tolerance))
            for ee, vv in zip(error[3:], velocity[3:])]

        synced_velocities.extend(synced_angular)
        return synced_velocities

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.current_pose = None
        self.target_pose = None
        self.event = None


def compute_cartesian_error(target, current):
    """
    Computes the Cartesian error between poses. The linear part is expressed in meters
    and the angular part is expressed in radians. It assumes both poses are expressed
    in the same frame of reference.

    :param target: The target pose.
    :type target: geometry_msgs.msg.PoseStamped

    :param current: The current pose.
    :type current: geometry_msgs.msg.PoseStamped

    :return: Cartesian error.
    :rtype: list

    """
    assert target.header.frame_id == current.header.frame_id, "" \
        "Poses must be expressed in the same frame. {} (target) =! {} (current)".format(
            target.header.frame_id, current.header.frame_id)

    error_linear_x = target.pose.position.x - current.pose.position.x
    error_linear_y = target.pose.position.y - current.pose.position.y
    error_linear_z = target.pose.position.z - current.pose.position.z

    current_quaternion = [
        current.pose.orientation.x, current.pose.orientation.y,
        current.pose.orientation.z, current.pose.orientation.w]
    target_quaternion = [
        target.pose.orientation.x, target.pose.orientation.y,
        target.pose.orientation.z, target.pose.orientation.w]

    # Convert quaternions into roll, pitch, yaw angles.
    current_angles = tf.transformations.euler_from_quaternion(current_quaternion)
    target_angles = tf.transformations.euler_from_quaternion(target_quaternion)

    # Calculate angular distances.
    error_angular_x = target_angles[0] - current_angles[0]
    error_angular_y = target_angles[1] - current_angles[1]
    error_angular_z = target_angles[2] - current_angles[2]

    return [error_linear_x, error_linear_y, error_linear_z,
            error_angular_x, error_angular_y, error_angular_z]


def calculate_duration(distance, speed, zero):
    if abs(speed) <= zero:
        return 0.0
    return abs(float(distance) / speed)


def synchronize_velocity(dist, vel, max_time, zero):
    if abs(max_time) <= zero:
        return 0.0
    return abs(float(dist) / max_time) * cmp(vel, 0)


def limit_value(value, limit):
    """
    Limits a value if it is greater than the limit, or lower than the negative limit.

    :param value: The input value.
    :type value: float

    :param limit: Maximum and minimum (as negative maximum) value allowed.
    :type limit: float

    :return: The limited value if exceeds the specified limit, or the input value.
    :rtype: float

    """
    return np.sign(value) * min(abs(value), limit)


def main():
    rospy.init_node("pose_controller", anonymous=True)
    pose_controller = PoseController()
    pose_controller.start()
