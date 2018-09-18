# UDOM Pose control
This package contains controllers to regulate the distance between two poses.

------

## The *pose_controller* node
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

------

### Usage
**1. Launch the component (example):**

```
roslaunch udom_pose_control pose_controller_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /pose_controller/twist_out
```

**3. Publish to the desired topics (example):**

 ```
 rostopic pub /pose_controller/current_pose geometry_msgs/PoseStamped "{header: {frame_id: 'base_frame'}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}"
 ```

 ```
 rostopic pub /pose_controller/target_pose geometry_msgs/PoseStamped "{header: {frame_id: 'base_frame'}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}"
 ```

**4. Start the component:**

```
rostopic pub /pose_controller/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /pose_controller/event_in std_msgs/String "e_stop"
```

------

### Usage (interactive)
**1. Launch the component (example):**

```
roslaunch udom_pose_control pose_controller_interactive.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /pose_controller/twist_out
```

**3. Start the component:**

```
rostopic pub /pose_controller/event_in std_msgs/String "e_start"
```

**4. To stop the component:**

```
rostopic pub /pose_controller/event_in std_msgs/String "e_stop"
```

------

### Usage (interactive with simulated robot)
**1. Launch the component (example):**

```
roslaunch udom_pose_control pose_controller_interactive_kuka_sim.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /pose_controller/twist_out
```

**3. Switch to velocity controller:**

```
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['joint_velocity_controller'], stop_controllers: [], strictness: 2}"
```

**4. Obtain the pose of the end-effector:**

```
rostopic pub /transform_to_pose_converter/event_in std_msgs/String "data: 'e_start'"
```

**5. Start the component:**

```
rostopic pub /pose_controller/event_in std_msgs/String "e_start"
```

**6. To stop the component:**

```
rostopic pub /pose_controller/event_in std_msgs/String "e_stop"
```

------

### Usage (interactive with real robot)
**0. Make sure the KUKA robot is running.**

**1. Launch the component (example):**

```
roslaunch udom_pose_control pose_controller_dq_interactive_kuka_real.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /pose_controller/twist_out
```

**3. Switch to velocity controller:**

```
rosservice call /kuka_lwr_right/controller_manager/switch_controller "{start_controllers: ['joint_velocity_controller'], stop_controllers: [], strictness: 2}"
```

**4. Obtain the pose of the end-effector:**

```
rostopic pub /transform_to_pose_converter/event_in std_msgs/String "data: 'e_start'"
```

**5. Start the component:**

```
rostopic pub /pose_controller/event_in std_msgs/String "e_start"
```

**6. To stop the component:**

```
rostopic pub /pose_controller/event_in std_msgs/String "e_stop"
```

------