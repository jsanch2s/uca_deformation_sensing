# UDOM Grasping sensing
This package contains nodes to demonstrate different grasping strategies.
  
## The *reactive_grasp_demo* node
This node uses a pipeline of components to perform a reactive grasp based on
the tactile data obtained from the fingertips of the Shadow hand (e.g. to apply
a desired grasp force on the object).

It uses the following nodes:


  * `udom_topic_tools/tactile_demux`

  * `udom_sensor_model/tactile_sensor_model`

  * `udom_grasp_control/reactive_grasp`

**Assumptions:**
  * It assumes five BioTac sensors and the joint_states of the Shadow hand as input.

**Input(s):**

  * `tactile_info`: The output data of five BioTac sensors.

    - *type:* `sr_robot_msgs/BiotacAll`

  * `joint_states`: The joints' state of the Shadow hand.

    - *type:* `sensor_msgs/JointState`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_reset`: resets the hand to its initial open configuration.

    - *type:* `std_msgs/String`

**Output(s):**
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `force_threshold_max`: Maximum force, if the current force of any finger exceeds this
        value, then the finger will move backwards (in N).

  * `force_threshold_min`: Minimum force, if the current force of any finger is less than
        this value, then the finger will move forwards (in N).

  * `increment`: The value of the joint position increment to move each finger (in radians).

  * `increment_thumb`: The value of the joint position increment to move the thumb
        (in radians).

  * `force_feedback`: Whether to use force feedback or not.

### Usage
**1. Launch the component (example):**

```
roslaunch udom_grasping reactive_grasping_demo.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Publish to the desired topic (example):**

```
roscd udom_grasping/data/
```

```
rosbag play -l example.bag
```

**4. Start the component:**

```
rostopic pub /udom_reactive_grasping/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /udom_reactive_grasping/event_in std_msgs/String "e_stop"
```

**6. To reset the hand to its initial open configuration:**

```
rostopic pub /udom_reactive_grasping/event_in std_msgs/String "e_reset"
```
