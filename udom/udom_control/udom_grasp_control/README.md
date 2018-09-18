# UDOM Grasping sensing
This package contains nodes for different grasp control strategies.
  
## The *reactive_grasp* node
This node controls the fingers of the Shadow hand so that they grasp an object without
exceeding a force threshold. If excessive force is applied on a finger, that finger will
move backwards in order to reduce the grasp force.

**Assumptions:**
  * It assumes five BioTac sensors as the input sensors.
  * It also assumes the joint positions of the fingers to be available.

**Input(s):**

  * `tactile_data`: Contact information of each fingertip (5x).
    - *type:* `udom_perception_msgs/ContactInfo`

  * `joint_states`: Joint position values of all five fingers.
    - *type:* `sensor_msgs/JointState`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_reset`: resets the hand to its initial open configuration.

    - *type:* `std_msgs/String`

**Output(s):**

  * `joint_info`: Joint position value for each finger (5x).
    - *type:* `std_msgs/Float64`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

### Usage
**1. Launch the component (example):**

```
roslaunch udom_grasp_control reactive_grasp.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Publish to the desired topic (example):**

```
roscd udom_grasp_control/data/
```

```
rosbag play -l example.bag
```

**4. Start the component:**

```
rostopic pub /udom_grasp_control/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /udom_grasp_control/event_in std_msgs/String "e_stop"
```

**6. To reset the hand to its initial open configuration:**

```
rostopic pub /udom_grasp_control/event_in std_msgs/String "e_reset"
```
