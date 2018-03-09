# UCR Data collection
This package contains nodes that are used for data collection in experiments.
___
___
## The *finger_data_experiment* node
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
  * `filename`: Name to record the rosbag file.
    - *type:* `std_msgs/String`
  * `recorder_trigger`: Start/stops the recording of teh bag file.
    - *type:* `std_msgs/String`
  * `finger_controller`: Commands a joint position to the specified finger.
    - *type:* `std_msgs/Float64`

**Parameter(s):**
  * `finger`: Name of the finger to be used in the experiments (see config file).
  * `probe`: Name of the probe to be used in the experiments (see config file).
  * `number_of_trials`: Times the finger would be moved into contact for the same location
    and force profile (e.g. low, mid and high).
  * `duration`: Duration the finger should be in contact with the probe (in seconds).
  * `loop_rate`: Node cycle rate (in Hz).

---
### Usage
**1. Launch the component (example showing the optional 'finger' parameter):**

```
roslaunch ucr_data_collection finger_data_experiment.launch finger:=ff 
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the tactile and wrench data (example):**

 **2.1. Alternative you can publish them manually (example):**

```
rostopic pub -r 50 /finger_data_experiment/tactile_data sr_robot_msgs/BiotacAll '{tactiles: [{pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [2551, 3178, 3223, 3078, 3159, 2911, 2744, 2842, 2859, 2946, 2602, 3173, 3196, 3003, 3126, 2820, 3088, 3163, 2932]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}]}' 
```

```
rostopic pub -r 50 /finger_data_experiment/wrench_data geometry_msgs/WrenchStamped 'wrench: {force:  {x: 0.1, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}'
```

**3. Start the component:**

```
rostopic pub /finger_data_experiment/event_in std_msgs/String "e_start" 
```

**4. To stop the component:**

```
rostopic pub /finger_data_experiment/event_in std_msgs/String "e_stop"
```
___
___
## The *rosbag_recorder* node
Based on this script:
https://github.com/mas-group/robocup-at-work/blob/c5cb9fe903a5214b4221fd2190d44acb8d077e1d/mas_common_robotics/mcr_tools/mcr_rosbag_recorder/ros/src/mcr_rosbag_recorder/rosbag_recorder.py

This node records the specified topics in a .bag file.

**Input(s):**
  * `filename`: Name to save the recorded file.
    - *type:* `std_msgs/String`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).


---
### Usage
**1. Launch the component (example):**

```
roslaunch ucr_data_collection rosbag_recorder.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the tactile and wrench data (example):**

 **3.1. Alternative you can publish them manually (example):**

```
rostopic pub -r 50 /finger_data_experiment/tactile_data sr_robot_msgs/BiotacAll '{tactiles: [{pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [2551, 3178, 3223, 3078, 3159, 2911, 2744, 2842, 2859, 2946, 2602, 3173, 3196, 3003, 3126, 2820, 3088, 3163, 2932]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}]}' 
```

```
rostopic pub -r 50 /finger_data_experiment/wrench_data geometry_msgs/WrenchStamped 'wrench: {force:  {x: 0.1, y: 0.0, z: 0.0}, torque: {x: 0.0, y: 0.0, z: 0.0}}'
```

**4. Start the component:**

```
rostopic pub /rosbag_recorder/filename std_msgs/String "filename" 
```

**4. Start the component:**

```
rostopic pub /rosbag_recorder/event_in std_msgs/String "e_start" 
```

**5. To stop the component:**

```
rostopic pub /rosbag_recorder/event_in std_msgs/String "e_stop"
```
