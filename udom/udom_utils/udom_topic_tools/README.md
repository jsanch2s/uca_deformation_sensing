# UDOM Topic tools
This package contains nodes to manipulate ROS topics. For instance, for directing,
merging or selecting topics.
___
___
## The *force_merger* node
This node combines the messages (udom_common_msgs.msg.ForceArray) from various subscribed
topics into a single udom_common_msgs.msg.ForceMultiArray topic which is then published.

**Input(s):**
  * `contact_info`: The messages, as array of forces, of each sensor to be merged.
    - *type:* `udom_common_msgs/ForceArray`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `force_multi_array`: A single message containing the array of forces for each sensor.
    - *type:* `udom_common_msgs/ForceMultiArray`
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `config_file`: Specifies which topics the node should subscribe to.
  * `loop_rate`: Node cycle rate (in Hz).
---
### Usage
**1. Launch the component (example):**

```
roslaunch udom_topic_tools force_merger_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /force_merger/force_multi_array
```
**3. Subscribe to the desired topics:**

This should be specified in a `yaml` file in the `config` directory (see `config/example.yaml`).

 **3.1. Alternative you can publish them manually (example):**

 ```
 rostopic pub /force_merger/force_array_1 udom_common_msgs/ForceArray "{header: {frame_id: 'f1'}, positions: [{x: 1.0, y: 1.0, z: 1.0}], wrenches: [{force: {x: 1.0, y: 1.0, z: 1.0}}]}"
 ```

 ```
 rostopic pub /force_merger/force_array_2 udom_common_msgs/ForceArray "{header: {frame_id: 'f2'}, positions: [{x: 2.0, y: 2.0, z: 2.0}], wrenches: [{force: {x: 2.0, y: 2.0, z: 2.0}}]}"
 ```
**4. Start the component:**

```
rostopic pub /force_merger/event_in std_msgs/String "e_start"
```
**5. To stop the component:**

```
rostopic pub /force_merger/event_in std_msgs/String "e_stop"
```
___
___
## The *tactile_demux* node
This node selects a single general tactile data message (BioTacAll), specified
by a given sensor index, and publishes it as tactile stamped message.

**Input(s):**
  * `tactile_data_in`: The tactile data for all the sensors in the hand.
    - *type:* `sr_robot_msgs/BiotacAll`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `tactile_data_out`: A stamped tactile message for the selected sensor.
    - *type:* `udom_perception_msgs/BiotacStamped`
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `sensor_index`: Index of the sensor to be selected.
  * `sensor_frame`: Reference frame of the sensor to be selected.
  * `loop_rate`: Node cycle rate (in Hz).
---
### Usage
**1. Launch the component (example):**

```
roslaunch udom_topic_tools tactile_demux_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /tactile_demux/tactile_data_out
```
**3. Publish the tactile data (example):**

 ```
 rostopic pub -r 10 /tactile_demux/tactile_data_in sr_robot_msgs/BiotacAll '{tactiles: [{pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [2551, 3178, 3223, 3078, 3159, 2911, 2744, 2842, 2859, 2946, 2602, 3173, 3196, 3003, 3126, 2820, 3088, 3163, 2932]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}, {pac0: 0, pac1: 0, pdc: 0, tac: 0, tdc: 0, electrodes: [0]}]}'
 ```
**4. Start the component:**

```
rostopic pub /tactile_demux/event_in std_msgs/String "e_start"
```
**5. To stop the component:**

```
rostopic pub /tactile_demux/event_in std_msgs/String "e_stop"
```