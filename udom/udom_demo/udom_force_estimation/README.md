# UDOM Force estimation
This package contains nodes to demonstrate force estimation based on the output of
a BioTac sensor.
___
___
## The *force_estimation* node
This node uses a pipeline of components to demonstrate the force estimation.
The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

It uses the following nodes:
  * `udom_sensor_model/tactile_sensor_model`
  * `udom_topic_tools/tactile_demux`
  * `udom_topic_tools/topic_relay`

**Assumptions:**
  * It assumes a BioTac sensor as the input sensor.

**Input(s):**
  * `tactile_info`: The output data of a BioTac sensor.
    - *type:* `udom_perception_msgs/BiotacStamped`
  * `event_in`: The desired event for the node:
      `e_start`: starts the component.
      `e_stop`: stops the component.

**Output(s):**
  * `actual_wrench`: Actual wrench information.
    - *type:* `geometry_msgs/WrenchStamped`
  * `estimated_wrench`: Estimated wrench information for each of the five fingers.
    - *type:* `udom_perception_msgs/ContactInfo`
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
roslaunch udom_force_estimation force_estimation_demo_sim.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result(s) of the component:**

```
/udom_force_estimation/wrench_out
/udom_force_estimation/ff_tactile_sensor_model/contact_info
/udom_force_estimation/mf_tactile_sensor_model/contact_info
/udom_force_estimation/rf_tactile_sensor_model/contact_info
/udom_force_estimation/lf_tactile_sensor_model/contact_info
/udom_force_estimation/th_tactile_sensor_model/contact_info
```

**3. Publish to the desired topic (example):**

```
cd ~/projects/ros_ws/indigo_ws/src/uca_deformable_object_manipulation/udom_demo/udom_force_estimation/data
rosbag play ff_concave_location1.bag /rh/tactile:=/udom_force_estimation/tactile_data_in /sensor_readings:=/udom_force_estimation/wrench_in
```

**4. Start the component:**

```
rostopic pub /udom_force_estimation/event_in std_msgs/String "data: 'e_start'"
```

**5. To stop the component:**

```
rostopic pub /udom_force_estimation/event_in std_msgs/String "data: 'e_stop'"
```
