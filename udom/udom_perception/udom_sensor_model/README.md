# UDOM Sensor models
This package contains sensor models for different types of sensors such as
tactile sensors, force sensors and cameras.

___

## The *tactile_sensor_model* node
This node instantiates a tactile sensor model in order to map the readings of the
sensor to contact information.

**Input(s):**

  * `tactile_data`: The tactile sensor's output.
    - *type:* `udom_perception_msgs/BiotacStamped`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

    - *type:* `std_msgs/String`

**Output(s):**

  * `contact_info`: A representation of the tactile data that specifies the contact
      locations and magnitudes.
    - *type:* `udom_perception_msgs/ContactInfo`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `max_intensity_value`: Maximum value that an electrode might reach.

  * `intensity_threshold`: Threshold used to decide whether or not an electrode is active.
      Since the electrode value decreases the more its surface is pressed, a negative value
      thus, means higher intensity. The values above this threshold are then removed.

  * `gaussian`: If True, it uses a Gaussian distribution to find the electrodes' centers.
      Otherwise, it assumes the center as specified in the electrodes config file.

  * `inverted`: If False, it assumes that the intensity value of each electrode is
      proportional to the Gaussian distribution's width. Otherwise, the width is inversely
      proportional to the intensity.

  * `calibration_samples`: Number of samples to calculate the resting impedance values,
      these values are then used to subtract them from the current impedance values
      to obtain the impedance changes.

  * `sequence_length`: Number of samples representing the sequence length that was
      used to learn the model.

---

### Usage
**1. Launch the component (example):**

```
roslaunch udom_sensor_model tactile_sensor_model_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /tactile_sensor_model/contact_info
```

**3. Publish to the desired topic (example):**

 ```
 rostopic pub /tactile_sensor_model/tactile_data udom_perception_msgs/BiotacStamped "{header: {frame_id: 'f1'}, electrodes: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
 ```

**4. Start the component:**

```
rostopic pub /tactile_sensor_model/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /tactile_sensor_model/event_in std_msgs/String "e_stop"
```

___

## The *force_sensor_model* node
This node instantiates a force sensor model that predicts the contact wrench at the
sensor's frame by first estimating its non-contact forces using data from joint encoders
and inertial measurement units (IMUs), e.g. pose, twist and accelerations of the robot;
and then subtracting those non-contact forces from the sensor's output wrench.

**Input(s):**

  * `robot_data`: The data from encoders and IMUs used to predict a wrench at the robot's end effector.
    - *type:* `std_msgs/Float64MultiArray`

  * `wrench_in`: The sensor's output wrench.
    - *type:* `geometry_msgs/WrenchStamped`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

    - *type:* `std_msgs/String`

**Output(s):**

  * `force_array`: A representation of the wrench data that specifies the contact locations
    and magnitudes for (possibly) multiple forces.
    - *type:* `udom_common_msgs/ForceMultiArray`

  * `wrench_out`: The estimated wrench at the sensor's frame.
    - *type:* `geometry_msgs/WrenchStamped`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `sequence_length`: Number of samples representing the sequence length that was used to learn the model.

  * `model_name`: Name of the model for the trained RNN.

---

### Usage
**1. Launch the component (example):**

```
roslaunch udom_sensor_model force_sensor_model_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result(s) of the component:**

```
rostopic echo /force_sensor_model/contact_info
```

```
rostopic echo /force_sensor_model/wrench_out
```

**3. Publish to the desired topic (example):**

```
roscd udom_sensor_model/data
```

```
rosbag play force_sensor_example_non_contact.bag robot_data:=/force_sensor_model/robot_data
```

on another terminal

```
rosrun topic_tools transform /force_sensor_model/robot_data /force_sensor_model/wrench_in geometry_msgs/WrenchStamped 'geometry_msgs.msg.WrenchStamped(header=std_msgs.msg.Header(stamp=rospy.Time.now()), wrench=geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(m.data[20], m.data[21], m.data[22]), torque=geometry_msgs.msg.Vector3(m.data[23], m.data[24], m.data[25])))' --import geometry_msgs std_msgs rospy
```

**4. Start the component:**

```
rostopic pub /force_sensor_model/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /force_sensor_model/event_in std_msgs/String "e_stop"
```

___

## The *force_sensor_analytical_model* node

This node instantiates a force sensor model, based on identification of
inertial parameters[1], that predicts the contact wrench at the sensor
frame by first estimating its non-contact forces using data from joint encoders
and then subtracting those non-contact forces from the sensor's output wrench.

[1] Kubus, Daniel, Torsten Kroger, and Friedrich M. Wahl. "On-line rigid object
recognition and pose estimation based on inertial parameters." Intelligent Robots
and Systems, 2007. IROS 2007.

**Input(s):**

  * `robot_data`: The data from encoders and IMUs used to predict a wrench at the robot's end effector.
    - *type:* `std_msgs/Float64MultiArray`

  * `wrench_in`: The sensor's output wrench.
    - *type:* `geometry_msgs/WrenchStamped`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

    - *type:* `std_msgs/String`

**Output(s):**

  * `force_array`: A representation of the wrench data that specifies the contact locations
    and magnitudes for (possibly) multiple forces.
    - *type:* `udom_common_msgs/ForceMultiArray`

  * `wrench_out`: The estimated wrench at the sensor's frame.
    - *type:* `geometry_msgs/WrenchStamped`

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
roslaunch udom_sensor_model force_sensor_analytical_model_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result(s) of the component:**

```
rostopic echo /force_sensor_model/contact_info
```

```
rostopic echo /force_sensor_model/wrench_out
```

**3. Publish to the desired topic (example):**

```
roscd udom_sensor_model/data
```

```
rosbag play force_sensor_example_non_contact.bag robot_data:=/force_sensor_model/robot_data
```

on another terminal

```
rosrun topic_tools transform /force_sensor_model/robot_data /force_sensor_model/wrench_in geometry_msgs/WrenchStamped 'geometry_msgs.msg.WrenchStamped(header=std_msgs.msg.Header(stamp=rospy.Time.now()), wrench=geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(m.data[20], m.data[21], m.data[22]), torque=geometry_msgs.msg.Vector3(m.data[23], m.data[24], m.data[25])))' --import geometry_msgs std_msgs rospy
```

**4. Start the component:**

```
rostopic pub /force_sensor_model/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /force_sensor_model/event_in std_msgs/String "e_stop"
```

___

## The *wrench_filter* node
This node filters a wrench using different types of filters (e.g. moving average (MA),
exponential moving average (EMA), etc.).

**Input(s):**

  * `wrench_in`: The sensor's output wrench.
    - *type:* `geometry_msgs/WrenchStamped`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

    - *type:* `std_msgs/String`

**Output(s):**

  * `wrench_out`: The filtered wrench.
    - *type:* `geometry_msgs/WrenchStamped`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `filter_type`: Name of the filter to apply.

  * `samples`: Number of samples to use by finite impulse response filters (e.g. MA).

  * `alpha`: Filter coefficient used by infinite impulse response filters (e.g. EMA).

---

### Usage
**1. Launch the component (example):**

```
roslaunch udom_sensor_model wrench_filter_example.launch filter_type:=ma
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result(s) of the component:**

```
rostopic echo /wrench_filter/wrench_out
```

**3. Publish to the desired topic (example):**

```
roscd udom_sensor_model/data
```

```
rosbag play force_sensor_example_non_contact.bag robot_data:=/wrench_filter/robot_data
```

on another terminal

```
rosrun topic_tools transform /wrench_filter/robot_data /wrench_filter/wrench_in geometry_msgs/WrenchStamped 'geometry_msgs.msg.WrenchStamped(header=std_msgs.msg.Header(stamp=rospy.Time.now()), wrench=geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(m.data[20], m.data[21], m.data[22]), torque=geometry_msgs.msg.Vector3(m.data[23], m.data[24], m.data[25])))' --import geometry_msgs std_msgs rospy
```

**4. Start the component:**

```
rostopic pub /wrench_filter/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /wrench_filter/event_in std_msgs/String "e_stop"
```

___