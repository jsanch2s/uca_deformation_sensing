# UDOM Sensor models
This package contains sensor models for different types of sensors such as
tactile sensors and cameras.
___
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
