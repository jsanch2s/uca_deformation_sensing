# UDOM Contact models
This package contains contact models for tactile sensors based on different assumptions.
___
___
## The *contact_model* node
This node instantiates a contact model for a tactile sensor in order to compute
a force array based on the sensor's contact information.

**Input(s):**
  * `contact_info`: The contact information (e.g. the output of the tactile sensor model).
    - *type:* `udom_perception_msgs/ContactInfo`
  * `event_in`: The desired event for the node:
      `e_start`: starts the component.
      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `force_array`: An array of forces representing the contact(s).
    - *type:* `udom_common_msgs/ForceArray`
  * `event_out`: The current event of the node.
      `e_running`: when the component is running.
      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `contact_model`:  Name of the contact model to be used. The model must be
      specified in the `udom_contact_model.contact_model` module.
  * `filter_force`: If set to true, it filters out residual forces that are below a specified
      threshold.
  * `threshold`: Threshold to remove residual forces of the sensor (in Newtons).
---
### Usage
**1. Launch the component (example):**

```
roslaunch udom_contact_model contact_model_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /contact_model/force_array
```
**3. Publish to the desired topic (example):**

 ```
 rostopic pub /contact_model/contact_info udom_perception_msgs/ContactInfo "{header: {frame_id: 'f1'}, contact_points: [{x: 1.0, y: 0.0, z: 0.0}], wrenches: [{force: {x: 1.0, y: 0.0, z: 0.0}}, {torque: {x: 0.0, y: 0.0, z: 0.0}}]}"
 ```
**4. Start the component:**

```
rostopic pub /contact_model/event_in std_msgs/String "e_start"
```
**5. To stop the component:**

```
rostopic pub /contact_model/event_in std_msgs/String "e_stop"
```
