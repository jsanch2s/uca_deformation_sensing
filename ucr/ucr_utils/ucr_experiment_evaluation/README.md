# UCR Data collection
This package contains nodes that are used for conducting experimental evaluation.
___
___
## The *grasp_deformable_object* node
This node controls the Shadow hand to open/close the desired fingers.

**Input(s):**
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `joint_positions`: Commands a joint joint_positions to the specified finger(s).
    - *type:* `std_msgs/Float64`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).

---
### Usage
**1. Launch the component (example showing the optional 'finger' parameter):**

```
roslaunch ucr_experiment_evaluation grasp_deformable_object.launch 
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Use the GUI to open or close the hand.**
