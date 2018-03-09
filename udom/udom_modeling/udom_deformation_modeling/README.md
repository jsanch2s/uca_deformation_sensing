# UDOM Deformation modeling
This package contains nodes to simulate the deformation of an object using the
Vega FEM library[1]

[1] http://run.usc.edu/vega/.
___
___
## The *deformation_model* node
This node subscribes to an array of forces message (std_msgs::Float32MultiArray)
and computes the deformation of a mesh based on the applied forces. It outputs a
volumetric mesh message (udom_modeling_msgs::Mesh).

**Input(s):**
  * `force_out`: Nodal forces. The linear forces (in X, Y and Z) applied on each node
        of the mesh.
    - *type:* `std_msgs/Float32MultiArray`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_reset`: resets the mesh to its original undeformed state.
    - *type:* `std_msgs/String`

**Output(s):**
  * `mesh`: The mesh configuration after the deformation.
    - *type:* `udom_modeling_msgs/Mesh`
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `mesh_filename`: Filename of the volumetric mesh in a .veg format. This file should be located in the `config` directory.
---
### Usage
**1. Launch the component (example):**

```
roslaunch udom_deformation_modeling deformation_model_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /deformation_model/mesh
```

**3. Publish to the desired topic (example):**

 ```
 rostopic pub /deformation_model/force_info std_msgs/Float32MultiArray "{data: [1, 0, 0, 0, 0, 0]}"
 ```
 
**4. Start the component:**

```
rostopic pub /deformation_model/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /deformation_model/event_in std_msgs/String "e_stop"
```

**6. To reset the object to its original undeformed shape:**

```
rostopic pub /deformation_model/event_in std_msgs/String "e_reset"
```