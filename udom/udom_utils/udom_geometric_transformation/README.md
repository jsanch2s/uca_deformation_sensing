# UDOM Geometric transformations
This package contains nodes to perform geometric transformations such as
transforming a wrench from one frame of reference to another.
___
___
## The *force_transformer* node
This node transforms every force in a udom_common_msgs.msg.ForceMultiArray into a single
message of type udom_common_msgs.msg.ForceArray with a single specified reference frame.

<sub>[**Note**: This node requires that a reference frame is specified and that it exists.]</sub>

**Input(s):**
  * `force_in`: The forces to be transformed.
    - *type:* `udom_common_msgs/ForceMultiArray`
  * `event_in`: The desired event for the node:
      `e_start`: starts the component.
      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `force_out`: An array of forces on which all elements are specified with respect to a
        single reference frame.
    - *type:* `udom_common_msgs/ForceArray`
  * `event_out`: The current event of the node.
      `e_running`: when the component is running.
      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `wait_for_transform`: Maximum duration to wait for a transform (in seconds).
  * `reference_frame`: The forces will be described with respect to this frame.
---
### Usage
**1. Launch the component (example):**

```
roslaunch udom_geometric_transformation force_transformer_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /force_transformer/force_out
```
**3. Publish to the desired topic (example):**

 ```
 rostopic pub /force_transformer/force_in udom_common_msgs/ForceMultiArray "{forces: [{header: {frame_id: 'f1'}, positions: [{x: 1.0, y: 1.0, z: 1.0}], wrenches: [{force: {x: 1.0, y: 1.0, z: 1.0}}]}, {header: {frame_id: 'f2'}, positions: [{x: 2.0, y: 2.0, z: 2.0}], wrenches: [{force: {x: 2.0, y: 2.0, z: 2.0}}]}]}"
 ```
**3.1. In case there is no transform, you can publish them manually (example):**

```
rosrun tf static_transform_publisher 1 0 0 0 0 0 object_frame f1 1000
rosrun tf static_transform_publisher 2 0 0 0 0 0 object_frame f2 1000
```
**4. Start the component:**

```
rostopic pub /force_transformer/event_in std_msgs/String "e_start"
```
**5. To stop the component:**

```
rostopic pub /force_transformer/event_in std_msgs/String "e_stop"
```
___
___
## The *nodal_force_calculator* node
This node transforms the forces in a udom_common_msgs.msg.ForceArray message
to loads that are applied on each node of the specified volumetric mesh. The output
is an array of a three-element force vector for each node. By setting `add_gravity`
to True, it computes the gravity vector with respect to the `reference_frame`, it
requires that the user sets the gravity vector, e.g. (0, 0, -9.81) would mean the
gravity acts downwards if the Z axis of the reference frame is pointing upwards.

<sub>[**Note:** The mesh nodes are assumed to be on the same frame reference as the forces.]</sub>

**Input(s):**
  * `force_info`: The forces applied on a mesh.
    - *type:* `udom_common_msgs/ForceArray`
  * `mesh`: Volumetric mesh on which the forces are applied.
    - *type:* `udom_modeling_msgs/Mesh`
  * `event_in`: The desired event for the node:
      `e_start`: starts the component.
      `e_stop`: stops the component.
    - *type:* `std_msgs/String`

**Output(s):**
  * `force_out`: Nodal forces. The linear forces (in X, Y and Z) applied on each node
        of the mesh.
    - *type:* `std_msgs/Float32MultiArray`
  * `event_out`: The current event of the node.
      `e_running`: when the component is running.
      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `collision_distance`: Distance to determine if a collision is occurring between the mesh and the finger.
  * `add_gravity`: If true, it adds the gravity force with respect to the reference frame.
  * `gravity_vector`: The gravity vector, defaults to [0, 0, -9.805665] (in m/s^2).
  * `reference_frame`: Reference frame to compute the gravity vector.
  * `object_frame`: Object frame to compute the gravity vector.
  * `wait_for_transform`: Maximum duration to wait for a transform (in seconds).
  * `mass`: Object's mass (in Kg).
---
### Usage
**1. Launch the component (example):**

```
roslaunch udom_geometric_transformation nodal_force_calculator_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /nodal_force_calculator/force_out
```
**3. Publish to the desired topics (example):**

 ```
 rostopic pub /nodal_force_calculator/force_in udom_common_msgs/ForceArray "{header: {frame_id: 'object_frame'}, positions: [{x: 1.9, y: 0.4, z: 0.0}], wrenches: [{force: {x: 1.0, y: 1.0, z: 1.0}}]}"
 ```
 ```
 rostopic pub /nodal_force_calculator/mesh udom_modeling_msgs/Mesh "{tetrahedra: [vertex_indices: [3, 2, 4, 0], vertex_indices: [3, 1, 4, 0], vertex_indices: [3, 6, 2, 4], vertex_indices: [3, 6, 7, 4], vertex_indices: [3, 5, 1, 4], vertex_indices: [3, 5, 7, 4]], vertices: [{x: 0.0, y: 0.0, z: 0.0}, {x: 0.0, y: 0.0, z: 1.0}, {x: 0.0, y: 1.0, z: 0.0}, {x: 0.0, y: 1.0, z: 1.0}, {x: 1.0, y: 0.0, z: 0.0}, {x: 1.0, y: 0.0, z: 1.0}, {x: 1.0, y: 1.0, z: 0.0}, {x: 1.0, y: 1.0, z: 1.0}]}"
 ```

**4. Start the component:**

```
rostopic pub /nodal_force_calculator/event_in std_msgs/String "e_start"
```
**5. To stop the component:**

```
rostopic pub /nodal_force_calculator/event_in std_msgs/String "e_stop"
```
---
