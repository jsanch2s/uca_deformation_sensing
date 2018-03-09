# UDOM Visualization
This package contains nodes to visualize different objects (e.g. a mesh) used
in the UDOM packages.
___
___
## The *mesh_visualizer* node
This node converts a Mesh into a Marker message in order to be visualized (e.g. in RViz).

**Input(s):**
  * `mesh`: Volumetric mesh to be visualized
    - *type:* `udom_modeling_msgs/Mesh`

**Output(s):**
  * `points`: The vertices of the mesh represented as a Marker.
    - *type:* `visualization_msgs/Marker`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `mesh_frame`: Reference frame of the mesh to be converted.
  * `scale`: Marker's scale for the X, Y and Z axes.
  * `color`: Marker's color values for the red, green, blue and alpha.
---
### Usage
**1. Launch the component (example):**

```
roslaunch udom_visualization mesh_visualizer_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /mesh_visualizer/points
```
**3. Publish to the desired topic (example):**

 ```
 rostopic pub /mesh_visualizer/mesh udom_modeling_msgs/Mesh "{tetrahedra: [vertex_indices: [3, 2, 4, 0], vertex_indices: [3, 1, 4, 0], vertex_indices: [3, 6, 2, 4], vertex_indices: [3, 6, 7, 4], vertex_indices: [3, 5, 1, 4], vertex_indices: [3, 5, 7, 4]], vertices: [{x: 0.0, y: 0.0, z: 0.0}, {x: 0.0, y: 0.0, z: 1.0}, {x: 0.0, y: 1.0, z: 0.0}, {x: 0.0, y: 1.0, z: 1.0}, {x: 1.0, y: 0.0, z: 0.0}, {x: 1.0, y: 0.0, z: 1.0}, {x: 1.0, y: 1.0, z: 0.0}, {x: 1.0, y: 1.0, z: 1.0}]}"
 ```
___
___
## The *points_visualizer_node* node
This node fills a Marker message in order to be visualized (e.g. in RViz).

**Input(s):**
  * `points`: Points to be visualized.
    - *type:* `visualization_msgs/Marker`

**Output(s):**
  * `points`: Points to be visualized with their marker_out message correctly filled.
    - *type:* `visualization_msgs/Marker`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `points_frame`: Reference frame of the marker_out message to be filled.
  * `scale`: Marker's scale for the X, Y and Z axes.
  * `color`: Marker's color values for the red, green, blue and alpha.
---
### Usage
**1. Launch the component (example):**

```
roslaunch udom_visualization points_visualizer_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /points_visualizer/points_out
```
**3. Publish to the desired topic (example):**

 ```
 rostopic pub /points_visualizer/points_in visualization_msgs/Marker "{points: [{x: 0.0, y: 0.0, z: 0.1}, {x: 0.3, y: 0.2, z: 0.0}, {x: 0.0, y: 0.1, z: 0.2}]}"
 ```
___
___
## The *wrench_mock_up* node
This node publishes a wrench message based on the controls of a GUI.

**Output(s):**
  * `mock_up_wrench`: A wrench message.
    - *type:* `geometry_msgs/WrenchStamped`
  * `force_array`: The wrench message converted to a force array message.
    - *type:* `udom_common_msgs/ForceArray`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `reference_frame`: Reference frame of the wrench to be published.
---
### Usage
**1. Launch the component (example):**

```
roslaunch udom_visualization wrench_mock_up_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /wrench_mock_up_gui/mockup_wrench
```
___
___
## The *wrench_visualizer_node* node
This node converts a ContactInfo message into a PointStamped and a WrenchStamped messages,
it also adds their header information in order to be visualized (e.g. in RViz).

**Input(s):**
  * `contact_info`: Contact information to be visualized.
    - *type:* `udom_perception_msgs/ContactInfo`

**Output(s):**
  * `point`: Point to be visualized with their marker_out message correctly filled.
    - *type:* `geometry_msgs/PointStamped`
  * `wrench`: Wrench to be visualized with their marker_out message correctly filled.
    - *type:* `geometry_msgs/WrenchStamped`


**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
---
### Usage
**1. Launch the component (example):**

```
roslaunch udom_visualization wrench_visualizer_example.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result(s) of the component:**

```
rostopic echo /wrench_visualizer/point
rostopic echo /wrench_visualizer/wrench
```
**3. Publish to the desired topic (example):**

 ```
 rostopic pub /wrench_visualizer/contact_info udom_perception_msgs/ContactInfo "{header: {frame_id: 'frame'}, contact_points: [{x: 1.0, y: 0.0, z: 0.0}], wrenches: [{force: {x: 0.0, y: 0.0, z: 1.0}}, torque: {x: 0.0, y: 0.0, z: 0.0}]}"
 ```