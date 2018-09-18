# UDOM Deformation control
This package contains nodes to demonstrate deformation control. 

## The *deformation_control_demo* node

This node uses a pipeline of components to perform deformation control using a
a KUKA arm and a force/torque sensor. The component serves as a configurator/coordinator,
i.e. it sets the required parameters for all the components and starts/stops them accordingly.

It uses the following nodes:

  * `udom_sensor_model/force_sensor_model`
  * `udom_geometric_transformation/force_transformer`
  * `udom_geometric_transformation/nodal_force_calculator`
  * `udom_deformation_modeling/deformation_model`
  * `udom_sensor_model/wrench_filter`
  * `udom_geometric_transformation/pose_extractor`
  * `udom_pose_control/pose_controller`

**Assumptions:**

  * It assumes a force/torque sensor as input sensor.

**Input(s):**

  * `wrench_in`: The output data of the force sensor.
    - *type:* `geometry_msgs/WrenchStamped`
  * `robot_data`: The data from encoders and IMUs used to predict a wrench at the robot's end effector.
    - *type:* `std_msgs/Float64MultiArray`
  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_reset`: resets the mesh to its original undeformed state.

    - *type:* `std_msgs/String`

**Output(s):**

  * `mesh`: A mesh representation of the object with the updated nodes' position based
        on  the deformation.
    - *type:* `udom_modeling_msgs/Mesh`
  * `twist`: Twist command to the robot controller.
    - *type:* `geometry_msgs/TwistStamped`
  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).
  * `mesh_filename`: Filename of the volumetric mesh in a .veg format. **Note:** This file
        should be located in the config directory of the `deformation_sensing` package.
  * `object_frame`: Reference frame of the object.
  * `twist_frame`: The twist will be described with respect to this frame (e.g. end-effector frame).
  * `pose_nodes`: Nodes used to extract a pose from the mesh.
  * `constrained_nodes`: Constrained vertices of the mesh. Each constrained node must
        specify its three degrees of freedom. E.g., to constrain vertices 4, 10 and 14 the
        constrained_nodes should be [12, 13, 14, 30, 31, 32, 42, 43, 44].

___

### Usage
**1. Launch the component (example):**

```
roslaunch udom_deformation_control control_and_sensing_bar.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /udom_deformation_control/mesh
```

**2. Alternatively, you can subscribe to the result of the component for
 visualization (e.g. using RViz):**

```
rostopic echo /udom_deformation_control/points
```

**3. Publish to the desired topic (example):**

```
roscd udom_deformation_control/data/
```

```
rosbag play -l example.bag
```

**4. Start the component:**

```
rostopic pub /udom_deformation_control/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /udom_deformation_control/event_in std_msgs/String "e_stop"
```

**6. To reset the object to its original undeformed shape:**

```
rostopic pub /udom_deformation_control/event_in std_msgs/String "e_reset"
```
