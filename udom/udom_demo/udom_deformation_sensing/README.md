# UDOM Deformation sensing
This package contains nodes to demonstrate the sensing of deformations (e.g. how a
deformable object's shape is changed due to its manipulation). 

## The *deformation_sensing_demo* node
This node uses a pipeline of components to perform deformation sensing using a
a Shadow hand with five BioTac sensors.
The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

It uses the following nodes:


  * `udom_topic_tools/tactile_demux`

  * `udom_sensor_model/tactile_sensor_model`

  * `udom_contact_model/contact_model`

  * `udom_topic_tools/force_merger`

  * `udom_geometric_transformation/force_transformer`

  * `udom_geometric_transformation/nodal_force_calculator`

  * `udom_deformation_modeling/deformation_model`

**Assumptions:**
  * It assumes five BioTac sensors as the input sensors.

**Input(s):**

  * `tactile_info`: The output data of five BioTac sensors.
    - *type:* `sr_robot_msgs/BiotacAll`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_reset`: resets the mesh to its original undeformed state.

    - *type:* `std_msgs/String`

**Output(s):**
  
  * `mesh`: A mesh representation of the object with the updated nodes' position based
        on  the deformation.
    - *type:* `udom_modeling_msgs/Mesh`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `mesh_filename`: Filename of the volumetric mesh in a .veg format. **Note:** This file
        should be located in the config directory of the `deformation_sensing` package.

  * `reference_frame`: Reference frame of the object.

  * `first_finger_frame`: Reference frame of the first finger.

  * `middle_finger_frame`: Reference frame of the middle finger.

  * `ring_finger_frame`: Reference frame of the ring finger.

  * `little_finger_frame`: Reference frame of the little finger.

  * `thumb_frame`: Reference frame of the thumb.

  * `constrained_nodes`: Constrained vertices of the mesh. Each constrained node must
        specify its three degrees of freedom. E.g., to constrain vertices 4, 10 and 14 the
        constrained_nodes should be [12, 13, 14, 30, 31, 32, 42, 43, 44].

### Usage
**1. Launch the component (example):**

```
roslaunch udom_deformation_sensing deformation_sensing_demo.launch
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /udom_deformation_sensing_full/mesh
```

**2. Alternatively, you can subscribe to the result of the component for
 visualization (e.g. using RViz):**

```
rostopic echo /udom_deformation_sensing_full/points
```

**3. Publish to the desired topic (example):**

```
roscd udom_deformation_sensing/data/
```

```
rosbag play -l example.bag
```

**4. Start the component:**

```
rostopic pub /udom_deformation_sensing_full/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /udom_deformation_sensing_full/event_in std_msgs/String "e_stop"
```

**6. To reset the object to its original undeformed shape:**

```
rostopic pub /udom_deformation_sensing_full/event_in std_msgs/String "e_reset"
```

---

## The *deformation_sensing_interactive_demo* node
This node uses several components and a GUI to interactively demonstrate deformation
sensing. The component serves as a configurator/coordinator, i.e. it sets
the required parameters for all the components and starts/stops them accordingly.

It uses the following nodes:

  * `udom_visualization/wrench_mock_up_gui`

  * `udom_geometric_transformation/nodal_force_calculator`

  * `udom_deformation_modeling/deformation_model`

**Input(s):**

  * `force_info`: The external force to apply to the object (can be set through the GUI).
    - *type:* `udom_common_msgs/ForceArray`

  * `event_in`: The desired event for the node (can be set through the GUI):

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_resets`: resets the object's mesh to its original undeformed shape.

    - *type:* `std_msgs/String`

**Output(s):**

  * `mesh`: A mesh representation of the object with the updated nodes' position based
        on  the deformation.
    - *type:* `udom_modeling_msgs/Mesh`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `mesh_filename`: Filename of the volumetric mesh in a .veg format. **Note:** This file
        should be located in the config directory of the `deformation_sensing` package.

  * `reference_frame`: Reference frame of the object.

  * `constrained_nodes`: Constrained vertices of the mesh. Each constrained node must
        specify its three degrees of freedom. E.g., to constrain vertices 4, 10 and 14 the
        constrained_nodes should be [12, 13, 14, 30, 31, 32, 42, 43, 44].

### Usage
**1. Launch the component (example):**

```
roslaunch udom_deformation_sensing deformation_sensing_interactive_demo.launch mesh_filename:=bar_hard
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /udom_deformation_sensing_full/mesh
```

**2. Alternatively, you can subscribe to the result of the component for
 visualization (e.g. using RViz):**

```
rostopic echo /udom_deformation_sensing_full/points
```

**3. Use the GUI to control the force exerted to the object as well as to
 start/stop/reset the component.**

---

## The *deformation_sensing_experiment* node
These nodes use a pipeline of components to perform deformation sensing using a
a Shadow hand with five BioTac sensors.
The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.
The possible meshes are:

  * `bar_hard.veg`

  * `bar_mid.veg`

  * `bar_soft.veg`

  * `cube_hard.veg`

  * `cube_mid.veg`

  * `cube_soft.veg`

  * `sponge_hard.veg`

  * `sponge_mid.veg`

  * `sponge_soft.veg`

It uses the following nodes:

  * `udom_topic_tools/topic_throttle`

  * `udom_topic_tools/tactile_demux`

  * `udom_sensor_model/tactile_sensor_model`

  * `udom_contact_model/contact_model`

  * `udom_topic_tools/force_merger`

  * `udom_geometric_transformation/force_transformer`

  * `udom_geometric_transformation/nodal_force_calculator`

  * `udom_deformation_modeling/deformation_model`

**Assumptions:**

  * It assumes five BioTac sensors as the input sensors.
  
  * It also assumes a TF tree of the Shadow is available.

**Input(s):**

  * `tactile_info`: The output data of five BioTac sensors.
    - *type:* `sr_robot_msgs/BiotacAll`

  * `event_in`: The desired event for the node:

      `e_start`: starts the component.

      `e_stop`: stops the component.

      `e_reset`: resets the mesh to its original undeformed state.

    - *type:* `std_msgs/String`

**Output(s):**
  
  * `mesh`: A mesh representation of the object with the updated nodes' position based
        on  the deformation.
    - *type:* `udom_modeling_msgs/Mesh`

  * `event_out`: The current event of the node.

      `e_running`: when the component is running.

      `e_stopped`: when the component is stopped.

    - *type:* `std_msgs/String`

**Parameter(s):**

  * `loop_rate`: Node cycle rate (in Hz).

  * `mesh_filename`: Filename of the volumetric mesh in a .veg format. **Note:** This file
        should be located in the config directory of the `deformation_sensing` package.

  * `reference_frame`: Reference frame of the object.

  * `first_finger_frame`: Reference frame of the first finger.

  * `middle_finger_frame`: Reference frame of the middle finger.

  * `ring_finger_frame`: Reference frame of the ring finger.

  * `little_finger_frame`: Reference frame of the little finger.

  * `thumb_frame`: Reference frame of the thumb.

  * `constrained_nodes`: Constrained vertices of the mesh. Each constrained node must
        specify its three degrees of freedom. E.g., to constrain vertices 4, 10 and 14 the
        constrained_nodes should be [12, 13, 14, 30, 31, 32, 42, 43, 44].

### Usage
**1. Launch the component (example in case the data has been previously recorded):**

```
roslaunch udom_deformation_sensing deformation_sensing_experiment_cube.launch data_recorded:=true
```

<sub>[**Note**: You will probably need to create your own launch file and configure it according to your needs.]</sub>

**2. Subscribe to the result of the component:**

```
rostopic echo /udom_deformation_sensing_full/mesh
```

**2. Alternatively, you can subscribe to the result of the component for
 visualization (e.g. using RViz):**

```
rostopic echo /udom_deformation_sensing_full/points
```

**3. Publish to the desired topic (example):**

```
roscd udom_deformation_sensing/data/
```

```
rosbag play -l example_with_tf.bag tf:=/tf_old
```

**4. Start the component:**

```
rostopic pub /udom_deformation_sensing_full/event_in std_msgs/String "e_start"
```

**5. To stop the component:**

```
rostopic pub /udom_deformation_sensing_full/event_in std_msgs/String "e_stop"
```

**6. To reset the object to its original undeformed shape:**

```
rostopic pub /udom_deformation_sensing_full/event_in std_msgs/String "e_reset"
```
