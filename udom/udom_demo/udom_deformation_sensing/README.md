# UDOM Deformation sensing
This package contains nodes to demonstrate the sensing of deformations (e.g. how a
deformable object's shape is changed due to its manipulation). 

## The *deformation_sensing_interactive_demo* node
This node uses several components and a GUI to interactively demonstrate deformation
sensing. The component serves as a configurator/coordinator, i.e. it sets
the required parameters for all the components and starts/stops them accordingly.

It uses the following nodes:

  * `udom_visualization/wrench_mock_up_gui`

  * `udom_topic_tools/force_merger`

  * `udom_geometric_transformation/force_transformer`

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

**2. Alternatively, you can subscribe to the mesh surface of the component for
 visualization (e.g. using RViz):**

```
rostopic echo /tet_mesh_to_mesh_node/mesh_visualization
```

**3. Use the GUI to control the force exerted to the object as well as to
 start/stop/reset the component.**
