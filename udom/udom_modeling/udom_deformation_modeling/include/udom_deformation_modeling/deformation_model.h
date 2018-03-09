/*
 * Copyright 2016 Université Clermont Auvergne (UCA)
 *
 * Author: Jose Sanchez
 *
 */

#ifndef UDOM_DEFORMATION_MODELING_DEFORMATION_MODEL_H
#define UDOM_DEFORMATION_MODELING_DEFORMATION_MODEL_H

#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <udom_modeling_msgs/Mesh.h>
#include <udom_deformation_modeling/vega_wrapper.h>

/**
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
    - *type:* `std_msgs/String`

**Output(s):**
  * `mesh`: The mesh configuration after the deformation.
    - *type:* `udom_modeling_msgs/Mesh`
  * `constrained_nodes_visualization`: Constrained nodes on the object for visualization.
    - *type:* `visualization_msgs/Marker`
  * `event_out`: The current event of the node.
      `e_running`: when the component is running.
      `e_stopped`: when the component is stopped.
    - *type:* `std_msgs/String`

**Parameter(s):**
  * `loop_rate`: Node cycle rate (in Hz).
  * `mesh_filename`: Filename of the volumetric mesh in a .veg format. This file should
        be specified in the `config` directory.
 */
class DeformationModel
{
    public:
        /**
         * Constructor.
         *
         * @param nh An instance of the ROS node handle.
         */
        explicit DeformationModel(const ros::NodeHandle &nh);
        /// Destructor.
        virtual ~DeformationModel();
        /// Method to start the node's cycle.
        void start();
    private:
        /// Copy constructor.
        DeformationModel(const DeformationModel &other);
        /// Assignment operator
        DeformationModel &operator=(const DeformationModel &other);

        /** @name States
         *  States of the state machine used by the node.
         */
        ///@{
        /// Waits for the start event, once received it sets the node state to IDLE.
        void initState();
        /// Waits for the force information, once received it sets the node state to RUNNING.
        void idleState();
        /// Computes the deformation of the node.
        void runningState();
        ///@}

        /** @name Callbacks
         *  Callbacks of the node's subscribers.
         */
        ///@{
        /// Obtains the event for the node (e.g. start, stop).
        void eventInCallback(const std_msgs::String::ConstPtr &msg);
        /// Obtains the force information input.
        void forceInfoCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
        ///@}

        /**
         * Computes the deformation of an object based on the applied forces and publishes
         * the deformed surface mesh.
         */
        void computeDeformation(const std_msgs::Float32MultiArray &force_in);

    private:
        /// Node's states to represent each cycle.
        enum State
        {
            INIT,
            IDLE,
            RUNNING
        };

    private:
        /// Node handle to communicate with the ROS system.
        ros::NodeHandle nh_;

        /** Filename of the volumetric mesh in a .veg format.
         * Note: This file should be located in the config directory.
        */
        std::string mesh_filename_;

        /**
         * Constrained vertices of the mesh. Each constrained node must specify
         * its three degrees of freedom. E.g., to constrain vertices 4, 10 and 14:
         * constrained_nodes_ = {12, 13, 14, 30, 31, 32, 42, 43, 44};
         */
        std::vector<int> constrained_nodes_;

        /// Time step of the simulation (in seconds).
        double timestep_;

        /** @name Damping parameters.
         *  (tangential) Rayleigh damping.
         */
        ///@{
        /// Mass damping coefficient, "underwater"-like damping.
        double damping_mass_;
        /// Stiffness damping coefficient, (primarily) high-frequency damping.
        double damping_stiffness_;

        /// Where the indexes start (e.g. 0 for zero-based numbering).
        int index_start_;

        /// State of the node (e.g. init, idle or running).
        State state_;

        /// Event to control the state of the node (e.g. e_start, e_stop).
        std::string event_;

        /// The external force to be applied to the mesh's surface.
        std_msgs::Float32MultiArray force_info_;

        /// Flag to check if the input has been received.
        bool force_info_received_;

        /// Interface to the Vega FEM library.
        std::shared_ptr<VegaWrapper> vega_interface_;

        /// Object's mesh.
        udom_modeling_msgs::Mesh mesh;

        /// Constrained nodes on the object for visualization.
        visualization_msgs::Marker constrained_points;

        /// Degrees of freedom of the mesh.
        int mesh_dofs;

        /// Publishers.
        ros::Publisher pub_mesh_;
        ros::Publisher pub_constrained_nodes_;
        ros::Publisher pub_event_out_;

        /// Subscribers.
        ros::Subscriber sub_force_info_;
        ros::Subscriber sub_event_in_;
};

#endif  // UDOM_DEFORMATION_MODELING_DEFORMATION_MODEL_H
