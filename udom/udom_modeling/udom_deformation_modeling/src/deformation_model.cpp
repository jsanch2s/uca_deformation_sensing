/*
 * Copyright 2016 Université Clermont Auvergne (UCA)
 *
 * Author: Jose Sanchez
 *
 */

#include <string>
#include <vector>
#include <udom_deformation_modeling/deformation_model.h>
#include <udom_deformation_modeling/vega_wrapper.h>


DeformationModel::DeformationModel(const ros::NodeHandle &nh) :
        nh_(nh), force_info_received_(false), state_(INIT)
{
    std::string file_path = ros::package::getPath("udom_deformation_modeling");
    ros::param::param<std::string>("~mesh_filename", mesh_filename_, "example_mesh.veg");
    mesh_filename_ = file_path + "/config/" + mesh_filename_;

    ros::param::param<std::vector<int>>(
        "~constrained_nodes", constrained_nodes_, std::vector<int>());
    ros::param::param<double>("~timestep", timestep_, 0.02);
    ros::param::param<double>("~damping_mass", damping_mass_, 1.0);
    ros::param::param<double>("~damping_stiffness", damping_stiffness_, 0.01);
    ros::param::param<int>("~index_start", index_start_, 0);

    vega_interface_.reset(new VegaWrapper(
        mesh_filename_, constrained_nodes_, timestep_, damping_mass_, damping_stiffness_));

    vega_interface_->init_mesh(mesh);
    mesh_dofs = vega_interface_->mesh_dofs();
    vega_interface_->extract_points(constrained_nodes_, constrained_points, index_start_);

    pub_mesh_ = nh_.advertise<udom_modeling_msgs::Mesh>("mesh", 1, true);
    pub_constrained_nodes_ = nh_.advertise<visualization_msgs::Marker>(
        "constrained_nodes_visualization", 10);
    pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 10);

    sub_force_info_ = nh_.subscribe(
        "force_info", 1, &DeformationModel::forceInfoCallback, this,
        ros::TransportHints().tcpNoDelay());
    sub_event_in_ = nh_.subscribe("event_in", 1, &DeformationModel::eventInCallback, this);

    ROS_DEBUG("State: INIT");
}

DeformationModel::~DeformationModel()
{
    pub_mesh_.shutdown();
    pub_constrained_nodes_.shutdown();
    pub_event_out_.shutdown();
    sub_force_info_.shutdown();
    sub_event_in_.shutdown();
}

void DeformationModel::eventInCallback(const std_msgs::String::ConstPtr &msg)
{
    event_ = msg->data;
}

void DeformationModel::forceInfoCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    force_info_ = *msg;
    force_info_received_ = true;
}

void DeformationModel::start()
{
    switch (state_)
    {
        case INIT:
            ROS_DEBUG("State: INIT");
            initState();
            break;
        case IDLE:
            ROS_DEBUG("State: IDLE");
            idleState();
            break;
        case RUNNING:
            ROS_DEBUG("State: RUNNING");
            runningState();
            break;
        default:
            initState();
    }
}

void DeformationModel::initState()
{
    if (event_ == "e_start") {
        pub_mesh_.publish(mesh);
        pub_constrained_nodes_.publish(constrained_points);
        state_ = IDLE;
        event_ = "";
    }
    else if (event_ == "e_reset")
    {
        state_ = IDLE;
        event_ = "";
        vega_interface_->reset_mesh(mesh);
        state_ = INIT;
    }
    else
    {
        state_ = INIT;
    }
}

void DeformationModel::idleState()
{
    if (event_ == "e_stop")
    {
        std_msgs::String event_out_msg;
        event_out_msg.data = "e_stopped";
        pub_event_out_.publish(event_out_msg);
        state_ = INIT;
        event_ = "";
    }
    else if (event_ == "e_reset")
    {
        state_ = IDLE;
        event_ = "";
        force_info_received_ = false;
        vega_interface_->reset_mesh(mesh);
    }
    else if (force_info_received_)
    {
        state_ = RUNNING;
//        force_info_received_ = false;
    }
    else
    {
        state_ = IDLE;
    }
}

void DeformationModel::runningState()
{
    if (event_ == "e_stop")
    {
        std_msgs::String event_out_msg;
        event_out_msg.data = "e_stopped";
        pub_event_out_.publish(event_out_msg);
        state_ = INIT;
        event_ = "";
        force_info_received_ = false;
    }
    else if (event_ == "e_reset")
    {
        state_ = INIT;
        event_ = "";
        force_info_received_ = false;
        vega_interface_->reset_mesh(mesh);
    }
    else
    {
        computeDeformation(force_info_);
        state_ = IDLE;
//        event_ = "";
//        force_info_received_ = false;
    }
}

void DeformationModel::computeDeformation(const std_msgs::Float32MultiArray &force_in)
{
    std::vector<double> displacements(mesh_dofs, 0.0);
    std_msgs::String event_out_msg;
    std::vector<double> forces(std::begin(force_in.data), std::end(force_in.data));

    // Apply forces and update the mesh.
    vega_interface_->apply_forces(forces);
    vega_interface_->get_displacements(displacements);
    vega_interface_->update_mesh(mesh, displacements);

    event_out_msg.data = "e_running";
    pub_event_out_.publish(event_out_msg);
    pub_mesh_.publish(mesh);
    pub_constrained_nodes_.publish(constrained_points);
}
