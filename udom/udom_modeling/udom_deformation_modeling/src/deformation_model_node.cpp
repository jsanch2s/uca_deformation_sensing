/*
 * Copyright 2016 Université Clermont Auvergne (UCA)
 *
 * Author: Jose Sanchez
 *
 */

#include <ros/ros.h>
#include <udom_deformation_modeling/deformation_model.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "deformation_model");
    ros::NodeHandle nh("~");
    double loop_rate;

    /// Node cycle rate (in Hz).
    ros::param::param<double>("~loop_rate", loop_rate, 10.0);
    ros::Rate my_rate(loop_rate);

    DeformationModel deformation_model_node(nh);
    ROS_INFO_STREAM("Ready to start...");

    while (nh.ok())
    {
        ros::spinOnce();
        deformation_model_node.start();
        my_rate.sleep();
    }

    return 0;
}
