cmake_minimum_required(VERSION 2.8.3)
project(ucr_experiment_evaluation)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    tf2_ros
    std_msgs
    geometry_msgs
    shape_msgs
    sensor_msgs
    visualization_msgs
    controller_manager_msgs
    udom_perception_msgs
    udom_modeling_msgs
)

install(PROGRAMS
  scripts/contact_localization_evaluation
  scripts/grasp_deformable_object
  scripts/mesh_to_point_cloud
  scripts/tet_mesh_to_mesh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
endif()
