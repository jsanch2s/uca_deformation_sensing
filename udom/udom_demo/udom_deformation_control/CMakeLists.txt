cmake_minimum_required(VERSION 2.8.3)
project(udom_deformation_control)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    udom_sensor_model
    udom_geometric_transformation
    udom_deformation_modeling
    udom_visualization
    std_msgs
    geometry_msgs
    udom_perception_msgs
    udom_modeling_msgs
)

install(PROGRAMS
  scripts/coordinator_icra_node
  scripts/coordinator_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
  add_rostest(launch/deformation_control_demo.test)
endif()
