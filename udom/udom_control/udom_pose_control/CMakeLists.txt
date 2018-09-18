cmake_minimum_required(VERSION 2.8.3)
project(udom_pose_control)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
    tf
    tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    tf
    tf2_ros
    std_msgs
    geometry_msgs
    udom_common_msgs
    udom_perception_msgs
)

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(launch)
endif()
