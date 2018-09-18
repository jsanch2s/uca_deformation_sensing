cmake_minimum_required(VERSION 2.8.3)
project(udom_force_estimation)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sr_robot_msgs
    udom_perception_msgs
)

install(PROGRAMS
  scripts/coordinator_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
  add_rostest(launch/force_estimation_demo_sim.test)
endif()
