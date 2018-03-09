cmake_minimum_required(VERSION 2.8.3)
project(ucr_data_collection)

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
    control_msgs
    sr_robot_msgs
)

install(PROGRAMS
  scripts/finger_data_experiment
  scripts/zhanat_experiment
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
endif()
