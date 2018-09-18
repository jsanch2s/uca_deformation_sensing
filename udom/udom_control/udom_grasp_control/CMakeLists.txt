cmake_minimum_required(VERSION 2.8.3)
project(udom_grasp_control)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
    dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/ReactiveGraspParams.cfg
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    controller_manager_msgs
    dynamic_reconfigure
    udom_perception_msgs
)

install(PROGRAMS
  scripts/reactive_grasp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
endif()
