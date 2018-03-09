cmake_minimum_required(VERSION 2.8.3)
project(udom_topic_tools)

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
    tf2_msgs
    sr_robot_msgs
    udom_common_msgs
    udom_modeling_msgs
    udom_perception_msgs
)

install(PROGRAMS
  scripts/force_merger
  scripts/tactile_demux
  scripts/topic_relay
  scripts/topic_resync
  scripts/topic_throttle
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
  add_rostest(launch/force_merger.test)
  add_rostest(launch/tactile_demux.test)
endif()
