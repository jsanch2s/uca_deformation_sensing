cmake_minimum_required(VERSION 2.8.3)
project(udom_visualization)

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
    visualization_msgs
    udom_common_msgs
    udom_modeling_msgs
    udom_perception_msgs
)

install(PROGRAMS
  scripts/mesh_visualizer
  scripts/points_visualizer
  scripts/wrench_mock_up_gui
  scripts/wrench_visualizer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
  add_rostest(launch/mesh_visualizer.test)
  add_rostest(launch/points_visualizer.test)
  add_rostest(launch/wrench_visualizer.test)
endif()
