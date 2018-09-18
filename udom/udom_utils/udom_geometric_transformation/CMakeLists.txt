cmake_minimum_required(VERSION 2.8.3)
project(udom_geometric_transformation)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roslint
    tf
    tf2_ros
    dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/CollisionDistance.cfg
  cfg/NodesToExtract.cfg
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    tf
    tf2_ros
    std_msgs
    geometry_msgs
    tf2_geometry_msgs
    dynamic_reconfigure
    udom_common_msgs
    udom_modeling_msgs
    udom_control_msgs
)

install(PROGRAMS
  scripts/force_transformer
  scripts/nodal_force_calculator
  scripts/pose_error_calculator
  scripts/pose_extractor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
  add_rostest(launch/force_transformer.test)
  add_rostest(launch/nodal_force_calculator.test)
  add_rostest(launch/pose_extractor.test)
  catkin_add_nosetests(test/force_transformation_unittest.py)
  catkin_add_nosetests(test/nodal_force_computation_unittest.py)
  catkin_add_nosetests(test/pose_extractor_unittest.py)
  catkin_add_nosetests(test/quaternion_algebra_unittest.py)
endif()
