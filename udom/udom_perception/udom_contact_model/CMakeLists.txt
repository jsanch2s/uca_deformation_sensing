cmake_minimum_required(VERSION 2.8.3)
project(udom_contact_model)

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
    udom_common_msgs
    udom_perception_msgs
)

install(PROGRAMS
  scripts/contact_model
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### TESTS
roslint_python()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
  add_rostest(launch/contact_model.test)
  catkin_add_nosetests(test/contact_model_unittest.py)
endif()
