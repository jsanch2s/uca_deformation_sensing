cmake_minimum_required(VERSION 2.8.3)
project(udom_deformation_modeling)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rosunit
    roslint
    roslib
    std_msgs
    visualization_msgs
    udom_modeling_msgs
    vega_fem
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosunit
    std_msgs
    visualization_msgs
    udom_modeling_msgs
    vega_fem
)

add_definitions(-std=c++11)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

### LIBRARY
add_library(deformation_model
  src/vega_wrapper.cpp
  src/deformation_model.cpp
)
target_link_libraries(deformation_model
  ${catkin_LIBRARIES}
)

### EXECUTABLES
add_executable(deformation_model_node
  src/deformation_model_node.cpp
)
add_dependencies(deformation_model_node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(deformation_model_node
  ${catkin_LIBRARIES}
  deformation_model
)

### TESTS
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  find_package(rostest REQUIRED)

  roslaunch_add_file_check(launch)
  add_rostest(launch/deformation_model.test)
  add_rostest(launch/deformation_model_1_indexed.test)

  catkin_add_gtest(vega_wrapper_test
    test/vega_wrapper_test.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/config/
  )
  target_link_libraries(vega_wrapper_test
    deformation_model
    ${catkin_LIBRARIES}
  )
    catkin_add_gtest(vega_wrapper_test_1_indexed
    test/vega_wrapper_test_1_indexed.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/config/
  )
  target_link_libraries(vega_wrapper_test_1_indexed
    deformation_model
    ${catkin_LIBRARIES}
  )
endif()

### INSTALLS
install(TARGETS deformation_model
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS deformation_model_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
