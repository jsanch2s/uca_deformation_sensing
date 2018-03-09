cmake_minimum_required(VERSION 2.8.3)
project(udom_modeling_msgs)


find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
)

add_message_files(
  FILES
    Mesh.msg
    MeshTetrahedron.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
)
