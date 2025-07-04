# File: CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(zx200_p2p_motion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(moveit_p2p_node src/moveit_p2p_node.cpp)
target_include_directories(moveit_p2p_node PRIVATE
  ${ament_index_cpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)
ament_target_dependencies(moveit_p2p_node
  rclcpp
  moveit_ros_planning_interface
  tf2_ros
  geometry_msgs
  moveit_msgs
  ament_index_cpp
)
target_link_libraries(moveit_p2p_node yaml-cpp)

install(TARGETS moveit_p2p_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()