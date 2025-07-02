cmake_minimum_required(VERSION 3.10)
project(zx200_p2p_motion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(moveit_p2p_node src/moveit_p2p_node.cpp)
ament_target_dependencies(moveit_p2p_node rclcpp moveit_ros_planning_interface)

install(TARGETS moveit_p2p_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
