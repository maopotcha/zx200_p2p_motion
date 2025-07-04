cmake_minimum_required(VERSION 3.10)
project(zx200_p2p_motion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(eigen_stl_containers REQUIRED)

add_executable(moveit_p2p_node src/moveit_p2p_node.cpp)

ament_target_dependencies(moveit_p2p_node
  rclcpp
  moveit_ros_planning_interface
  yaml-cpp
)

target_link_libraries(moveit_p2p_node yaml-cpp)

target_link_options(moveit_p2p_node BEFORE PRIVATE "-Wl,--exclude-libs,eigen_stl_containers")

install(TARGETS moveit_p2p_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
