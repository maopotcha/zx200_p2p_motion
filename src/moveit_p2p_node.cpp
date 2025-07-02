#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>

std::vector<std::vector<double>> load_joint_targets(const std::string &filename) {
  std::vector<std::vector<double>> targets;
  YAML::Node config = YAML::LoadFile(filename);

  for (const auto& target : config["p2p_targets"]) {
    std::vector<double> joint_values;
    for (const auto& val : target) {
      joint_values.push_back(val.as<double>());
    }
    targets.push_back(joint_values);
  }

  return targets;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("zx200_p2p_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // MoveGroupインターフェースを作成（manipulatorグループ）
  moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");

  // 目標関節位置を複数定義
  /*
  std::vector<std::vector<double>> joint_targets = {
      {0.0, -0.5, 1.0, 0.0, 0.0},
      {0.5, -0.2, 0.5, 0.3, 0.0},
      {1.0, 0.0, -0.5, -0.3, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0}};
  */
  
  std::string yaml_file = "~/ros2_ws/src/zx200_p2p_config/config/p2p_targets.yaml";
  std::vector<std::vector<double>> joint_targets = load_joint_targets(yaml_file);

  
  for (size_t i = 0; i < joint_targets.size(); ++i)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to joint target %zu", i + 1);

    move_group.setJointValueTarget(joint_targets[i]);

    // プラン作成
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(node->get_logger(), "Executing...");
      move_group.execute(plan);
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Planning failed for step %zu", i + 1);
    }

    rclcpp::sleep_for(std::chrono::seconds(1)); // 少し待機
  }

  rclcpp::shutdown();
  return 0;
}
