#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("zx200_p2p_node");

  moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");

  // 動作速度・加速度を最大に
  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  move_group.setPlanningTime(5.0);  // 必要に応じて調整
  move_group.setNumPlanningAttempts(10);

  // YAMLから目標関節値の読み込み
  std::string package_path = ament_index_cpp::get_package_share_directory("zx200_p2p_motion");
  std::string yaml_path = package_path + "/config/p2p_targets.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);
  auto joint_targets = config["p2p_targets"].as<std::vector<std::vector<double>>>();

  for (size_t i = 0; i < joint_targets.size(); ++i)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to joint target %zu...", i + 1);
    move_group.setJointValueTarget(joint_targets[i]);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(node->get_logger(), "Executing...");
      move_group.execute(plan);
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "Planning failed at step %zu", i + 1);
    }

    // 次の計画のために少し待機（最小限）
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  rclcpp::shutdown();
  return 0;
}
