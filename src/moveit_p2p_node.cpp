#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <string>
#include <chrono>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // /zx200 namespace に属するノードを作成
  auto node = rclcpp::Node::make_shared("p2p_motion", "/zx200");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  moveit::planning_interface::MoveGroupInterface::Options options("manipulator");
  options.move_group_namespace_ = "/zx200";

  // tf_buffer と timeout を定義
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  rclcpp::Duration timeout(10, 0);

  // MoveGroupInterface を作成（ROS2 Humble 用の4引数形式）
  moveit::planning_interface::MoveGroupInterface move_group(node, options, tf_buffer, timeout);

  // YAMLから目標関節位置を読み込む
  std::string yaml_path = ament_index_cpp::get_package_share_directory("zx200_p2p_motion") + "/config/p2p_targets_1-5.yaml";

  // 動作速度・加速度を最大に
  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  move_group.setPlanningTime(5.0);  // 必要に応じて調整
  move_group.setNumPlanningAttempts(10);

  YAML::Node config = YAML::LoadFile(yaml_path);
  auto joint_targets = config["p2p_targets"].as<std::vector<std::vector<double>>>();

  for (size_t i = 0; i < joint_targets.size(); ++i)
  {

    RCLCPP_INFO(node->get_logger(), "Planning to joint target %zu...", i + 1);

    move_group.setJointValueTarget(joint_targets[i]);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
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
