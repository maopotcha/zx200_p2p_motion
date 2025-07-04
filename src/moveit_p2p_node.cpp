// File: src/moveit_p2p_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose.h>
#include <moveit_msgs/msg/robot_trajectory.h>
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

  // MoveGroupInterface の設定
  moveit::planning_interface::MoveGroupInterface::Options options("manipulator");
  options.move_group_namespace_ = "/zx200";

  // TF2 バッファとリスナー
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  rclcpp::Duration timeout(10, 0);
  moveit::planning_interface::MoveGroupInterface move_group(node, options, tf_buffer, timeout);

  // 刃先リンクをエンドエフェクタとして指定
  move_group.setEndEffectorLink("bucket_end_link");

  // 最大速度・加速度、計画時間、試行回数
  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setMaxAccelerationScalingFactor(1.0);
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(10);

  // YAML からカートesian 目標姿勢を読み込む
  std::string yaml_path = ament_index_cpp::get_package_share_directory("zx200_p2p_motion") + "/config/cartesian_targets.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);
  auto cart_targets = config["cartesian_targets"].as<std::vector<std::vector<double>>>();

  // Pose のリストを生成
  std::vector<geometry_msgs::msg::Pose> waypoints;
  for (auto &pt : cart_targets) {
    if (pt.size() != 7) {
      RCLCPP_WARN(node->get_logger(), "Invalid target size: expected 7 elements per waypoint");
      continue;
    }
    geometry_msgs::msg::Pose pose;
    pose.position.x = pt[0];
    pose.position.y = pt[1];
    pose.position.z = pt[2];
    pose.orientation.x = pt[3];
    pose.orientation.y = pt[4];
    pose.orientation.z = pt[5];
    pose.orientation.w = pt[6];
    waypoints.push_back(pose);
  }

  const double eef_step         = 0.005;   // 5mm 刻み
  const double jump_threshold   = 0.1;     // 閾値あり（rad）で多少飛びを許容
  const bool   avoid_collisions = false;   // 衝突チェック OFF で IK-only

  // カートesian パスを計算
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(
      waypoints,  // Waypoints
      eef_step,       // eef_step
      jump_threshold,        // jump_threshold
      trajectory,
      avoid_collisions);

  RCLCPP_INFO(node->get_logger(),
    "Cartesian IK-only planned %.1f%% (eef_step=%.3f, jump=%.2f)",
    fraction * 100.0, eef_step, jump_threshold);

  if (fraction > 0.99) {
    RCLCPP_INFO(node->get_logger(), "Cartesian path planned (%.2f%%), executing...", fraction * 100.0);
    move_group.execute(trajectory);
  } else {
    RCLCPP_WARN(node->get_logger(), "Failed to plan Cartesian path");
  }

  rclcpp::shutdown();
  return 0;
}