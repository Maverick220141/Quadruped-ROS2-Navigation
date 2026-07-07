// Copyright 2026 Cowain
// Implementation of RotateToGoalHeading BT action node.

#include "my_nav/rotate_to_goal_heading_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.h"

namespace my_nav
{

RotateToGoalHeading::RotateToGoalHeading(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(action_name, conf),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  goal_yaw_(0.0),
  tolerance_(0.10),
  max_angular_speed_(0.8),
  min_angular_speed_(0.25),
  p_gain_(2.0),
  timeout_sec_(8.0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::NodeStatus RotateToGoalHeading::onStart()
{
  // Read all input ports (do this every onStart, not in constructor —
  // the constructor runs before blackboard is populated)
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_ERROR(node_->get_logger(), "RotateToGoalHeading: missing 'goal' input");
    return BT::NodeStatus::FAILURE;
  }

  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  getInput("tolerance", tolerance_);
  getInput("max_angular_speed", max_angular_speed_);
  getInput("min_angular_speed", min_angular_speed_);
  getInput("p_gain", p_gain_);
  getInput("timeout", timeout_sec_);

  std::string cmd_vel_topic = "/cmd_vel";
  getInput("cmd_vel_topic", cmd_vel_topic);

  // Lazily create publisher (first onStart only)
  if (!cmd_vel_pub_ || cmd_vel_pub_->get_topic_name() != std::string(cmd_vel_topic)) {
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, qos);
  }

  goal_yaw_ = tf2::getYaw(goal.pose.orientation);
  start_time_ = node_->now();

  // 诊断日志：打印 robot 当前位置 vs goal 位置距离
  // 用于确认 FollowPath 是否真的执行过（如果距离 > xy_tol 但本节点启动了，说明 FollowPath 把狗带到了 xy_tol 内）
  geometry_msgs::msg::PoseStamped current_pose;
  if (nav2_util::getCurrentPose(current_pose, *tf_, global_frame_, robot_base_frame_, 0.5)) {
    const double dx = goal.pose.position.x - current_pose.pose.position.x;
    const double dy = goal.pose.position.y - current_pose.pose.position.y;
    const double dist = std::hypot(dx, dy);
    const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
    double yaw_err = goal_yaw_ - current_yaw;
    while (yaw_err > M_PI) {yaw_err -= 2.0 * M_PI;}
    while (yaw_err < -M_PI) {yaw_err += 2.0 * M_PI;}

    RCLCPP_INFO(
      node_->get_logger(),
      "RotateToGoalHeading START: robot=(%.3f,%.3f,yaw=%.3f) goal=(%.3f,%.3f,yaw=%.3f) "
      "dist_to_goal=%.3fm yaw_err=%.3frad(%.1fdeg) tol=%.3f max=%.2f min=%.2f",
      current_pose.pose.position.x, current_pose.pose.position.y, current_yaw,
      goal.pose.position.x, goal.pose.position.y, goal_yaw_,
      dist, yaw_err, yaw_err * 180.0 / M_PI,
      tolerance_, max_angular_speed_, min_angular_speed_);
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "RotateToGoalHeading START: cannot get robot pose; goal_yaw=%.3f",
      goal_yaw_);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateToGoalHeading::onRunning()
{
  // Timeout guard
  const double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed > timeout_sec_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "RotateToGoalHeading: timeout (%.1fs > %.1fs), giving up",
      elapsed, timeout_sec_);
    stopRobot();
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, 0.5))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "RotateToGoalHeading: cannot get robot pose in '%s'", global_frame_.c_str());
    return BT::NodeStatus::RUNNING;  // Try again next tick
  }

  const double current_yaw = tf2::getYaw(current_pose.pose.orientation);

  // Shortest signed delta in [-pi, pi]
  double err = goal_yaw_ - current_yaw;
  while (err > M_PI) {err -= 2.0 * M_PI;}
  while (err < -M_PI) {err += 2.0 * M_PI;}

  // Convergence check
  if (std::abs(err) < tolerance_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "RotateToGoalHeading converged: current_yaw=%.3f goal=%.3f err=%.3f (tol=%.3f) elapsed=%.2fs",
      current_yaw, goal_yaw_, err, tolerance_, elapsed);
    stopRobot();
    return BT::NodeStatus::SUCCESS;
  }

  // P controller with min-speed floor (overcome dog yaw dead zone)
  double cmd = p_gain_ * err;
  cmd = std::clamp(cmd, -max_angular_speed_, max_angular_speed_);
  if (std::abs(cmd) < min_angular_speed_) {
    cmd = std::copysign(min_angular_speed_, err);
  }

  publishVel(cmd);
  return BT::NodeStatus::RUNNING;
}

void RotateToGoalHeading::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "RotateToGoalHeading halted, stopping robot");
  stopRobot();
}

void RotateToGoalHeading::publishVel(double angular_z)
{
  if (!cmd_vel_pub_) {
    return;
  }
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;
  cmd_vel_pub_->publish(twist);
}

void RotateToGoalHeading::stopRobot()
{
  // Publish a few zeros to ensure smoother also flushes
  for (int i = 0; i < 3; ++i) {
    publishVel(0.0);
  }
}

ClearCostmapsOnce::ClearCostmapsOnce(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(action_name, conf),
  timeout_sec_(1.5),
  clear_local_(true),
  clear_global_(true),
  local_sent_(false),
  global_sent_(false),
  local_done_(std::make_shared<std::atomic_bool>(false)),
  global_done_(std::make_shared<std::atomic_bool>(false))
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus ClearCostmapsOnce::onStart()
{
  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    RCLCPP_WARN(node_->get_logger(), "ClearCostmapsOnce: missing goal, skip startup clear");
    return BT::NodeStatus::SUCCESS;
  }

  getInput("clear_local", clear_local_);
  getInput("clear_global", clear_global_);
  getInput("local_service", local_service_);
  getInput("global_service", global_service_);
  getInput("timeout", timeout_sec_);

  const std::string goal_key = makeGoalKey(goal);
  if (goal_key == cleared_goal_key_) {
    return BT::NodeStatus::SUCCESS;
  }

  resetForGoal(goal_key);
  createClientsIfNeeded();
  sendRequestsIfReady();

  if ((!clear_local_ || local_sent_) && (!clear_global_ || global_sent_)) {
    return onRunning();
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ClearCostmapsOnce::onRunning()
{
  sendRequestsIfReady();

  const bool local_done = !clear_local_ || local_done_->load();
  const bool global_done = !clear_global_ || global_done_->load();
  if (local_done && global_done) {
    cleared_goal_key_ = active_goal_key_;
    RCLCPP_INFO(node_->get_logger(), "ClearCostmapsOnce: startup costmap clear complete");
    return BT::NodeStatus::SUCCESS;
  }

  const double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed > timeout_sec_) {
    cleared_goal_key_ = active_goal_key_;
    RCLCPP_WARN(
      node_->get_logger(),
      "ClearCostmapsOnce: timeout after %.2fs, continue navigation", elapsed);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ClearCostmapsOnce::onHalted()
{
}

std::string ClearCostmapsOnce::makeGoalKey(
  const geometry_msgs::msg::PoseStamped & goal) const
{
  std::ostringstream out;
  out.setf(std::ios::fixed, std::ios::floatfield);
  out.precision(3);
  out << goal.header.frame_id << ':'
      << goal.pose.position.x << ':'
      << goal.pose.position.y << ':'
      << tf2::getYaw(goal.pose.orientation);
  return out.str();
}

void ClearCostmapsOnce::resetForGoal(const std::string & goal_key)
{
  active_goal_key_ = goal_key;
  start_time_ = node_->now();
  local_sent_ = false;
  global_sent_ = false;
  local_done_->store(false);
  global_done_->store(false);
}

void ClearCostmapsOnce::createClientsIfNeeded()
{
  if (clear_local_ && (!local_client_ || local_client_->get_service_name() != local_service_)) {
    local_client_ = node_->create_client<ClearCostmap>(local_service_);
  }
  if (clear_global_ && (!global_client_ || global_client_->get_service_name() != global_service_)) {
    global_client_ = node_->create_client<ClearCostmap>(global_service_);
  }
}

void ClearCostmapsOnce::sendRequestsIfReady()
{
  if (clear_local_ && !local_sent_ && local_client_ && local_client_->service_is_ready()) {
    RCLCPP_INFO(node_->get_logger(), "ClearCostmapsOnce: clearing local costmap");
    auto request = std::make_shared<ClearCostmap::Request>();
    auto done = local_done_;
    local_client_->async_send_request(
      request,
      [done](rclcpp::Client<ClearCostmap>::SharedFuture) {
        done->store(true);
      });
    local_sent_ = true;
  }

  if (clear_global_ && !global_sent_ && global_client_ && global_client_->service_is_ready()) {
    RCLCPP_INFO(node_->get_logger(), "ClearCostmapsOnce: clearing global costmap");
    auto request = std::make_shared<ClearCostmap::Request>();
    auto done = global_done_;
    global_client_->async_send_request(
      request,
      [done](rclcpp::Client<ClearCostmap>::SharedFuture) {
        done->store(true);
      });
    global_sent_ = true;
  }
}

}  // namespace my_nav

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<my_nav::RotateToGoalHeading>(name, config);
    };
  factory.registerBuilder<my_nav::RotateToGoalHeading>("RotateToGoalHeading", builder);

  BT::NodeBuilder clear_builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<my_nav::ClearCostmapsOnce>(name, config);
    };
  factory.registerBuilder<my_nav::ClearCostmapsOnce>("ClearCostmapsOnce", clear_builder);
}
