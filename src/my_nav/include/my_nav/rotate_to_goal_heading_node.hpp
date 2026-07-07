// Copyright 2026 Cowain
// Custom Nav2 BT action node: rotate-in-place to align robot yaw to goal yaw.
//
// 设计目的：解决 nav2 humble 的 Spin 节点 bug —— Spin 在构造函数读 spin_dist，
// 那时 blackboard 中的 {yaw_delta} 还没赋值，导致 Spin 总是 spin_dist=0。
//
// 本节点是 StatefulActionNode：
//   - onStart()  ：第一次 tick，从 TF 读当前 yaw，从 goal 读目标 yaw，记录到成员变量
//   - onRunning()：每次 tick，重新读 TF 当前 yaw 计算误差，做 P 控制发 cmd_vel.angular
//                  误差 < tolerance 时发 0 并返回 SUCCESS
//   - onHalted() ：BT 中止时发 0 停止旋转
//
// 用法（BT XML）：
//   <RotateToGoalHeading
//      goal="{goal}"
//      tolerance="0.10"
//      max_angular_speed="0.8"
//      p_gain="2.0"
//      cmd_vel_topic="/cmd_vel"/>

#ifndef MY_NAV__ROTATE_TO_GOAL_HEADING_NODE_HPP_
#define MY_NAV__ROTATE_TO_GOAL_HEADING_NODE_HPP_

#include <memory>
#include <atomic>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace my_nav
{

class RotateToGoalHeading : public BT::StatefulActionNode
{
public:
  RotateToGoalHeading(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  RotateToGoalHeading() = delete;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot frame"),
      BT::InputPort<double>("tolerance", 0.10, "Yaw error tolerance (rad). |err|<tol => SUCCESS"),
      BT::InputPort<double>("max_angular_speed", 0.8, "Max angular speed (rad/s)"),
      BT::InputPort<double>("min_angular_speed", 0.25,
        "Min angular speed when |err|>tol to overcome dog dead zone (rad/s)"),
      BT::InputPort<double>("p_gain", 2.0, "Proportional gain"),
      BT::InputPort<std::string>("cmd_vel_topic", std::string("/cmd_vel"),
        "Topic to publish angular velocity to"),
      BT::InputPort<double>("timeout", 8.0, "Timeout in seconds, FAILURE on timeout"),
    };
  }

private:
  void publishVel(double angular_z);
  void stopRobot();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Cached params (read once at onStart)
  std::string global_frame_;
  std::string robot_base_frame_;
  double goal_yaw_;
  double tolerance_;
  double max_angular_speed_;
  double min_angular_speed_;
  double p_gain_;
  rclcpp::Time start_time_;
  double timeout_sec_;
};

class ClearCostmapsOnce : public BT::StatefulActionNode
{
public:
  ClearCostmapsOnce(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  ClearCostmapsOnce() = delete;

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal"),
      BT::InputPort<bool>("clear_local", true, "Clear local costmap once"),
      BT::InputPort<bool>("clear_global", true, "Clear global costmap once"),
      BT::InputPort<std::string>(
        "local_service", std::string("local_costmap/clear_entirely_local_costmap"),
        "Local costmap clear service"),
      BT::InputPort<std::string>(
        "global_service", std::string("global_costmap/clear_entirely_global_costmap"),
        "Global costmap clear service"),
      BT::InputPort<double>("timeout", 1.5, "Max seconds to wait before continuing"),
    };
  }

private:
  using ClearCostmap = nav2_msgs::srv::ClearEntireCostmap;

  std::string makeGoalKey(const geometry_msgs::msg::PoseStamped & goal) const;
  void resetForGoal(const std::string & goal_key);
  void createClientsIfNeeded();
  void sendRequestsIfReady();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<ClearCostmap>::SharedPtr local_client_;
  rclcpp::Client<ClearCostmap>::SharedPtr global_client_;

  std::string local_service_;
  std::string global_service_;
  std::string active_goal_key_;
  std::string cleared_goal_key_;
  rclcpp::Time start_time_;
  double timeout_sec_;
  bool clear_local_;
  bool clear_global_;
  bool local_sent_;
  bool global_sent_;
  std::shared_ptr<std::atomic_bool> local_done_;
  std::shared_ptr<std::atomic_bool> global_done_;
};

}  // namespace my_nav

#endif  // MY_NAV__ROTATE_TO_GOAL_HEADING_NODE_HPP_
