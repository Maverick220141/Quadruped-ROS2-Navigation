#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace my_nav
{

class BlockedPathMonitor : public rclcpp::Node
{
public:
  BlockedPathMonitor()
  : Node("blocked_path_monitor"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    plan_topic_ = declare_parameter<std::string>("plan_topic", "/plan");
    local_costmap_topic_ = declare_parameter<std::string>(
      "local_costmap_topic", "/local_costmap/costmap");
    virtual_obstacles_topic_ = declare_parameter<std::string>(
      "virtual_obstacles_topic", "/blocked_path/virtual_obstacles");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    min_check_distance_ = declare_parameter<double>("min_check_distance", 0.8);
    lookahead_distance_ = declare_parameter<double>("lookahead_distance", 2.0);
    path_check_radius_ = declare_parameter<double>("path_check_radius", 0.35);
    block_radius_ = declare_parameter<double>("block_radius", 0.55);
    block_ttl_sec_ = declare_parameter<double>("block_ttl_sec", 60.0);
    blocked_hold_sec_ = declare_parameter<double>("blocked_hold_sec", 2.0);
    merge_distance_ = declare_parameter<double>("merge_distance", 0.8);
    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 80);
    min_blocked_samples_ = declare_parameter<int>("min_blocked_samples", 2);
    timer_period_ms_ = declare_parameter<int>("timer_period_ms", 200);

    plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      plan_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&BlockedPathMonitor::planCallback, this, std::placeholders::_1));

    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&BlockedPathMonitor::costmapCallback, this, std::placeholders::_1));

    obstacles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      virtual_obstacles_topic_, rclcpp::SystemDefaultsQoS());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(timer_period_ms_),
      std::bind(&BlockedPathMonitor::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "BlockedPathMonitor: plan=%s local_costmap=%s output=%s ttl=%.1fs",
      plan_topic_.c_str(), local_costmap_topic_.c_str(),
      virtual_obstacles_topic_.c_str(), block_ttl_sec_);
  }

private:
  struct Block
  {
    double x{0.0};
    double y{0.0};
    rclcpp::Time expires_at;
  };

  void planCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    plan_ = msg;
  }

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    local_costmap_ = msg;
  }

  void onTimer()
  {
    const auto now = get_clock()->now();
    purgeExpired(now);

    geometry_msgs::msg::PoseStamped blocked_pose;
    if (isCurrentPathBlocked(blocked_pose)) {
      if (!blocked_since_.nanoseconds()) {
        blocked_since_ = now;
      }
      if ((now - blocked_since_).seconds() >= blocked_hold_sec_) {
        addOrRefreshBlock(blocked_pose, now);
        blocked_since_ = now;
      }
    } else {
      blocked_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }

    publishBlocks(now);
  }

  bool isCurrentPathBlocked(geometry_msgs::msg::PoseStamped & blocked_pose)
  {
    if (!plan_ || plan_->poses.empty() || !local_costmap_) {
      return false;
    }

    geometry_msgs::msg::PoseStamped robot_in_plan;
    if (!lookupRobotPose(plan_->header.frame_id, robot_in_plan)) {
      return false;
    }

    const int nearest = nearestPathIndex(robot_in_plan.pose.position.x, robot_in_plan.pose.position.y);
    if (nearest < 0) {
      return false;
    }

    double traveled = 0.0;
    int blocked_samples = 0;
    bool first_block_set = false;
    geometry_msgs::msg::PoseStamped first_block;

    for (size_t i = static_cast<size_t>(nearest); i < plan_->poses.size(); ++i) {
      if (i > static_cast<size_t>(nearest)) {
        const auto & a = plan_->poses[i - 1].pose.position;
        const auto & b = plan_->poses[i].pose.position;
        traveled += std::hypot(b.x - a.x, b.y - a.y);
      }
      if (traveled < min_check_distance_) {
        continue;
      }
      if (traveled > lookahead_distance_) {
        break;
      }

      geometry_msgs::msg::PoseStamped pose_in_costmap;
      if (!transformPose(plan_->poses[i], local_costmap_->header.frame_id, pose_in_costmap)) {
        continue;
      }

      if (isPoseOccupied(pose_in_costmap.pose.position.x, pose_in_costmap.pose.position.y)) {
        ++blocked_samples;
        if (!first_block_set) {
          first_block = plan_->poses[i];
          first_block_set = true;
        }
      }
    }

    if (blocked_samples < min_blocked_samples_ || !first_block_set) {
      return false;
    }

    if (!transformPose(first_block, map_frame_, blocked_pose)) {
      return false;
    }
    return true;
  }

  bool lookupRobotPose(const std::string & target_frame, geometry_msgs::msg::PoseStamped & out)
  {
    try {
      auto tf = tf_buffer_.lookupTransform(
        target_frame, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
      out.header = tf.header;
      out.pose.position.x = tf.transform.translation.x;
      out.pose.position.y = tf.transform.translation.y;
      out.pose.position.z = tf.transform.translation.z;
      out.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "BlockedPathMonitor TF robot lookup failed: %s", ex.what());
      return false;
    }
  }

  bool transformPose(
    const geometry_msgs::msg::PoseStamped & in,
    const std::string & target_frame,
    geometry_msgs::msg::PoseStamped & out)
  {
    try {
      tf_buffer_.transform(in, out, target_frame, tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "BlockedPathMonitor TF pose transform failed: %s", ex.what());
      return false;
    }
  }

  int nearestPathIndex(double x, double y) const
  {
    double best = std::numeric_limits<double>::max();
    int best_index = -1;
    for (size_t i = 0; i < plan_->poses.size(); ++i) {
      const auto & p = plan_->poses[i].pose.position;
      const double d = std::hypot(p.x - x, p.y - y);
      if (d < best) {
        best = d;
        best_index = static_cast<int>(i);
      }
    }
    return best_index;
  }

  bool isPoseOccupied(double x, double y) const
  {
    const auto & info = local_costmap_->info;
    const double res = info.resolution;
    const double origin_x = info.origin.position.x;
    const double origin_y = info.origin.position.y;
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(path_check_radius_ / res)));
    const int mx = static_cast<int>(std::floor((x - origin_x) / res));
    const int my = static_cast<int>(std::floor((y - origin_y) / res));

    if (mx < 0 || my < 0 ||
      mx >= static_cast<int>(info.width) || my >= static_cast<int>(info.height))
    {
      return false;
    }

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int cx = mx + dx;
        const int cy = my + dy;
        if (cx < 0 || cy < 0 ||
          cx >= static_cast<int>(info.width) || cy >= static_cast<int>(info.height))
        {
          continue;
        }
        const int index = cy * static_cast<int>(info.width) + cx;
        if (index >= 0 && index < static_cast<int>(local_costmap_->data.size()) &&
          local_costmap_->data[index] >= occupied_threshold_)
        {
          return true;
        }
      }
    }
    return false;
  }

  void addOrRefreshBlock(const geometry_msgs::msg::PoseStamped & pose, const rclcpp::Time & now)
  {
    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;
    const auto expires_at = now + rclcpp::Duration::from_seconds(block_ttl_sec_);

    for (auto & block : blocks_) {
      if (std::hypot(block.x - x, block.y - y) <= merge_distance_) {
        block.x = 0.5 * (block.x + x);
        block.y = 0.5 * (block.y + y);
        block.expires_at = expires_at;
        return;
      }
    }

    blocks_.push_back(Block{x, y, expires_at});
    RCLCPP_WARN(
      get_logger(),
      "Path blocked ahead, adding virtual global obstacle at map=(%.2f, %.2f) for %.1fs",
      x, y, block_ttl_sec_);
  }

  void purgeExpired(const rclcpp::Time & now)
  {
    blocks_.erase(
      std::remove_if(
        blocks_.begin(), blocks_.end(),
        [&](const Block & block) {return block.expires_at <= now;}),
      blocks_.end());
  }

  void publishBlocks(const rclcpp::Time & now)
  {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = now;
    msg.header.frame_id = map_frame_;
    msg.poses.reserve(blocks_.size());
    for (const auto & block : blocks_) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = block.x;
      pose.position.y = block.y;
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }
    obstacles_pub_->publish(msg);
  }

  std::string plan_topic_;
  std::string local_costmap_topic_;
  std::string virtual_obstacles_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double min_check_distance_;
  double lookahead_distance_;
  double path_check_radius_;
  double block_radius_;
  double block_ttl_sec_;
  double blocked_hold_sec_;
  double merge_distance_;
  int occupied_threshold_;
  int min_blocked_samples_;
  int timer_period_ms_;

  nav_msgs::msg::Path::SharedPtr plan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;
  std::vector<Block> blocks_;
  rclcpp::Time blocked_since_{0, 0, get_clock()->get_clock_type()};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace my_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_nav::BlockedPathMonitor>());
  rclcpp::shutdown();
  return 0;
}
