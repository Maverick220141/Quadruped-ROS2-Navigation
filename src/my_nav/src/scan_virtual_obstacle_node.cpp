#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace my_nav
{

class ScanVirtualObstacleNode : public rclcpp::Node
{
public:
  ScanVirtualObstacleNode()
  : Node("scan_virtual_obstacle_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/blocked_path/virtual_obstacles");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    min_range_ = declare_parameter<double>("min_range", 0.30);
    max_range_ = declare_parameter<double>("max_range", 4.0);
    min_angle_ = declare_parameter<double>("min_angle", -3.141592653589793);
    max_angle_ = declare_parameter<double>("max_angle", 3.141592653589793);
    grid_resolution_ = declare_parameter<double>("grid_resolution", 0.45);
    obstacle_ttl_sec_ = declare_parameter<double>("obstacle_ttl_sec", 5.0);
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 200);
    max_obstacles_ = declare_parameter<int>("max_obstacles", 220);
    min_samples_per_obstacle_ = declare_parameter<int>("min_samples_per_obstacle", 3);
    use_latest_tf_on_failure_ = declare_parameter<bool>("use_latest_tf_on_failure", true);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanVirtualObstacleNode::scanCallback, this, std::placeholders::_1));

    obstacles_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      output_topic_, rclcpp::SystemDefaultsQoS());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(&ScanVirtualObstacleNode::publishObstacles, this));

    RCLCPP_INFO(
      get_logger(),
      "ScanVirtualObstacleNode: scan=%s output=%s range=[%.2f, %.2f] grid=%.2f ttl=%.1fs",
      scan_topic_.c_str(), output_topic_.c_str(), min_range_, max_range_,
      grid_resolution_, obstacle_ttl_sec_);
  }

private:
  struct Block
  {
    double x{0.0};
    double y{0.0};
    int samples{0};
    rclcpp::Time expires_at;
  };

  static std::int64_t keyFor(int gx, int gy)
  {
    return (static_cast<std::int64_t>(gx) << 32) ^
      static_cast<std::uint32_t>(gy);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    if (!lookupScanTransform(*msg, tf)) {
      return;
    }

    const auto now = get_clock()->now();
    const auto expires_at = now + rclcpp::Duration::from_seconds(obstacle_ttl_sec_);
    int accepted = 0;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < min_range_ || r > max_range_) {
        continue;
      }

      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      if (angle < min_angle_ || angle > max_angle_) {
        continue;
      }

      geometry_msgs::msg::PoseStamped in;
      in.header = msg->header;
      in.pose.position.x = static_cast<double>(r) * std::cos(angle);
      in.pose.position.y = static_cast<double>(r) * std::sin(angle);
      in.pose.position.z = 0.0;
      in.pose.orientation.w = 1.0;

      geometry_msgs::msg::PoseStamped out;
      tf2::doTransform(in, out, tf);

      const int gx = static_cast<int>(std::floor(out.pose.position.x / grid_resolution_));
      const int gy = static_cast<int>(std::floor(out.pose.position.y / grid_resolution_));
      const auto key = keyFor(gx, gy);

      auto & block = blocks_[key];
      if (block.samples == 0) {
        block.x = out.pose.position.x;
        block.y = out.pose.position.y;
      } else {
        const double n = static_cast<double>(block.samples);
        block.x = (block.x * n + out.pose.position.x) / (n + 1.0);
        block.y = (block.y * n + out.pose.position.y) / (n + 1.0);
      }
      ++block.samples;
      block.expires_at = expires_at;
      ++accepted;
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ScanVirtualObstacleNode accepted %d scan points, active blocks=%zu",
      accepted, blocks_.size());
  }

  bool lookupScanTransform(
    const sensor_msgs::msg::LaserScan & msg,
    geometry_msgs::msg::TransformStamped & tf)
  {
    try {
      tf = tf_buffer_.lookupTransform(
        map_frame_, msg.header.frame_id, msg.header.stamp,
        tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException & ex) {
      if (!use_latest_tf_on_failure_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Scan virtual obstacle TF at stamp failed: %s", ex.what());
        return false;
      }
    }

    try {
      tf = tf_buffer_.lookupTransform(
        map_frame_, msg.header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(0.05));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Scan virtual obstacle latest TF failed: %s", ex.what());
      return false;
    }
  }

  void publishObstacles()
  {
    const auto now = get_clock()->now();
    purgeExpired(now);

    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = now;
    msg.header.frame_id = map_frame_;

    std::vector<const Block *> active;
    active.reserve(blocks_.size());
    for (const auto & item : blocks_) {
      if (item.second.samples < min_samples_per_obstacle_) {
        continue;
      }
      active.push_back(&item.second);
    }

    std::sort(
      active.begin(), active.end(),
      [](const Block * a, const Block * b) {
        return a->samples > b->samples;
      });

    const int limit = std::min(max_obstacles_, static_cast<int>(active.size()));
    msg.poses.reserve(static_cast<size_t>(limit));
    for (int i = 0; i < limit; ++i) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = active[static_cast<size_t>(i)]->x;
      pose.position.y = active[static_cast<size_t>(i)]->y;
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    obstacles_pub_->publish(msg);
  }

  void purgeExpired(const rclcpp::Time & now)
  {
    for (auto it = blocks_.begin(); it != blocks_.end();) {
      if (it->second.expires_at <= now) {
        it = blocks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::string scan_topic_;
  std::string output_topic_;
  std::string map_frame_;
  double min_range_;
  double max_range_;
  double min_angle_;
  double max_angle_;
  double grid_resolution_;
  double obstacle_ttl_sec_;
  int publish_period_ms_;
  int max_obstacles_;
  int min_samples_per_obstacle_;
  bool use_latest_tf_on_failure_;

  std::unordered_map<std::int64_t, Block> blocks_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacles_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace my_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<my_nav::ScanVirtualObstacleNode>());
  rclcpp::shutdown();
  return 0;
}
