#include "my_nav/virtual_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace my_nav
{

void VirtualObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("VirtualObstacleLayer failed to lock lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/blocked_path/virtual_obstacles")));
  declareParameter("obstacle_radius", rclcpp::ParameterValue(0.55));
  declareParameter("obstacle_cost", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".obstacle_radius", obstacle_radius_);

  int obstacle_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
  node->get_parameter(name_ + ".obstacle_cost", obstacle_cost);
  obstacle_cost_ = static_cast<unsigned char>(
    std::clamp(obstacle_cost, 0, static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));

  global_frame_ = layered_costmap_->getGlobalFrameID();
  current_ = true;

  sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
    topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&VirtualObstacleLayer::obstaclesCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "VirtualObstacleLayer active: topic=%s radius=%.2f global_frame=%s",
    topic_.c_str(), obstacle_radius_, global_frame_.c_str());
}

void VirtualObstacleLayer::obstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (msg->header.frame_id != global_frame_) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 5000,
        "Ignoring virtual obstacles in frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(), global_frame_.c_str());
    }
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_ = msg->poses;
}

void VirtualObstacleLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & pose : obstacles_) {
    *min_x = std::min(*min_x, pose.position.x - obstacle_radius_);
    *min_y = std::min(*min_y, pose.position.y - obstacle_radius_);
    *max_x = std::max(*max_x, pose.position.x + obstacle_radius_);
    *max_y = std::max(*max_y, pose.position.y + obstacle_radius_);
  }
}

void VirtualObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const double resolution = master_grid.getResolution();
  const int radius_cells = std::max(1, static_cast<int>(std::ceil(obstacle_radius_ / resolution)));

  for (const auto & pose : obstacles_) {
    unsigned int center_mx = 0;
    unsigned int center_my = 0;
    if (!master_grid.worldToMap(pose.position.x, pose.position.y, center_mx, center_my)) {
      continue;
    }

    const int cx = static_cast<int>(center_mx);
    const int cy = static_cast<int>(center_my);
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int mx = cx + dx;
        const int my = cy + dy;
        if (mx < min_i || mx >= max_i || my < min_j || my >= max_j) {
          continue;
        }
        if (mx < 0 || my < 0 ||
          mx >= static_cast<int>(master_grid.getSizeInCellsX()) ||
          my >= static_cast<int>(master_grid.getSizeInCellsY()))
        {
          continue;
        }
        master_grid.setCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), obstacle_cost_);
      }
    }
  }
}

void VirtualObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_.clear();
  current_ = true;
}

}  // namespace my_nav

PLUGINLIB_EXPORT_CLASS(my_nav::VirtualObstacleLayer, nav2_costmap_2d::Layer)
