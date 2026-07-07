#ifndef MY_NAV__VIRTUAL_OBSTACLE_LAYER_HPP_
#define MY_NAV__VIRTUAL_OBSTACLE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_nav
{

class VirtualObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  VirtualObstacleLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  void obstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  std::mutex mutex_;
  std::vector<geometry_msgs::msg::Pose> obstacles_;
  std::string global_frame_;
  std::string topic_;
  double obstacle_radius_{0.55};
  unsigned char obstacle_cost_{nav2_costmap_2d::LETHAL_OBSTACLE};
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
};

}  // namespace my_nav

#endif  // MY_NAV__VIRTUAL_OBSTACLE_LAYER_HPP_
