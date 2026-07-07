#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class FootprintPublisher : public rclcpp::Node
{
public:
    FootprintPublisher()
    : Node("footprint_publisher_node")
    {
        // 1. 初始化 TF 监听器 (监听 Fast_LIO 的输出)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 2. 初始化 TF 广播器 (发布 2D footprint)
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 3. 高频定时器 (50Hz)，保证 footprint 跟随流畅
        timer_ = this->create_wall_timer(
            20ms, std::bind(&FootprintPublisher::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "虚拟底盘坐标系节点已启动: odom -> base_footprint");
    }

private:
    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped transform_in;
        try {
            // Query the composed 3D body transform from the TF tree.
            // timeout 设为 0.05s，避免阻塞
            // transform_in = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            transform_in = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero, 50ms);
        } catch (const tf2::TransformException & ex) {
            // 刚启动时可能会查不到，warn 一下即可
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "等待 Fast_LIO TF: %s", ex.what());
            return;
        }

        // --- 核心逻辑：3D 压扁为 2D ---

        // 1. 提取四元数并转为 RPY
        tf2::Quaternion q(
            transform_in.transform.rotation.x,
            transform_in.transform.rotation.y,
            transform_in.transform.rotation.z,
            transform_in.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 2. 构建新的 2D 四元数 (只保留 Yaw)
        tf2::Quaternion q_2d;
        q_2d.setRPY(0.0, 0.0, yaw);

        // 3. 构建输出变换
        geometry_msgs::msg::TransformStamped transform_out;
        transform_out.header.stamp = transform_in.header.stamp; // 时间戳同步
        transform_out.header.frame_id = "odom";                 // 父坐标系同为 odom
        transform_out.child_frame_id = "base_footprint";        // 子坐标系为新名字

        // 4. 坐标赋值
        transform_out.transform.translation.x = transform_in.transform.translation.x;
        transform_out.transform.translation.y = transform_in.transform.translation.y;
        
        // 关键点：Z 轴强制归零 (或者根据你的雷达安装高度设为 -0.3 等，通常设 0 配合 2D 地图)
        transform_out.transform.translation.z = 0.0; 

        transform_out.transform.rotation.x = q_2d.x();
        transform_out.transform.rotation.y = q_2d.y();
        transform_out.transform.rotation.z = q_2d.z();
        transform_out.transform.rotation.w = q_2d.w();

        // 5. 发布
        tf_broadcaster_->sendTransform(transform_out);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FootprintPublisher>());
    rclcpp::shutdown();
    return 0;
}