#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"

using std::placeholders::_1;

class LivoxRepub : public rclcpp::Node
{
public:
    LivoxRepub() : Node("livox_repub_node")
    {
        // 声明参数：输入和输出的话题名
        this->declare_parameter<std::string>("input_topic", "/livox/lidar");
        this->declare_parameter<std::string>("output_topic", "/livox/lidar/pointcloud");
        this->declare_parameter<bool>("use_current_time_stamp", false);

        std::string input_topic;
        std::string output_topic;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("output_topic", output_topic);
        this->get_parameter("use_current_time_stamp", use_current_time_stamp_);

        RCLCPP_INFO(this->get_logger(), "Listening to: %s", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "Timestamp mode: %s",
            use_current_time_stamp_ ? "replace with node now()" : "preserve Livox header stamp");

        // 创建发布者 (PointCloud2)
        // Best Effort QoS 通常对传感器数据更好
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        // 创建订阅者 (CustomMsg)
        subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            input_topic,
            10,
            std::bind(&LivoxRepub::topic_callback, this, _1));
    }

private:
    void topic_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) const
    {
        // 1. 创建 PointCloud2 消息
        auto pc2_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        
        // 2. 填充头部信息
        pc2_msg->header = msg->header; // 继承 Frame ID 和 时间戳
        pc2_msg->height = 1;
        pc2_msg->width = msg->point_num;
        pc2_msg->is_dense = true;
        pc2_msg->is_bigendian = false;

        if (use_current_time_stamp_) {
            pc2_msg->header.stamp = this->now();
        }
        // 3. 定义点云字段 (x, y, z, intensity)
        sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        modifier.resize(msg->point_num);

        // 4. 创建迭代器进行填充
        sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(*pc2_msg, "intensity");

        // 5. 遍历 CustomMsg 并赋值
        for (size_t i = 0; i < msg->point_num; ++i) {
            *iter_x = msg->points[i].x;
            *iter_y = msg->points[i].y;
            *iter_z = msg->points[i].z;
            *iter_i = static_cast<float>(msg->points[i].reflectivity);

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_i;
        }

        // 6. 发布
        publisher_->publish(std::move(pc2_msg));
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    bool use_current_time_stamp_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxRepub>());
    rclcpp::shutdown();
    return 0;
}
