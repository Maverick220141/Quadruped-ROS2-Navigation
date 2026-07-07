#include <chrono>
#include <functional>
#include <string>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

// 包含机器狗的SDK头文件
#include "zsl-1/highlevel.h"

using namespace std::chrono_literals;
using namespace mc_sdk::ZSL_1;

class AgiBotBridge : public rclcpp::Node
{
public:
    AgiBotBridge()
    : Node("agibot_bridge_node")
    {
        // --- 1. 初始化并连接到机器狗 ---
        RCLCPP_INFO(this->get_logger(), "正在初始化 AgiBot SDK...");
        
        // 声明并获取 ROS 参数
        this->declare_parameter<std::string>("local_ip", "192.168.168.1");
        this->declare_parameter<int>("local_port", 43988);
        this->declare_parameter<std::string>("dog_ip", "192.168.168.168");
        this->declare_parameter<double>("state_publish_rate", 100.0);  // 状态发布频率

        std::string local_ip = this->get_parameter("local_ip").as_string();
        int local_port = this->get_parameter("local_port").as_int();
        std::string dog_ip = this->get_parameter("dog_ip").as_string();
        double state_rate = this->get_parameter("state_publish_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "连接参数 -> 本机IP: %s, 机器狗IP: %s", 
            local_ip.c_str(), dog_ip.c_str());
        
        // 创建 SDK 实例
        agibot_sdk_ = std::make_unique<HighLevel>();
        agibot_sdk_->initRobot(local_ip, local_port, dog_ip);
        
        std::this_thread::sleep_for(500ms);

        if (!agibot_sdk_->checkConnect()) {
            RCLCPP_FATAL(this->get_logger(), "连接机器狗失败！节点将关闭。");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "成功连接到机器狗。");
        
        // 发送站立指令
        RCLCPP_INFO(this->get_logger(), "发送 standUp (站立) 指令...");
        agibot_sdk_->standUp();
        std::this_thread::sleep_for(2s);

        // 发送趴下指令 (确保它保持趴着)
        // RCLCPP_INFO(this->get_logger(), "初始化完成，保持趴下状态 (lieDown)...");
        // agibot_sdk_->lieDown();
        // std::this_thread::sleep_for(1s);

        // --- 2. 创建 ROS 发布者 ---

        // 机器狗速度（机体坐标系，含线速度和角速度）
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/dog/velocity", 50);

        // 机器狗 IMU 数据（加速度 + 角速度 + 姿态）
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/dog/imu", 100);

        // 机器狗里程计（位置会漂移，但速度可用）
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/dog/odom", 50);

        // --- 3. 创建 ROS 订阅者 ---

        // cmd_vel 订阅者
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&AgiBotBridge::cmd_vel_callback, this, std::placeholders::_1));
        
        // --- 4. 创建定时器 ---

        // 状态发布定时器
        int period_ms = static_cast<int>(1000.0 / state_rate);
        state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms), 
            std::bind(&AgiBotBridge::state_publish_callback, this));

        // 安全看门狗定时器  
        watchdog_timer_ = this->create_wall_timer(
            100ms, 
            std::bind(&AgiBotBridge::watchdog_callback, this));

        RCLCPP_INFO(this->get_logger(), 
            "AgiBot Bridge 已启动 (状态发布频率: %.0fHz)", state_rate);
    }
    
    HighLevel* get_sdk_instance() {
        return agibot_sdk_.get();
    }

private:
    // =========================================================================
    // 状态发布回调（核心函数）
    // =========================================================================
    void state_publish_callback()
    {
        try {
            rclcpp::Time now = this->get_clock()->now();
            
            // --- 获取所有状态数据 ---
            std::vector<float> body_vel = agibot_sdk_->getBodyVelocity();    // 机体线速度
            std::vector<float> body_gyro = agibot_sdk_->getBodyGyro();       // 机体角速度
            std::vector<float> body_acc = agibot_sdk_->getBodyAcc();         // 机体加速度
            std::vector<float> quat = agibot_sdk_->getQuaternion();          // 姿态四元数
            std::vector<float> rpy = agibot_sdk_->getRPY();                  // 欧拉角
            std::vector<float> position = agibot_sdk_->getPosition();        // 位置（会漂移）
            std::vector<float> world_vel = agibot_sdk_->getWorldVelocity();  // 世界速度
            
            // --- 1. 发布速度 (TwistStamped) ---
            if (body_vel.size() >= 3 && body_gyro.size() >= 3) {
                geometry_msgs::msg::TwistStamped vel_msg;
                vel_msg.header.stamp = now;
                vel_msg.header.frame_id = "base_link";
                
                // 线速度（机体坐标系）
                vel_msg.twist.linear.x = static_cast<double>(body_vel[0]);
                vel_msg.twist.linear.y = static_cast<double>(body_vel[1]);
                vel_msg.twist.linear.z = static_cast<double>(body_vel[2]);
                
                // 角速度（机体坐标系）
                vel_msg.twist.angular.x = static_cast<double>(body_gyro[0]);
                vel_msg.twist.angular.y = static_cast<double>(body_gyro[1]);
                vel_msg.twist.angular.z = static_cast<double>(body_gyro[2]);
                
                velocity_pub_->publish(vel_msg);
            }
            
            // --- 2. 发布 IMU 数据 ---
            if (body_acc.size() >= 3 && body_gyro.size() >= 3 && quat.size() >= 4) {
                sensor_msgs::msg::Imu imu_msg;
                imu_msg.header.stamp = now;
                imu_msg.header.frame_id = "base_link";
                
                // 姿态四元数 [w, x, y, z]
                imu_msg.orientation.w = static_cast<double>(quat[0]);
                imu_msg.orientation.x = static_cast<double>(quat[1]);
                imu_msg.orientation.y = static_cast<double>(quat[2]);
                imu_msg.orientation.z = static_cast<double>(quat[3]);
                
                // 角速度
                imu_msg.angular_velocity.x = static_cast<double>(body_gyro[0]);
                imu_msg.angular_velocity.y = static_cast<double>(body_gyro[1]);
                imu_msg.angular_velocity.z = static_cast<double>(body_gyro[2]);
                
                // 线加速度
                imu_msg.linear_acceleration.x = static_cast<double>(body_acc[0]);
                imu_msg.linear_acceleration.y = static_cast<double>(body_acc[1]);
                imu_msg.linear_acceleration.z = static_cast<double>(body_acc[2]);
                
                // 协方差设置
                // 姿态协方差：roll/pitch 可信，yaw 会漂移
                imu_msg.orientation_covariance[0] = 0.01;  // roll
                imu_msg.orientation_covariance[4] = 0.01;  // pitch
                imu_msg.orientation_covariance[8] = 1.0;   // yaw（不太可信）
                
                // 角速度协方差
                imu_msg.angular_velocity_covariance[0] = 0.001;
                imu_msg.angular_velocity_covariance[4] = 0.001;
                imu_msg.angular_velocity_covariance[8] = 0.001;
                
                // 线加速度协方差
                imu_msg.linear_acceleration_covariance[0] = 0.01;
                imu_msg.linear_acceleration_covariance[4] = 0.01;
                imu_msg.linear_acceleration_covariance[8] = 0.01;
                
                imu_pub_->publish(imu_msg);
            }
            
            // --- 3. 发布里程计 ---
            if (position.size() >= 3 && quat.size() >= 4 && 
                body_vel.size() >= 3 && body_gyro.size() >= 3) {
                
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = now;
                odom_msg.header.frame_id = "dog_odom";  // 机器狗自己的 odom 坐标系
                odom_msg.child_frame_id = "base_link";
                
                // 位置（注意：会漂移，仅供参考）
                odom_msg.pose.pose.position.x = static_cast<double>(position[0]);
                odom_msg.pose.pose.position.y = static_cast<double>(position[1]);
                odom_msg.pose.pose.position.z = static_cast<double>(position[2]);
                
                // 姿态
                odom_msg.pose.pose.orientation.w = static_cast<double>(quat[0]);
                odom_msg.pose.pose.orientation.x = static_cast<double>(quat[1]);
                odom_msg.pose.pose.orientation.y = static_cast<double>(quat[2]);
                odom_msg.pose.pose.orientation.z = static_cast<double>(quat[3]);
                
                // 位置协方差（yaw 相关的协方差设置较大，表示不可信）
                odom_msg.pose.covariance[0] = 0.1;    // x
                odom_msg.pose.covariance[7] = 0.1;    // y
                odom_msg.pose.covariance[14] = 0.1;   // z
                odom_msg.pose.covariance[21] = 0.01;  // roll
                odom_msg.pose.covariance[28] = 0.01;  // pitch
                odom_msg.pose.covariance[35] = 100.0; // yaw（不可信，设置很大）
                
                // 速度（机体坐标系，可信赖）
                odom_msg.twist.twist.linear.x = static_cast<double>(body_vel[0]);
                odom_msg.twist.twist.linear.y = static_cast<double>(body_vel[1]);
                odom_msg.twist.twist.linear.z = static_cast<double>(body_vel[2]);
                odom_msg.twist.twist.angular.x = static_cast<double>(body_gyro[0]);
                odom_msg.twist.twist.angular.y = static_cast<double>(body_gyro[1]);
                odom_msg.twist.twist.angular.z = static_cast<double>(body_gyro[2]);
                
                // 速度协方差（可信赖）
                odom_msg.twist.covariance[0] = 0.01;   // vx
                odom_msg.twist.covariance[7] = 0.01;   // vy
                odom_msg.twist.covariance[14] = 0.01;  // vz
                odom_msg.twist.covariance[21] = 0.001; // wx
                odom_msg.twist.covariance[28] = 0.001; // wy
                odom_msg.twist.covariance[35] = 0.001; // wz
                
                odom_pub_->publish(odom_msg);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "获取机器狗状态失败: %s", e.what());
        }
    }

    // =========================================================================
    // /cmd_vel 回调
    // =========================================================================
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float vx = static_cast<float>(msg->linear.x);
        float vy = static_cast<float>(msg->linear.y);
        float wz = static_cast<float>(msg->angular.z);

        // 死区过滤
        if (std::abs(vx) < 0.05) vx = 0.0f;
        if (std::abs(vy) < 0.05) vy = 0.0f;
        if (std::abs(wz) < 0.02) wz = 0.0f;

        agibot_sdk_->move(vx, vy, wz);
        last_cmd_vel_time_ = this->get_clock()->now();
    }

    // =========================================================================
    // 看门狗回调
    // =========================================================================
    void watchdog_callback()
    {
        const auto time_since_last_cmd = this->get_clock()->now() - last_cmd_vel_time_;
        if (time_since_last_cmd > 500ms)
        {
            agibot_sdk_->move(0.0f, 0.0f, 0.0f);
        }
    }

    // ROS 成员变量
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};

    // SDK 实例
    std::unique_ptr<HighLevel> agibot_sdk_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto agibot_bridge_node = std::make_shared<AgiBotBridge>();
    
    // 注册安全关闭回调
    rclcpp::on_shutdown([&agibot_bridge_node]() {
        if (!agibot_bridge_node) { return; }

        RCLCPP_INFO(agibot_bridge_node->get_logger(), "接收到关闭信号...");
        
        agibot_bridge_node->get_sdk_instance()->lieDown();
        std::this_thread::sleep_for(2s);
        
        agibot_bridge_node->get_sdk_instance()->passive();
        std::this_thread::sleep_for(500ms); 
        
        RCLCPP_INFO(agibot_bridge_node->get_logger(), "机器狗已进入安全状态。");
    });

    rclcpp::spin(agibot_bridge_node);
    rclcpp::shutdown();
    
    return 0;
}
