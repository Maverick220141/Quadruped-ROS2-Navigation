#include "rclcpp/rclcpp.hpp"
#include "agibot_motion_service/srv/motion_control.hpp"
#include "agibot_motion_service/srv/get_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
// 其他#include保持不变
#include <functional>  // 确保已包含该头文件（ROS2的rclcpp已默认包含，可省略）
#include <future>      // 新增：用于异步执行
#include <cmath>       // copysign / fabs
#include <algorithm>
#include <cstdint>
#include <thread>
using namespace std::placeholders;  // 新增这一行，引入占位符命名空间

// 标准消息类型
#include "std_msgs/msg/string.hpp"
// 包含机器狗SDK头文件
#include <zsl-1/highlevel.h>

using namespace std::chrono_literals;
using namespace mc_sdk::ZSL_1;
using MotionControlSrv = agibot_motion_service::srv::MotionControl;
using GetStateSrv = agibot_motion_service::srv::GetState;

class MotionServerNode : public rclcpp::Node
{
public:
    MotionServerNode() : Node("motion_server_node")
    {
        // 1. 初始化ROS2参数
        this->declare_parameter<std::string>("local_ip", "192.168.168.1");
        this->declare_parameter<int>("local_port", 43988);
        this->declare_parameter<std::string>("dog_ip", "192.168.168.168");
        this->declare_parameter<double>("state_publish_rate", 50.0);
        this->declare_parameter<double>("voice_linear_speed_mps", 0.25);
        this->declare_parameter<double>("voice_turn_yaw_rate_radps", 0.5);

        std::string local_ip = this->get_parameter("local_ip").as_string();
        int local_port = this->get_parameter("local_port").as_int();
        std::string dog_ip = this->get_parameter("dog_ip").as_string();
        double state_rate = this->get_parameter("state_publish_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "配置参数：本地IP=%s，机器狗IP=%s，本地端口=%d",
                    local_ip.c_str(), dog_ip.c_str(), local_port);

        // 2. 初始化SDK
        sdk_ptr_ = std::make_unique<HighLevel>();
        sdk_ptr_->initRobot(local_ip, local_port, dog_ip);
        std::this_thread::sleep_for(2s);

        if (!sdk_ptr_->checkConnect()) {
            RCLCPP_FATAL(this->get_logger(), "连接机器狗失败！");
            rclcpp::shutdown();
            return;
        }

        // 3. 创建服务
        motion_service_ = this->create_service<MotionControlSrv>(
            "/agibot/motion_control",
            std::bind(&MotionServerNode::motion_control_callback, this, _1, _2)
        );

        state_service_ = this->create_service<GetStateSrv>(
            "/agibot/get_state",
            std::bind(&MotionServerNode::get_state_callback, this, _1, _2)
        );

        // 4. 创建发布者
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/agibot/velocity", 50);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/agibot/imu", 100);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/agibot/odom", 50);
        // 创建状态发布者
        state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/agibot/current_state", 50);

        // 5. 创建 cmd_vel 订阅者（用于导航系统）
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&MotionServerNode::cmd_vel_callback, this, std::placeholders::_1)
        );

        // 6. 创建定时器
        int period_ms = static_cast<int>(1000.0 / state_rate);
        state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&MotionServerNode::state_publish_callback, this));

        watchdog_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&MotionServerNode::watchdog_callback, this));

        relative_motion_timer_ = this->create_wall_timer(
            50ms,
            std::bind(&MotionServerNode::relative_motion_timer_callback, this));

        // 6. 初始化状态
        current_state_ = "lying_down";
        current_control_source_ = "none";
        last_cmd_time_ = this->get_clock()->now();
        // // 7. 启动时自动站立
        // try {
        //     RCLCPP_INFO(this->get_logger(), "启动时执行自动站立动作...");
        //     sdk_ptr_->standUp();
        //     current_state_ = "standing";
        //     RCLCPP_INFO(this->get_logger(), "自动站立成功！");
        // } catch (const std::exception& e) {
        //     RCLCPP_WARN(this->get_logger(), "自动站立失败：%s", e.what());
        // }

        RCLCPP_INFO(this->get_logger(), "运动控制服务已启动");
    }

    ~MotionServerNode()
    {
        // 节点关闭时先趴下，再进入被动模式
        try {
            RCLCPP_INFO(this->get_logger(), "节点关闭，执行安全状态转换...");
            if (sdk_ptr_ && sdk_ptr_->checkConnect()) {
                // 先趴下
                RCLCPP_INFO(this->get_logger(), "执行趴下动作...");
                sdk_ptr_->lieDown();
                // 等待趴下完成
                std::this_thread::sleep_for(1s);
                // 再进入被动模式
                RCLCPP_INFO(this->get_logger(), "执行被动模式...");
                sdk_ptr_->passive();
                RCLCPP_INFO(this->get_logger(), "已进入安全状态（趴下+被动模式）");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "安全状态转换失败：%s", e.what());
        }
    }

private:
    // 控制仲裁
    bool is_control_allowed(const std::string& source)
    {
        // 紧急控制总是允许
        if (source == "emergency") return true;

        // 相同源总是允许
        if (source == current_control_source_) return true;

        // 检查控制超时
        auto now = this->get_clock()->now();
        auto time_since_last = now - last_cmd_time_;
        if (time_since_last > 2s) {
            return true;
        }

        // 优先级检查：voice > navigation
        if (source == "voice" && current_control_source_ == "navigation") {
            return true;
        }

        return false;
    }

    void cancel_relative_motion(bool send_stop)
    {
        if (!relative_motion_active_) {
            return;
        }

        relative_motion_active_ = false;
        if (send_stop) {
            try {
                sdk_ptr_->move(0.0, 0.0, 0.0);
                current_state_ = "standing";
                last_cmd_time_ = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "已停止上一条语音相对运动");
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "停止上一条语音相对运动失败：%s", e.what());
            }
        }
    }

    bool start_timed_motion(
        float vx,
        float vy,
        float yaw_rate,
        double duration,
        const std::string& source,
        std::string& message)
    {
        if (!(current_state_ == "standing" || current_state_ == "moving")) {
            message = "移动失败：需要先站立";
            return false;
        }

        if (duration <= 0.0) {
            message = "移动失败：duration 必须大于 0";
            return false;
        }

        cancel_relative_motion(true);

        const auto now = this->get_clock()->now();
        const auto duration_ns = static_cast<int64_t>(duration * 1000000000.0);
        relative_motion_vx_ = vx;
        relative_motion_vy_ = vy;
        relative_motion_yaw_rate_ = yaw_rate;
        relative_motion_end_time_ = now + rclcpp::Duration::from_nanoseconds(duration_ns);
        relative_motion_active_ = true;
        current_control_source_ = source;
        last_cmd_time_ = now;
        current_state_ = "moving";

        message = "定时移动已开始";
        return true;
    }

    bool start_voice_relative_motion(
        const std::shared_ptr<MotionControlSrv::Request> req,
        std::string& message)
    {
        const std::string& action = req->action_name;
        const bool is_linear = (action == "forward" || action == "backward");
        const bool is_turn = (action == "turn");
        if (!is_linear && !is_turn) {
            message = "未知相对运动：" + action;
            return false;
        }

        if (req->control_value < 0.0f) {
            message = "相对运动参数必须大于 0";
            return false;
        }

        const double value = (req->control_value == 0.0f) ? 1.0 : static_cast<double>(req->control_value);
        const std::string expected_unit = is_linear ? "m" : "circle";
        const std::string unit = req->control_unit.empty() ? expected_unit : req->control_unit;
        if (unit != expected_unit) {
            message = action + " 仅支持单位 " + expected_unit;
            return false;
        }

        if (is_linear) {
            double linear_speed = std::fabs(this->get_parameter("voice_linear_speed_mps").as_double());
            if (linear_speed < 0.01) {
                message = "voice_linear_speed_mps 参数过小，无法执行";
                return false;
            }

            const float vx = static_cast<float>((action == "forward" ? 1.0 : -1.0) * linear_speed);
            const double duration = value / linear_speed;
            const bool ok = start_timed_motion(vx, 0.0f, 0.0f, duration, req->control_source, message);
            if (ok) {
                message = action + " 已开始，距离=" + std::to_string(value) +
                          "m，速度=" + std::to_string(linear_speed) + "m/s";
            }
            return ok;
        }

        double yaw_rate = std::fabs(this->get_parameter("voice_turn_yaw_rate_radps").as_double());
        if (yaw_rate < 0.01) {
            message = "voice_turn_yaw_rate_radps 参数过小，无法执行";
            return false;
        }

        constexpr double kTwoPi = 6.283185307179586;
        const double duration = value * kTwoPi / yaw_rate;
        const bool ok = start_timed_motion(0.0f, 0.0f, static_cast<float>(yaw_rate), duration, req->control_source, message);
        if (ok) {
            message = "turn 已开始，圈数=" + std::to_string(value) +
                      "，角速度=" + std::to_string(yaw_rate) + "rad/s";
        }
        return ok;
    }

    // 运动控制回调
    void motion_control_callback(
        const std::shared_ptr<MotionControlSrv::Request> req,
        const std::shared_ptr<MotionControlSrv::Response> res)
    {
        // 检查控制权限
        if (!is_control_allowed(req->control_source)) {
            res->success = false;
            res->message = "控制权限被拒绝：当前控制源正在活动";
            res->current_state = current_state_;
            return;
        }

        // 更新控制源和时间
        current_control_source_ = req->control_source;
        last_cmd_time_ = this->get_clock()->now();

        // 执行控制指令
        bool success = false;
        std::string message = "";

        try {
            const bool is_relative_action =
                (req->action_name == "forward" || req->action_name == "backward" || req->action_name == "turn");
            if (!is_relative_action && req->action_name != "move") {
                cancel_relative_motion(true);
            }

            // 根据动作名称执行相应操作
            if (is_relative_action) {
                success = start_voice_relative_motion(req, message);
            } else if (req->action_name == "move") {
                // 处理移动指令
                if(current_state_ == "standing" || current_state_ == "moving"){
                    if (req->duration > 0.0f) {
                        success = start_timed_motion(
                            req->vx,
                            req->vy,
                            req->yaw_rate,
                            static_cast<double>(req->duration),
                            req->control_source,
                            message);
                    } else {
                        cancel_relative_motion(false);
                        sdk_ptr_->move(req->vx, req->vy, req->yaw_rate);
                        success = true;
                        message = "移动指令执行成功";
                        if (std::fabs(req->vx) < 1e-5f && std::fabs(req->vy) < 1e-5f && std::fabs(req->yaw_rate) < 1e-5f) {
                            current_state_ = "standing";
                        } else {
                            current_state_ = "moving";
                        }
                    }
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "请先站立或已经在执行中");
                }
            } else if (req->action_name == "stand_up") {
                // 处理站立指令
                sdk_ptr_->standUp();
                success = true;
                message = "站立成功";
                current_state_ = "standing";
            } else if (req->action_name == "lie_down") {
                // 处理趴下指令
                sdk_ptr_->lieDown();
                success = true;
                message = "趴下成功";
                current_state_ = "lying_down";
            } else if (req->action_name == "jump") {
                // 处理跳跃指令
                if (current_state_ == "standing" || current_state_ == "standing_stable") {
                    sdk_ptr_->jump();
                    success = true;
                    message = "跳跃成功";
                } else {
                    success = false;
                    message = "跳跃失败：需要先站立";
                }
            } else if (req->action_name == "passive") {
                // 处理被动模式
                sdk_ptr_->passive();
                success = true;
                message = "已进入被动模式";
                current_state_ = "passive";
            } else if (req->action_name == "shake_hand") {
                // 处理握手指令
                if (current_state_ == "standing" || current_state_ == "standing_stable") {
                    try {
                        // 直接执行握手动作，参考agibot_voice_control的实现
                        // 执行握手前设置状态为shaking_hand
                        current_state_ = "shaking_hand";
                        
                        // 执行握手动作
                        uint32_t result = sdk_ptr_->shakeHand();
                        if (result == 0) {
                            // 添加延时确保握手动作完整执行
                            std::this_thread::sleep_for(3s);  // 等待3秒让握手动作完成
                            last_cmd_time_ = this->get_clock()->now();  // 更新最后指令时间，防止看门狗中断
                            success = true;
                            message = "握手成功";
                            
                            // 延时后恢复状态为站立
                            current_state_ = "standing";
                            RCLCPP_INFO(this->get_logger(), "握手动作完成，状态恢复为站立");
                        } else {
                            success = false;
                            // 根据错误码设置具体错误信息
                            switch (result) {
                                case 0x3012:
                                    message = "握手失败：电机数据丢失";
                                    break;
                                case 0x3010:
                                    message = "握手失败：电机失能";
                                    break;
                                case 0x3011:
                                    message = "握手失败：电机故障";
                                    break;
                                case 0x3009:
                                    message = "握手失败：电机角度超限";
                                    break;
                                case 0x3007:
                                    message = "握手失败：状态机切换失败";
                                    break;
                                case 0x3013:
                                    message = "握手失败：速度命令过大";
                                    break;
                                default:
                                    message = "握手失败：未知错误码 " + std::to_string(result);
                                    break;
                            }
                            RCLCPP_ERROR(this->get_logger(), "握手动作执行失败，错误码: %#x", result);
                        }
                    } catch (const std::exception& e) {
                        success = false;
                        message = "握手失败：" + std::string(e.what());
                    }
                } else {
                    success = false;
                    message = "握手失败：需要先站立";
                }
            } else if (req->action_name == "front_jump") {
                // 处理前跳指令
                if (current_state_ == "standing" || current_state_ == "standing_stable") {
                    sdk_ptr_->frontJump();
                    success = true;
                    message = "前跳成功";
                } else {
                    success = false;
                    message = "前跳失败：需要先站立";
                }
            } else {
                success = false;
                message = "未知动作：" + req->action_name;
            }
        } catch (const std::exception& e) {
            success = false;
            message = "执行失败：" + std::string(e.what());
        }

        // 设置响应
        res->success = success;
        res->message = message;
        res->current_state = current_state_;

        RCLCPP_INFO(this->get_logger(), "控制指令执行：%s，结果：%s", 
                   req->action_name.c_str(), message.c_str());
    }

    // 状态查询回调
    void get_state_callback(
        [[maybe_unused]] const std::shared_ptr<GetStateSrv::Request> req,
        const std::shared_ptr<GetStateSrv::Response> res)
    {
        res->current_state = current_state_;
        res->battery_level = static_cast<float>(sdk_ptr_->getBatteryPower());  // 获取电池电量
        res->is_ready = sdk_ptr_->checkConnect();
        res->control_source = current_control_source_;
    }

    // 状态发布回调
    void state_publish_callback()
    {
        try {
            rclcpp::Time now = this->get_clock()->now();
            
            // 获取状态数据
            std::vector<float> body_vel = sdk_ptr_->getBodyVelocity();
            std::vector<float> body_gyro = sdk_ptr_->getBodyGyro();
            std::vector<float> body_acc = sdk_ptr_->getBodyAcc();
            std::vector<float> quat = sdk_ptr_->getQuaternion();
            std::vector<float> position = sdk_ptr_->getPosition();
            
            // 发布速度
            if (body_vel.size() >= 3 && body_gyro.size() >= 3) {
                geometry_msgs::msg::TwistStamped vel_msg;
                vel_msg.header.stamp = now;
                vel_msg.header.frame_id = "base_link";
                vel_msg.twist.linear.x = static_cast<double>(body_vel[0]);
                vel_msg.twist.linear.y = static_cast<double>(body_vel[1]);
                vel_msg.twist.linear.z = static_cast<double>(body_vel[2]);
                vel_msg.twist.angular.x = static_cast<double>(body_gyro[0]);
                vel_msg.twist.angular.y = static_cast<double>(body_gyro[1]);
                vel_msg.twist.angular.z = static_cast<double>(body_gyro[2]);
                velocity_pub_->publish(vel_msg);
            }
            
            // 发布IMU数据
            if (body_acc.size() >= 3 && body_gyro.size() >= 3 && quat.size() >= 4) {
                sensor_msgs::msg::Imu imu_msg;
                imu_msg.header.stamp = now;
                imu_msg.header.frame_id = "base_link";
                imu_msg.orientation.w = static_cast<double>(quat[0]);
                imu_msg.orientation.x = static_cast<double>(quat[1]);
                imu_msg.orientation.y = static_cast<double>(quat[2]);
                imu_msg.orientation.z = static_cast<double>(quat[3]);
                imu_msg.angular_velocity.x = static_cast<double>(body_gyro[0]);
                imu_msg.angular_velocity.y = static_cast<double>(body_gyro[1]);
                imu_msg.angular_velocity.z = static_cast<double>(body_gyro[2]);
                imu_msg.linear_acceleration.x = static_cast<double>(body_acc[0]);
                imu_msg.linear_acceleration.y = static_cast<double>(body_acc[1]);
                imu_msg.linear_acceleration.z = static_cast<double>(body_acc[2]);
                imu_pub_->publish(imu_msg);
            }
            
            // 发布里程计
            if (position.size() >= 3 && quat.size() >= 4) {
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = now;
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id = "base_link";
                odom_msg.pose.pose.position.x = static_cast<double>(position[0]);
                odom_msg.pose.pose.position.y = static_cast<double>(position[1]);
                odom_msg.pose.pose.position.z = static_cast<double>(position[2]);
                odom_msg.pose.pose.orientation.w = static_cast<double>(quat[0]);
                odom_msg.pose.pose.orientation.x = static_cast<double>(quat[1]);
                odom_msg.pose.pose.orientation.y = static_cast<double>(quat[2]);
                odom_msg.pose.pose.orientation.z = static_cast<double>(quat[3]);
                odom_pub_->publish(odom_msg);
            }
            
            // 发布机器狗状态
            std_msgs::msg::String state_msg;
            state_msg.data = current_state_;
            state_pub_->publish(state_msg);
            
            // 状态恢复逻辑：如果当前是握手状态，检查是否需要恢复到站立状态
            if (current_state_ == "shaking_hand") {
                // 握手动作完成后，恢复到站立状态
                // 这里可以根据实际情况添加延迟或条件判断
                current_state_ = "standing";
                RCLCPP_INFO(this->get_logger(), "握手动作完成，状态恢复为站立");
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "状态发布失败：%s", e.what());
        }
    }

    void relative_motion_timer_callback()
    {
        if (!relative_motion_active_) {
            return;
        }

        try {
            const auto now = this->get_clock()->now();
            if (now >= relative_motion_end_time_) {
                sdk_ptr_->move(0.0, 0.0, 0.0);
                relative_motion_active_ = false;
                current_state_ = "standing";
                last_cmd_time_ = now;
                RCLCPP_INFO(this->get_logger(), "语音相对运动完成，已停止");
                return;
            }

            sdk_ptr_->move(relative_motion_vx_, relative_motion_vy_, relative_motion_yaw_rate_);
            current_state_ = "moving";
            last_cmd_time_ = now;
        } catch (const std::exception& e) {
            relative_motion_active_ = false;
            current_state_ = "standing";
            RCLCPP_WARN(this->get_logger(), "语音相对运动执行失败：%s", e.what());
        }
    }

    // 看门狗回调
    void watchdog_callback()
    {
        auto now = this->get_clock()->now();
        auto time_since_last = now - last_cmd_time_;
        
        // 超时检查：cmd_vel 频率应该 10~20Hz，间隔 50~100ms
        // 1s 超时太长，狗会按最后速度滑行 0.6m+
        // 改成 300ms：goal 达成/controller 卡顿时狗能及时停下
        if (current_state_ == "moving") {
            if (time_since_last > 300ms) {
                try {
                    sdk_ptr_->move(0.0, 0.0, 0.0);
                    current_state_ = "standing";
                    RCLCPP_INFO(this->get_logger(), "moving结束，状态更新为站立");
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), "看门狗停止失败：%s", e.what());
                }
            }
        }
    }

    // cmd_vel 回调函数（用于导航系统）
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 检查控制权限
        if (!is_control_allowed("navigation")) {
            RCLCPP_INFO(this->get_logger(), "导航控制被拒绝，当前控制权属于: %s", 
                        current_control_source_.c_str());
            return;
        }

        cancel_relative_motion(true);

        // 执行移动指令
        float vx = static_cast<float>(msg->linear.x);
        float vy = static_cast<float>(msg->linear.y);
        const float yaw_rate = static_cast<float>(msg->angular.z);

        // 线速度「最小可执行」与死区对齐为 0.10：略低于 0.10 的爬行指令视为无效。
        constexpr float k_lin_dead = 0.10f;
        constexpr float k_lin_min_exec = 0.10f;
        constexpr float k_yaw_eps_for_lin_boost = 0.099f;

        // 末端直线爬行：vy、ω 都很小时，若有非零 vx 却低于线死区 → 抬到 k_lin_min_exec（仍保留符号）
        if (std::fabs(vy) < k_lin_dead && std::fabs(yaw_rate) < k_yaw_eps_for_lin_boost) {
            if (std::fabs(vx) > 1e-5f && std::fabs(vx) < k_lin_dead) {
                vx = std::copysign(std::max(std::fabs(vx), k_lin_min_exec), vx);
            }
        }

        const bool lin_dead =
            (std::fabs(vx) < k_lin_dead && std::fabs(vy) < k_lin_dead);

        if (lin_dead) {
            vx = 0.0f;
            vy = 0.0f;
        } else {
            if (std::fabs(vx) < k_lin_dead) vx = 0.0f;
            if (std::fabs(vy) < k_lin_dead) vy = 0.0f;
        }

        // angular.z 原样下发（/cmd_vel 若为全零须查 Nav2 controller/smoother，勿在此处「修补」）

        // 执行移动
        try {
            sdk_ptr_->move(vx, vy, yaw_rate);
            current_control_source_ = "navigation";
            last_cmd_time_ = this->get_clock()->now();
            current_state_ = "moving";
            
            RCLCPP_INFO(this->get_logger(), "执行导航移动指令: vx=%.2f, vy=%.2f, yaw_rate=%.2f", 
                        vx, vy, yaw_rate);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "执行导航移动指令失败：%s", e.what());
        }
    }

    // 成员变量
    std::unique_ptr<HighLevel> sdk_ptr_;
    rclcpp::Service<MotionControlSrv>::SharedPtr motion_service_;
    rclcpp::Service<GetStateSrv>::SharedPtr state_service_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr relative_motion_timer_;
    std::string current_state_;
    std::string current_control_source_;
    rclcpp::Time last_cmd_time_;
    bool relative_motion_active_{false};
    rclcpp::Time relative_motion_end_time_;
    float relative_motion_vx_{0.0f};
    float relative_motion_vy_{0.0f};
    float relative_motion_yaw_rate_{0.0f};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
