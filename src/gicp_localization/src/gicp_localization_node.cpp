#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/ros/ros2.hpp>

class GicpLocalizationNode : public rclcpp::Node {
public:
    GicpLocalizationNode() : Node("gicp_localization")
    {
        downsample_resolution_ = declare_parameter("downsample_resolution", 0.2);
        num_threads_ = declare_parameter("num_threads", 4);
        map_path_ = declare_parameter("map_path", "");
        input_cloud_topic_ = declare_parameter("input_cloud_topic", "/cloud_registered_body");

        map_frame_ = declare_parameter("map_frame", "map");
        odom_frame_ = declare_parameter("odom_frame", "odom");
        base_frame_ = declare_parameter("base_frame", "base_link");
        publish_tf_ = declare_parameter("publish_tf", true);
        tf_lookup_timeout_sec_ = declare_parameter("tf_lookup_timeout", 0.2);
        use_latest_tf_fallback_ = declare_parameter("use_latest_tf_fallback", false);

        frame_skip_ = declare_parameter("frame_skip", 5);
        max_correspondence_distance_ = declare_parameter("max_correspondence_distance", 1.5);
        max_tracking_translation_jump_ =
            declare_parameter("max_tracking_translation_jump", 0.35);
        max_tracking_yaw_jump_deg_ =
            declare_parameter("max_tracking_yaw_jump_deg", 8.0);
        max_consecutive_failures_ = declare_parameter("max_consecutive_failures", 12);
        tf_publish_rate_hz_ = declare_parameter("tf_publish_rate_hz", 20.0);

        publish_global_map_ = declare_parameter("publish_global_map", true);
        global_map_topic_ = declare_parameter("global_map_topic", "/gicp/global_map");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        pose_pub_ =
            create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/gicp_pose", 10);
        global_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            global_map_topic_, rclcpp::QoS(1).reliable().transient_local());

        initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&GicpLocalizationNode::initialPoseCallback, this, std::placeholders::_1));

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_topic_, 10,
            std::bind(&GicpLocalizationNode::cloudCallback, this, std::placeholders::_1));

        const double tf_rate = std::max(1.0, tf_publish_rate_hz_);
        const auto tf_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / tf_rate));
        tf_timer_ = create_wall_timer(
            tf_period, std::bind(&GicpLocalizationNode::publishCachedMapOdom, this));

        loadGlobalMap(map_path_);
    }

private:
    double downsample_resolution_;
    int num_threads_;
    std::string map_path_;
    std::string input_cloud_topic_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;
    double tf_lookup_timeout_sec_;
    bool use_latest_tf_fallback_;
    int frame_skip_;
    double max_correspondence_distance_;
    double max_tracking_translation_jump_;
    double max_tracking_yaw_jump_deg_;
    int max_consecutive_failures_;
    double tf_publish_rate_hz_;
    bool publish_global_map_;
    std::string global_map_topic_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<small_gicp::PointCloud> target_map_;
    std::shared_ptr<small_gicp::KdTree<small_gicp::PointCloud>> target_tree_;

    Eigen::Isometry3d initial_pose_guess_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d current_map_base_ = Eigen::Isometry3d::Identity();
    bool has_initial_pose_ = false;
    bool tracking_initialized_ = false;
    int frame_counter_ = 0;
    int consecutive_failures_ = 0;

    bool has_last_map_odom_ = false;
    geometry_msgs::msg::TransformStamped last_map_odom_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    static Eigen::Isometry3d transformMsgToIsometry(const geometry_msgs::msg::Transform & msg)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translation() << msg.translation.x, msg.translation.y, msg.translation.z;
        Eigen::Quaterniond q(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
        transform.linear() = q.normalized().toRotationMatrix();
        return transform;
    }

    static geometry_msgs::msg::Transform isometryToTransformMsg(const Eigen::Isometry3d & transform)
    {
        geometry_msgs::msg::Transform msg;
        msg.translation.x = transform.translation().x();
        msg.translation.y = transform.translation().y();
        msg.translation.z = transform.translation().z();
        const Eigen::Quaterniond q(transform.rotation());
        msg.rotation.x = q.x();
        msg.rotation.y = q.y();
        msg.rotation.z = q.z();
        msg.rotation.w = q.w();
        return msg;
    }

    static double yawFromIsometry(const Eigen::Isometry3d & transform)
    {
        const Eigen::Matrix3d & rotation = transform.rotation();
        return std::atan2(rotation(1, 0), rotation(0, 0));
    }

    bool lookupOdomToBase(const rclcpp::Time & stamp, Eigen::Isometry3d & odom_base)
    {
        const auto timeout = rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_);

        try {
            geometry_msgs::msg::TransformStamped tf_msg;
            try {
                tf_msg = tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp, timeout);
            } catch (const tf2::TransformException &) {
                if (!use_latest_tf_fallback_) {
                    throw;
                }
                const rclcpp::Time latest_time(0, 0, get_clock()->get_clock_type());
                tf_msg = tf_buffer_->lookupTransform(odom_frame_, base_frame_, latest_time, timeout);
            }
            odom_base = transformMsgToIsometry(tf_msg.transform);
            return true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skip GICP TF update: cannot lookup %s -> %s: %s",
                odom_frame_.c_str(), base_frame_.c_str(), ex.what());
            return false;
        }
    }

    bool lookupCloudToBase(
        const sensor_msgs::msg::PointCloud2 & msg,
        const rclcpp::Time & stamp,
        Eigen::Isometry3d & base_cloud)
    {
        if (msg.header.frame_id.empty() || msg.header.frame_id == base_frame_) {
            base_cloud = Eigen::Isometry3d::Identity();
            return true;
        }

        const auto timeout = rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_);

        try {
            geometry_msgs::msg::TransformStamped tf_msg;
            try {
                tf_msg = tf_buffer_->lookupTransform(
                    base_frame_, msg.header.frame_id, stamp, timeout);
            } catch (const tf2::TransformException &) {
                if (!use_latest_tf_fallback_) {
                    throw;
                }
                const rclcpp::Time latest_time(0, 0, get_clock()->get_clock_type());
                tf_msg = tf_buffer_->lookupTransform(
                    base_frame_, msg.header.frame_id, latest_time, timeout);
            }
            base_cloud = transformMsgToIsometry(tf_msg.transform);
            return true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skip GICP cloud: cannot transform %s -> %s: %s",
                msg.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
            return false;
        }
    }

    bool predictedMapBase(const rclcpp::Time & stamp, Eigen::Isometry3d & map_base)
    {
        if (!has_last_map_odom_) {
            map_base = current_map_base_;
            return true;
        }

        Eigen::Isometry3d odom_base;
        if (!lookupOdomToBase(stamp, odom_base)) {
            return false;
        }

        const Eigen::Isometry3d map_odom = transformMsgToIsometry(last_map_odom_.transform);
        map_base = map_odom * odom_base;
        return true;
    }

    bool publishMapOdomFromPose(const Eigen::Isometry3d & map_base, const rclcpp::Time & stamp)
    {
        if (!publish_tf_) {
            return true;
        }

        Eigen::Isometry3d odom_base;
        if (!lookupOdomToBase(stamp, odom_base)) {
            return false;
        }

        const Eigen::Isometry3d map_odom = map_base * odom_base.inverse();
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = map_frame_;
        tf_msg.child_frame_id = odom_frame_;
        tf_msg.transform = isometryToTransformMsg(map_odom);
        tf_broadcaster_->sendTransform(tf_msg);

        last_map_odom_ = tf_msg;
        has_last_map_odom_ = true;
        return true;
    }

    void publishCachedMapOdom()
    {
        if (!publish_tf_ || !has_last_map_odom_) {
            return;
        }

        auto tf_msg = last_map_odom_;
        tf_msg.header.stamp = now();
        tf_broadcaster_->sendTransform(tf_msg);
    }

    void loadGlobalMap(const std::string & path)
    {
        RCLCPP_INFO(get_logger(), "Loading GICP global map: %s", path.c_str());

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *pcl_map) == -1) {
            RCLCPP_ERROR(get_logger(), "Failed to load PCD map: %s", path.c_str());
            return;
        }

        if (publish_global_map_) {
            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*pcl_map, map_msg);
            map_msg.header.stamp = now();
            map_msg.header.frame_id = map_frame_;
            global_map_pub_->publish(map_msg);
        }

        std::vector<Eigen::Vector3d> points;
        points.reserve(pcl_map->size());
        for (const auto & point : *pcl_map) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                points.emplace_back(point.x, point.y, point.z);
            }
        }

        auto [map_cloud, map_tree] =
            small_gicp::preprocess_points(points, downsample_resolution_, 10, num_threads_);
        target_map_ = map_cloud;
        target_tree_ = map_tree;

        RCLCPP_INFO(
            get_logger(),
            "GICP map ready. Waiting for RViz /initialpose before publishing map->odom.");
    }

    std::shared_ptr<small_gicp::PointCloud> preprocessCloud(
        const sensor_msgs::msg::PointCloud2 & msg,
        const Eigen::Isometry3d & base_cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(msg, *pcl_cloud);

        std::vector<Eigen::Vector3d> points;
        points.reserve(pcl_cloud->size());
        for (const auto & point : *pcl_cloud) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                const Eigen::Vector3d p_base =
                    base_cloud * Eigen::Vector3d(point.x, point.y, point.z);
                points.emplace_back(p_base);
            }
        }

        auto [cloud, tree] =
            small_gicp::preprocess_points(points, downsample_resolution_, 10, num_threads_);
        (void)tree;
        return cloud;
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        tf2::fromMsg(msg->pose.pose, initial_pose_guess_);
        current_map_base_ = initial_pose_guess_;
        has_initial_pose_ = true;
        tracking_initialized_ = false;
        has_last_map_odom_ = false;
        consecutive_failures_ = 0;

        RCLCPP_INFO(
            get_logger(),
            "Received initial pose. Next GICP convergence will initialize map->odom.");
    }

    void handleFailure(const std::string & reason)
    {
        consecutive_failures_++;
        const int fail_limit = std::max(max_consecutive_failures_, 1);

        if (consecutive_failures_ >= fail_limit) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "GICP failed %d/%d frames. Holding localization invalid. Reason: %s",
                consecutive_failures_, fail_limit, reason.c_str());
            return;
        }

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1500,
            "GICP temporary failure %d/%d: %s",
            consecutive_failures_, fail_limit, reason.c_str());
    }

    void publishPose(
        const Eigen::Isometry3d & map_base,
        const Eigen::Matrix<double, 6, 6> & hessian,
        const rclcpp::Time & stamp)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = map_frame_;
        pose_msg.pose.pose = tf2::toMsg(map_base);

        const Eigen::Matrix<double, 6, 6> cov = hessian.inverse();
        for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 6; ++col) {
                pose_msg.pose.covariance[row * 6 + col] = cov(row, col);
            }
        }
        pose_pub_->publish(pose_msg);
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!has_initial_pose_ || !target_map_) {
            return;
        }

        frame_counter_++;
        if (frame_counter_ % std::max(frame_skip_, 1) != 0) {
            return;
        }

        const rclcpp::Time stamp(msg->header.stamp);
        Eigen::Isometry3d base_cloud;
        if (!lookupCloudToBase(*msg, stamp, base_cloud)) {
            handleFailure("cannot transform input cloud into base frame");
            return;
        }

        const auto source_cloud = preprocessCloud(*msg, base_cloud);

        small_gicp::RegistrationSetting setting;
        setting.num_threads = num_threads_;
        setting.max_correspondence_distance = max_correspondence_distance_;

        Eigen::Isometry3d seed;
        if (tracking_initialized_) {
            if (!predictedMapBase(stamp, seed)) {
                handleFailure("cannot generate odom predicted seed");
                return;
            }
        } else {
            seed = initial_pose_guess_;
        }

        auto result =
            small_gicp::align(*target_map_, *source_cloud, *target_tree_, seed, setting);

        if (!result.converged) {
            handleFailure("registration did not converge");
            return;
        }

        const Eigen::Isometry3d candidate = result.T_target_source;
        if (tracking_initialized_) {
            const Eigen::Isometry3d delta = seed.inverse() * candidate;
            const double dtrans = delta.translation().norm();
            const double dyaw_deg =
                std::abs(yawFromIsometry(delta)) * 180.0 / 3.14159265358979323846;

            if (dtrans > max_tracking_translation_jump_ ||
                dyaw_deg > max_tracking_yaw_jump_deg_) {
                handleFailure(
                    "tracking jump rejected: dtrans=" + std::to_string(dtrans) +
                    "m, dyaw=" + std::to_string(dyaw_deg) + "deg");
                return;
            }
        }

        if (!publishMapOdomFromPose(candidate, stamp)) {
            handleFailure("cannot publish map->odom because odom TF lookup failed");
            return;
        }

        current_map_base_ = candidate;
        tracking_initialized_ = true;
        consecutive_failures_ = 0;
        publishPose(candidate, result.H, stamp);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GicpLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
