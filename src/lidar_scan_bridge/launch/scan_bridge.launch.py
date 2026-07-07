from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("lidar_scan_bridge")
    pcl2scan_yaml = os.path.join(pkg_share, "config", "pointcloud_to_laserscan.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan_node",
                output="screen",
                parameters=[
                    pcl2scan_yaml,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"target_frame": "base_link"},
                    {"output_frame": "base_link"},
                ],
                remappings=[
                    ("/cloud_in", "/cloud_registered_body"),
                    ("/scan", "/scan_raw"),
                ],
            ),
            Node(
                package="lidar_scan_bridge",
                executable="fix_scan_stamp",
                name="fix_scan_stamp",
                output="screen",
                parameters=[
                    {"input_scan": "/scan_raw"},
                    {"output_scan": "/scan"},
                    {"output_frame": "base_link"},
                    {"stamp_offset_sec": -0.08},
                ],
            ),
        ]
    )
