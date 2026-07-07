from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    local_ip = LaunchConfiguration("local_ip")
    dog_ip = LaunchConfiguration("dog_ip")
    local_port = LaunchConfiguration("local_port")

    # Historical RoboSense/YESENSE mounting values from the old navigation stack.
    # Keep this launch as a reference only; the current stack uses agibot_motion_service.
    lidar_xyz_rpy = ["0.08", "0.0", "0.174", "0.0", "0.0", "0.0"]
    imu_xyz_rpy = ["0.0", "0.0", "0.05335", "0.0", "0.0", "0.0"]

    return LaunchDescription(
        [
            DeclareLaunchArgument("local_ip", default_value="192.168.168.1"),
            DeclareLaunchArgument("dog_ip", default_value="192.168.168.168"),
            DeclareLaunchArgument("local_port", default_value="43988"),
            Node(
                package="legacy_agibot_driver",
                executable="agibot_bridge_node",
                name="legacy_agibot_bridge",
                output="screen",
                parameters=[
                    {
                        "local_ip": local_ip,
                        "dog_ip": dog_ip,
                        "local_port": local_port,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="legacy_static_tf_pub_lidar",
                arguments=[*lidar_xyz_rpy, "base_link", "rslidar"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="legacy_static_tf_pub_imu",
                arguments=[*imu_xyz_rpy, "base_link", "imu_link"],
            ),
        ]
    )
