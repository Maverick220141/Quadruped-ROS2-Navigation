import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    my_nav_dir = get_package_share_directory("my_nav")
    fast_lio_dir = get_package_share_directory("fast_lio")
    livox_dir = get_package_share_directory("livox_ros_driver2")
    scan_dir = get_package_share_directory("lidar_scan_bridge")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_map = LaunchConfiguration("map")
    pcd_map = LaunchConfiguration("pcd_map")
    nav2_params = LaunchConfiguration("nav2_params")
    fast_lio_config = LaunchConfiguration("fast_lio_config")

    start_livox = LaunchConfiguration("start_livox")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    start_scan = LaunchConfiguration("start_scan")
    start_gicp = LaunchConfiguration("start_gicp")
    start_nav2 = LaunchConfiguration("start_nav2")
    use_rviz = LaunchConfiguration("use_rviz")

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_dir, "launch_ROS2", "msg_MID360_launch.py")
        ),
        condition=IfCondition(start_livox),
    )

    base_link_to_livox_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_livox",
        arguments=[
            "0.0",
            "0.0",
            "0.10",
            "0.0",
            "0.0",
            "0.0",
            "1.0",
            "base_link",
            "livox_frame",
        ],
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, "launch", "mapping.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "config_file": fast_lio_config,
            "rviz": "false",
        }.items(),
        condition=IfCondition(start_fast_lio),
    )

    scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scan_dir, "launch", "scan_bridge.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(start_scan),
    )

    gicp_node = Node(
        package="gicp_localization",
        executable="gicp_localization_node",
        name="gicp_localization",
        output="screen",
        parameters=[
            {
                "map_path": pcd_map,
                "input_cloud_topic": "/cloud_registered_body",
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "frame_skip": 5,
                "downsample_resolution": 0.2,
                "max_correspondence_distance": 1.5,
                "max_tracking_translation_jump": 0.35,
                "max_tracking_yaw_jump_deg": 8.0,
                "max_consecutive_failures": 12,
                "tf_publish_rate_hz": 20.0,
                "use_latest_tf_fallback": True,
            }
        ],
        condition=IfCondition(start_gicp),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_nav_dir, "launch", "nav2_bringup_gicp.launch.py")
        ),
        launch_arguments={
            "map": nav2_map,
            "params_file": nav2_params,
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
        }.items(),
        condition=IfCondition(start_nav2),
    )

    scan_virtual_obstacle_node = Node(
        package="my_nav",
        executable="scan_virtual_obstacle_node",
        name="scan_virtual_obstacle",
        output="screen",
        parameters=[
            {
                "scan_topic": "/scan",
                "output_topic": "/blocked_path/virtual_obstacles",
                "map_frame": "map",
                "min_range": 0.30,
                "max_range": 3.5,
                "grid_resolution": 0.70,
                "obstacle_ttl_sec": 5.0,
                "publish_period_ms": 200,
                "max_obstacles": 120,
                "min_samples_per_obstacle": 3,
                "use_latest_tf_on_failure": True,
            }
        ],
        condition=IfCondition(start_nav2),
    )

    delayed_fast_lio = TimerAction(period=2.0, actions=[fast_lio_launch])
    delayed_scan = TimerAction(period=4.0, actions=[scan_launch])
    delayed_gicp = TimerAction(period=6.0, actions=[gicp_node])
    delayed_nav2 = TimerAction(period=8.0, actions=[nav2_launch])
    delayed_scan_virtual_obstacle = TimerAction(period=10.0, actions=[scan_virtual_obstacle_node])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_fast_lio", default_value="true"),
            DeclareLaunchArgument("start_scan", default_value="true"),
            DeclareLaunchArgument("start_gicp", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("fast_lio_config", default_value="mid360.yaml"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(my_nav_dir, "maps", "map1.yaml"),
                description="2D Nav2 map YAML",
            ),
            DeclareLaunchArgument(
                "pcd_map",
                default_value="",
                description="3D PCD map used by GICP localization",
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=os.path.join(my_nav_dir, "config", "nav2_params_gicp.yaml"),
                description="Nav2 params for external GICP map->odom localization",
            ),
            livox_launch,
            base_link_to_livox_tf,
            delayed_fast_lio,
            delayed_scan,
            delayed_gicp,
            delayed_nav2,
            delayed_scan_virtual_obstacle,
        ]
    )
