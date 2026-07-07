import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("quadruped_navigation_bringup")
    my_nav_dir = get_package_share_directory("my_nav")
    motion_dir = get_package_share_directory("agibot_motion_service")

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_motion_service = LaunchConfiguration("start_motion_service")

    nav_sensors_and_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_nav_dir, "launch", "gicp_navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_livox": LaunchConfiguration("start_livox"),
            "start_fast_lio": LaunchConfiguration("start_fast_lio"),
            "start_scan": LaunchConfiguration("start_scan"),
            "start_gicp": LaunchConfiguration("start_gicp"),
            "start_nav2": "false",
            "use_rviz": "false",
            "fast_lio_config": LaunchConfiguration("fast_lio_config"),
            "map": LaunchConfiguration("map"),
            "pcd_map": LaunchConfiguration("pcd_map"),
            "nav2_params": LaunchConfiguration("nav2_params"),
        }.items(),
    )

    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motion_dir, "launch", "motion_service.launch.py")
        ),
        launch_arguments={
            "local_ip": LaunchConfiguration("local_ip"),
            "dog_ip": LaunchConfiguration("dog_ip"),
            "local_port": LaunchConfiguration("local_port"),
            "voice_linear_speed_mps": LaunchConfiguration("voice_linear_speed_mps"),
            "voice_turn_yaw_rate_radps": LaunchConfiguration("voice_turn_yaw_rate_radps"),
        }.items(),
        condition=IfCondition(start_motion_service),
    )

    wait_for_nav_ready = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "quadruped_navigation_bringup",
            "wait_for_nav_ready",
            "--map",
            LaunchConfiguration("map"),
            "--params-file",
            LaunchConfiguration("nav2_params"),
            "--use-sim-time",
            use_sim_time,
            "--use-rviz",
            LaunchConfiguration("use_rviz"),
            "--timeout-sec",
            LaunchConfiguration("nav_ready_timeout_sec"),
            "--max-data-age-sec",
            LaunchConfiguration("nav_ready_max_data_age_sec"),
            "--max-odom-lag-sec",
            LaunchConfiguration("nav_ready_max_odom_lag_sec"),
            "--max-odom-lag-release-sec",
            LaunchConfiguration("nav_ready_max_odom_lag_release_sec"),
            "--stable-hold-sec",
            LaunchConfiguration("nav_ready_stable_hold_sec"),
            "--odom-frame",
            LaunchConfiguration("nav_ready_odom_frame"),
            "--base-frame",
            LaunchConfiguration("nav_ready_base_frame"),
            "--time-lag-topic",
            LaunchConfiguration("nav_ready_time_lag_topic"),
            "--restart-fast-lio-on-bad-lag",
            LaunchConfiguration("nav_ready_restart_fast_lio_on_bad_lag"),
            "--restart-lag-threshold-sec",
            LaunchConfiguration("nav_ready_restart_lag_threshold_sec"),
            "--restart-cooldown-sec",
            LaunchConfiguration("nav_ready_restart_cooldown_sec"),
            "--restart-grace-after-kill-sec",
            LaunchConfiguration("nav_ready_restart_grace_after_kill_sec"),
        ],
        output="screen",
        shell=False,
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

    run_when_ready = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "quadruped_navigation_bringup",
            "run_when_ready",
            "--command",
            LaunchConfiguration("ready_command"),
            "--timeout-sec",
            LaunchConfiguration("ready_command_timeout_sec"),
            "--max-data-age-sec",
            LaunchConfiguration("nav_ready_max_data_age_sec"),
            "--stable-hold-sec",
            LaunchConfiguration("nav_ready_stable_hold_sec"),
            "--delay-after-ready-sec",
            LaunchConfiguration("ready_command_delay_after_ready_sec"),
            "--odom-frame",
            LaunchConfiguration("nav_ready_odom_frame"),
            "--base-frame",
            LaunchConfiguration("nav_ready_base_frame"),
        ],
        output="screen",
        shell=False,
        condition=IfCondition(LaunchConfiguration("run_ready_command")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_livox", default_value="true"),
            DeclareLaunchArgument("start_fast_lio", default_value="true"),
            DeclareLaunchArgument("start_scan", default_value="true"),
            DeclareLaunchArgument("start_gicp", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_motion_service", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("fast_lio_config", default_value="mid360.yaml"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(
                    bringup_dir, "config", "maps", "map_nt_4_all.yaml"
                ),
                description="2D Nav2 map YAML.",
            ),
            DeclareLaunchArgument(
                "pcd_map",
                default_value=os.path.join(bringup_dir, "config", "pcd", "nt_4_all.pcd"),
                description="3D PCD map used by GICP localization.",
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=os.path.join(my_nav_dir, "config", "nav2_params_gicp.yaml"),
                description="Nav2 params for external GICP map->odom localization.",
            ),
            DeclareLaunchArgument("local_ip", default_value="192.168.168.1"),
            DeclareLaunchArgument("dog_ip", default_value="192.168.168.168"),
            DeclareLaunchArgument("local_port", default_value="43988"),
            DeclareLaunchArgument("voice_linear_speed_mps", default_value="0.35"),
            DeclareLaunchArgument("voice_turn_yaw_rate_radps", default_value="1.0"),
            DeclareLaunchArgument("nav_ready_timeout_sec", default_value="120"),
            DeclareLaunchArgument("nav_ready_max_data_age_sec", default_value="1.5"),
            DeclareLaunchArgument("nav_ready_max_odom_lag_sec", default_value="0.06"),
            DeclareLaunchArgument("nav_ready_max_odom_lag_release_sec", default_value="0.12"),
            DeclareLaunchArgument("nav_ready_stable_hold_sec", default_value="2.0"),
            DeclareLaunchArgument("nav_ready_odom_frame", default_value="odom"),
            DeclareLaunchArgument("nav_ready_base_frame", default_value="base_link"),
            DeclareLaunchArgument("nav_ready_time_lag_topic", default_value="/fast_lio/time_lag_sec"),
            DeclareLaunchArgument("nav_ready_restart_fast_lio_on_bad_lag", default_value="false"),
            DeclareLaunchArgument("nav_ready_restart_lag_threshold_sec", default_value="1.0"),
            DeclareLaunchArgument("nav_ready_restart_cooldown_sec", default_value="25.0"),
            DeclareLaunchArgument("nav_ready_restart_grace_after_kill_sec", default_value="15.0"),
            DeclareLaunchArgument("run_ready_command", default_value="false"),
            DeclareLaunchArgument("ready_command", default_value=""),
            DeclareLaunchArgument("ready_command_timeout_sec", default_value="180"),
            DeclareLaunchArgument("ready_command_delay_after_ready_sec", default_value="6.0"),
            nav_sensors_and_localization,
            motion_launch,
            TimerAction(period=7.0, actions=[wait_for_nav_ready]),
            TimerAction(period=10.0, actions=[scan_virtual_obstacle_node]),
            TimerAction(period=11.0, actions=[run_when_ready]),
        ]
    )
