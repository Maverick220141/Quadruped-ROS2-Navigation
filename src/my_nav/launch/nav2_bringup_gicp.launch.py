import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_dir = get_package_share_directory("my_nav")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    map_yaml_path = LaunchConfiguration("map")
    params_file_path = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    bt_xml = os.path.join(package_dir, "behavior_trees", "my_nav_tree.xml")
    configured_params = RewrittenYaml(
        source_file=params_file_path,
        root_key="",
        param_rewrites={"default_nav_to_pose_bt_xml": bt_xml},
        convert_types=True,
    )

    declare_map_arg = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(package_dir, "maps", "map1.yaml"),
        description="Full path to map YAML file to load",
    )

    declare_params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(package_dir, "config", "nav2_params_gicp.yaml"),
        description="Full path to the Nav2 parameter file",
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Whether to start RViz2",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        condition=IfCondition(use_rviz),
    )

    # Start Nav2 navigation without AMCL/localization; GICP owns map -> odom.
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
        }.items(),
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml_path},
            {"use_sim_time": use_sim_time},
        ],
    )

    lifecycle_manager_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    delay_action = TimerAction(
        period=3.0,
        actions=[map_server_node, lifecycle_manager_map, nav2_navigation],
    )

    return LaunchDescription(
        [
            declare_map_arg,
            declare_params_arg,
            declare_use_sim_time_arg,
            declare_use_rviz_arg,
            rviz_node,
            delay_action,
        ]
    )
