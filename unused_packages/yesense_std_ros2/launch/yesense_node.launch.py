from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('yesense_std_ros2'),
        'config',
        'yesense_config.yaml',
    ),

    return LaunchDescription([
         Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            parameters=[config],
            output='screen',
            ),
        ])
