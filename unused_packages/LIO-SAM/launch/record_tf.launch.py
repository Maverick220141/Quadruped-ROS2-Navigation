# 文件名: record_tf.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    # 确保这里的路径指向您的 URDF 文件
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')

    # 这个节点专门用于读取URDF并发布静态TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # 使用Command直接执行xacro命令来加载URDF
            'robot_description': Command(['xacro', ' ', xacro_path])
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])