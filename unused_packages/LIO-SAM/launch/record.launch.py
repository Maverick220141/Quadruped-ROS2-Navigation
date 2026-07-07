# 这个文件可以命名为 record.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')
    
    robot_description = ParameterValue(Command([FindExecutable(name='xacro'), ' ', xacro_path]), value_type=str)
    
    
    return LaunchDescription([
        params_declare,
        
        # 我们只需要这个节点！它的作用是读取URDF并发布 /tf_static
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # 以下所有LIO-SAM算法节点和RViz节点都已被删除
        # Node(package='tf2_ros', ...),
        # Node(package='lio_sam', executable='lio_sam_imuPreintegration', ...),
        # Node(package='lio_sam', executable='lio_sam_imageProjection', ...),
        # Node(package='lio_sam', executable='lio_sam_featureExtraction', ...),
        # Node(package='lio_sam', executable='lio_sam_mapOptimization', ...),
        # Node(package='rviz2', ...),
    ])
