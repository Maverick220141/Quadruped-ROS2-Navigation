from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明启动参数
    local_ip_arg = DeclareLaunchArgument(
        'local_ip',
        default_value='192.168.168.1',
        description='本地IP地址，用于与机器狗通信'
    )
    
    dog_ip_arg = DeclareLaunchArgument(
        'dog_ip',
        default_value='192.168.168.168',
        description='机器狗的IP地址'
    )
    
    local_port_arg = DeclareLaunchArgument(
        'local_port',
        default_value='43988',
        description='本地端口，用于与机器狗通信'
    )

    voice_linear_speed_arg = DeclareLaunchArgument(
        'voice_linear_speed_mps',
        default_value='0.35',
        description='语音前进/后退相对运动线速度，单位 m/s'
    )

    voice_turn_yaw_rate_arg = DeclareLaunchArgument(
        'voice_turn_yaw_rate_radps',
        default_value='1.0',
        description='语音原地转圈角速度，单位 rad/s'
    )
    
    # 创建机器狗控制节点
    agibot_node = Node(
        package='agibot_motion_service',
        executable='motion_server_node',
        name='motion_server_node',
        output='screen',
        parameters=[
            {
                'local_ip': LaunchConfiguration('local_ip'),
                'dog_ip': LaunchConfiguration('dog_ip'),
                'local_port': LaunchConfiguration('local_port'),
                'voice_linear_speed_mps': LaunchConfiguration('voice_linear_speed_mps'),
                'voice_turn_yaw_rate_radps': LaunchConfiguration('voice_turn_yaw_rate_radps')
            }
        ]
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(local_ip_arg)
    ld.add_action(dog_ip_arg)
    ld.add_action(local_port_arg)
    ld.add_action(voice_linear_speed_arg)
    ld.add_action(voice_turn_yaw_rate_arg)
    
    # 添加节点
    ld.add_action(agibot_node)
    
    return ld
