from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controls',
            node_namespace='sub',
            node_executable='velocity_transformer',
            node_name='velocity_transformer'
        ),
        Node(
            package='teleop_twist_keyboard',
            node_namespace='sub',
            node_executable='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            node_name='teleop_twist_keyboard'
        ),
    ])
