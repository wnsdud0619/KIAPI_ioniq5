from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autoware_convert_topic',
            executable='autoware_convert_topic',
            name='convert_node',
            output='screen',
            emulate_tty=True,  # This is useful for proper output formatting in the terminal
            parameters=[]
        )
    ])

