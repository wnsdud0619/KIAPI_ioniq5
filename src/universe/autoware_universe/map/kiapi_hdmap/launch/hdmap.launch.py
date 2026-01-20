import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 실행 시 받을 인자(Argument) 선언
    # ros2 launch kiapi_hdmap hdmap.launch.py map_path:=/원하는/경로
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value='/home/kiapi/Downloads/KIAPI_SHP_260107 (final)', # 기본값
        description='Path to the SHP files directory'
    )

    # 2. 노드 설정
    hdmap_node = Node(
        package='kiapi_hdmap',
        executable='visualizer',
        name='hd_map_visualizer',
        output='screen',
        parameters=[{
            'map_path': LaunchConfiguration('map_path') # 위에서 받은 Argument를 파라미터로 전달
        }]
    )

    # 3. LaunchDescription 반환
    return LaunchDescription([
        map_path_arg,
        hdmap_node
    ])
