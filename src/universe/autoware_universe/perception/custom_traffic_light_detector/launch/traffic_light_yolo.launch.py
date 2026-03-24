import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 외부에서 입력받을 파라미터(Argument) 선언 및 기본값 설정
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/kiapi/KIAPI_ioniq5/src/universe/autoware_universe/perception/custom_traffic_light_detector/model/best.pt',
        description='Path to the YOLO model file (.pt)'
    )
    
    osm_map_path_arg = DeclareLaunchArgument(
        'osm_map_path',
        default_value='/home/kiapi/KIAPI_ioniq5/map_data/lanelet2_map.osm', # 기본 맵 경로를 적어두면 편합니다
        description='Path to the Lanelet2 OSM map file (.osm)'
    )

    # 2. 실행할 노드 설정 및 파라미터 전달
    yolo_detector_node = Node(
        package='custom_traffic_light_detector',
        executable='traffic_light_yolo_detector',
        name='traffic_light_yolo_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'osm_map_path': LaunchConfiguration('osm_map_path')
        }]
    )

    # 3. LaunchDescription에 추가하여 반환
    return LaunchDescription([
        model_path_arg,
        osm_map_path_arg,
        yolo_detector_node
    ])
