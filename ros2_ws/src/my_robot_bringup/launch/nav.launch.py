import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Duong dan mac dinh (ban co the sua lai ten map mac dinh tai day)
    default_map_path = os.path.join(get_package_share_directory('my_robot_bringup'), 'config', 'my_map.yaml')
    
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Khai bao tham so 'map'
    map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load')

    # Goi nav2_bringup
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false', # Chay thuc te thi la false
            'params_file': os.path.join(get_package_share_directory('my_robot_bringup'), 'config', 'nav2.yaml') 
        }.items()
    )

    return LaunchDescription([
        map_yaml_cmd,
        nav2_cmd
    ])