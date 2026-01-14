import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')
    desc_pkg = get_package_share_directory('amr_description')
    
    use_fake_odom = LaunchConfiguration('use_fake_odom')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_odom', default_value='false'),

        # Real Driver (If use_fake_odom=false)
        Node(
            package='my_robot_base_driver', executable='base_driver_node',
            name='base_driver_real',
            parameters=[{'port': '/dev/ttyACM0', 'baudrate': 115200, 'publish_odom': True}],
            condition=UnlessCondition(use_fake_odom)
        ),
        # Silent Driver (If use_fake_odom=true)
        Node(
            package='my_robot_base_driver', executable='base_driver_node',
            name='base_driver_silent',
            parameters=[{'port': '/dev/ttyACM0', 'baudrate': 115200, 'publish_odom': False}],
            condition=IfCondition(use_fake_odom)
        ),
        # Fake Odom Node
        Node(
            package='amr_fake_odom', executable='fake_odom_node',
            condition=IfCondition(use_fake_odom)
        ),
        # Lidar
        Node(
            package='sllidar_ros2', executable='sllidar_node',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'serial_baudrate': 115200, 
                         'frame_id': 'laser_frame', 'inverted': False, 'angle_compensate': True}]
        ),
        # URDF
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            arguments=[os.path.join(desc_pkg, 'urdf', 'amr.urdf')],
            parameters=[{'use_sim_time': False}]
        )
    ])