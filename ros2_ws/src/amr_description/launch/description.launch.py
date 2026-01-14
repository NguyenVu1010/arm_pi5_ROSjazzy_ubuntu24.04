from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share/amr_description/urdf/amr.urdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf).read()}]
        )
    ])