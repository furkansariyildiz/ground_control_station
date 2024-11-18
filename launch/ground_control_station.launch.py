import os
from launch import LaunchDescription
from launch_ros.actions import SetParameter, Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ground_control_station',
            executable='ground_control_station',
            name='ground_control_station',
            output='screen',
            parameters=[os.path.join(
                get_package_share_directory('ground_control_station'),
                'config', 'config.yaml'
            )]
        )
    ])