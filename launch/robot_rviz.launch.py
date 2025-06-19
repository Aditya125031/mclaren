from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('shitty_robo'),
        'description',
        'robot.urdf.xacro'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True}],
            arguments=[Command(['xacro ', xacro_path])],
            output='screen'
        )
    ])


