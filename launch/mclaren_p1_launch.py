import os                                 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'mclaren_robo'           

    # ---------- resource paths ----------
    world_path = os.path.join(
        get_package_share_directory(package_name),
        'world', 'hennesey_world.world'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'description', 'mclaren_robot.rviz'
    )


    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch', 'mclaren_p1_rsp_launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_path,
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros', executable='spawn_entity.py',
                name='spawn_my_bot',
                arguments=['-entity', 'my_bot', '-topic', 'robot_description'],
                output='screen'
            )
        ]
    )

    # ---------- safety node ----------
    wall_alert = Node(
        package='mclaren_robo',
        executable='wall_alert_node',
        name='wall_alert_node',
        output='screen',
        parameters=[{'alert_distance': 0.65}]
    )

    # ---------- teleop ----------
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        prefix='xterm -e',              
        output='screen',
        remappings=[('/cmd_vel', '/raw_cmd_vel')]
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    delayed_rviz = TimerAction(
        period=6.0,       
        actions=[rviz_node]
    )

    # ---------- assemble launch description ----------
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_robot,
        teleop,
        jsp,
        wall_alert,
        delayed_rviz,     
    ])



