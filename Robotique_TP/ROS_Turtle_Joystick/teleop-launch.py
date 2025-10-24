import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    joy_vel = launch.substitutions.LaunchConfiguration('joy_vel')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')

    return launch.LaunchDescription([
        # Arguments optionnels
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='/turtle1/cmd_vel'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='0'),
        launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='false'),

        # Node joystick
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),

        # Node teleop directement avec les paramètres inline
        launch_ros.actions.Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'enable_button': 5,
		'require_enable_button' : False,# Pas besoin d'activer			         # toujours actif
                'axis_linear.x': 1,             # stick gauche vertical
                'axis_angular.yaw': 3,          # stick droit horizontal
                'scale_linear.x': 1.0,
                'scale_angular.yaw': 1.5,
                'publish_stamped_twist': publish_stamped_twist
            }],
            remappings=[('/cmd_vel', joy_vel)],
        ),

        # Node turtlesim
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
    ])
