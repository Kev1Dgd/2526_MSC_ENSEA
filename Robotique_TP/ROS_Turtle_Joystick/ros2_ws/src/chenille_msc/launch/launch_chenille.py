import launch
import launch_ros.actions

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    joy_vel = LaunchConfiguration('joy_vel')

    return launch.LaunchDescription([

        # Arguments optionnels
        DeclareLaunchArgument('joy_vel', default_value='/turtle1/cmd_vel'),
        DeclareLaunchArgument('joy_dev', default_value='0'),

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

        # Node chenille
        launch_ros.actions.Node(
            package='chenille_msc',
            executable='chenille_node',
            name='chenille_node'
        ),

        # Node turtlesim
        launch_ros.actions.Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        )
    ])
