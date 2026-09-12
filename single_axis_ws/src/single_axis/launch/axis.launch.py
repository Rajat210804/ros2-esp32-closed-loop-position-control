"""Bring up the axis driver.

    ros2 launch single_axis axis.launch.py                 # real hardware
    ros2 launch single_axis axis.launch.py mock:=true      # no hardware needed
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    port = LaunchConfiguration('port')

    params = PathJoinSubstitution(
        [FindPackageShare('single_axis'), 'config', 'axis_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mock', default_value='false',
            description='Run against the simulated axis instead of real hardware.'),
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='Serial port the ESP32 enumerates on.'),

        Node(
            package='single_axis',
            executable='axis_driver',
            name='axis_driver',
            output='screen',
            parameters=[params, {'mock': mock, 'port': port}],
        ),
    ])
