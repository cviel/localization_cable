from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='magnetometer_calibration',
            executable='magnetometer_visualizer',
            name='mag_visualizer',
        ),
    ])
