from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imus',
            executable='imus',
            name='imus',
            parameters=[{"port": "/dev/ttyAMA0", "topic_name": "IMU_ROV"}],
        ),
        Node(
            package='bar30',
            executable='bar30',
            name='bar30',
            parameters=[{"pressure_calibration": 100000.0}],
        ),
    ])