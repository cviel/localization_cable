from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    shared_params = {
        "L": 6.0,
        "z_offset": 0.0,
    }
    
    return LaunchDescription([
        Node(
            package='interval_computation',
            executable='interval_computation',
            name='interval_computation',
            parameters=[{"sivia_eps": 0.278, "IMU_precision": 0.053, "L_precision": 0.05, "z_precision": 0.05} | shared_params],
        ),
        Node(
            package='imus',
            executable='imus',
            name='imus',
            parameters=[{"port": "/dev/ttyUSB0", "topic_name": "IMU_comp"}],
        ),
        Node(
            package='simulation',
            executable='simulation',
            name='simulation',
            parameters=[shared_params],
        ),
    ])