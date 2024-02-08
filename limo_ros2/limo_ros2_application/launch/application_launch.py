from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limo_ros2_application',
            executable='detect_line',
            name='detect_line'
        ),
        Node(
            package='limo_ros2_application',
            executable='limo_e_stop',
            name='limo_e_stop'
        ),
        Node(
            package='limo_ros2_application',
            executable='limo_control',
            name='limo_control'
        )
    ])