import launch

from launch import LaunchDescription
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             name='base_link_to_camera',
             arguments = ['--x', '0.012', 
                          '--y', '-0.05',
                          '--z', '0.033', 
                          '--yaw', '0', 
                          '--pitch', '0', 
                          '--roll', '0', 
                          '--frame-id', 'base_link', 
                          '--child-frame-id', 'camera_link']
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('limo_description'), 'launch', 'load_urdf.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('limo_base'), 'launch', 'limo_base.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('astra_camera'), 'launch', 'dabai.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('ydlidar_ros2_driver'), 'launch', 'ydlidar.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('robot_localization'), 'launch', 'limo_ekf_launch.py'])
        )
  ])