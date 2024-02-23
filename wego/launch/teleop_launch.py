import launch

from launch import LaunchDescription

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution

from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    degree = LaunchConfiguration('degree')

    degree_launch_arg = DeclareLaunchArgument(
        'degree',
        default_value='0.0'
    )

    return launch.LaunchDescription([
        degree_launch_arg,

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('limo_description'), 'launch', 'load_urdf.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('wego'),
                    'launch', 
                    'camera_tilt_launch.py'
                    ])
            ]),
            launch_arguments={
                'degree': degree
            }.items()
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