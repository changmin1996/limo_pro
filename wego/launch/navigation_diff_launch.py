import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wego_share_dir = get_package_share_directory('wego')

    # setting for rviz configuration path
    rviz_file_name = 'navigation.rviz'
    rviz_config_path = os.path.join(wego_share_dir, 'rviz', rviz_file_name)

    # For localization
    localization_launch = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('wego_2d_nav'),
                'launch', 
                'localization_diff_launch.py'
            ])
        )


    # For navigation



    # setting for rviz
    rviz_config_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )

    return LaunchDescription([
        localization_launch,
        rviz_config_node,
    ])