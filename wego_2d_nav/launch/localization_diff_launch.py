import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    wego_share_dir = get_package_share_directory('wego_2d_nav')

    # setting for map path
    map_file_name = 'map.yaml' # set the map file name
    map_file_path = os.path.join(wego_share_dir, 'maps', map_file_name)

    # setting for the param path
    param_file_name = 'diff_navigation_params.yaml'
    param_file_path = os.path.join(wego_share_dir, 'params', param_file_name)

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = ['map_server', 'amcl']

    # For loading map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=False,
        respawn_delay=2.0,
        parameters=[{'yaml_filename': map_file_path}],
        remappings=remappings,
    )

    # for adaptive monte localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=False,
        respawn_delay=2.0,
        parameters=[param_file_path],
        remappings=remappings,
    )

    # For lifecycle
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])