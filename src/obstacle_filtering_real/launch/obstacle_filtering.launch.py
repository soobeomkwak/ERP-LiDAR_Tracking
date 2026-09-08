import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('obstacle_filtering_real')
    default_params = os.path.join(pkg_share, 'config', 'obstacle_filtering.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='YAML parameter file for obstacle_filtering node',
    )

    node = Node(
        package='obstacle_filtering_real',
        executable='obstacle_filtering_node',
        name='obstacle_filtering',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_arg,
        node,
    ])
