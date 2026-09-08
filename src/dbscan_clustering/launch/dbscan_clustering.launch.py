from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('dbscan_clustering')
    default_params = os.path.join(pkg_share, 'config', 'dbscan_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to params yaml'
    )

    node = Node(
        package='dbscan_clustering',
        executable='dbscan_clustering_node',     # ← 실제 실행 파일명 확인!
        name='dbscan_clustering',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        #arguments=["--ros-args", "--qos-history=keep_last", "--qos-depth=1"],
    )

    return LaunchDescription([params_arg, node])

