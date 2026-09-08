import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _share(package_name: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package_name), *parts)


def _include(package_name: str, relative_launch_path: str, launch_arguments=None):
    launch_file = _share(package_name, relative_launch_path)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_topic = LaunchConfiguration('cloud_topic')
    visualize_patchwork = LaunchConfiguration('visualize_patchwork')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated time for the post-process pipeline.',
    )
    declare_cloud_topic = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/velodyne_points',
        description='Input point cloud topic from bag playback or the Velodyne launch.',
    )
    declare_visualize_patchwork = DeclareLaunchArgument(
        'visualize_patchwork',
        default_value='false',
        description='Launch RViz from patchworkpp.',
    )

    patchwork = _include(
        'patchworkpp',
        'launch/patchworkpp.launch.py',
        {
            'cloud_topic': cloud_topic,
            'use_sim_time': use_sim_time,
            'visualize': visualize_patchwork,
        },
    )
    dbscan = _include(
        'dbscan_clustering',
        'launch/dbscan_clustering.launch.py',
        {'params_file': _share('dbscan_clustering', 'config', 'dbscan_params.yaml')},
    )
    obstacle_filtering = _include(
        'obstacle_filtering_real',
        'launch/obstacle_filtering.launch.py',
        {'params_file': _share('obstacle_filtering_real', 'config', 'obstacle_filtering.yaml')},
    )
    lshape = _include(
        'lshape_fitting',
        'launch/lshape_fitting.launch.py',
        {
            'params_file': _share('lshape_fitting', 'config', 'lshape_params.yaml'),
            'use_sim_time': use_sim_time,
        },
    )
    obstacle_tracking = _include(
        'obstacle_tracking',
        'launch/obstacle_tracking.launch.py',
        {
            'params_file': _share('obstacle_tracking', 'config', 'tracker_params.yaml'),
            'use_sim_time': use_sim_time,
        },
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_cloud_topic,
        declare_visualize_patchwork,
        patchwork,
        TimerAction(period=2.0, actions=[dbscan]),
        TimerAction(period=4.0, actions=[obstacle_filtering]),
        TimerAction(period=6.0, actions=[lshape]),
        TimerAction(period=8.0, actions=[obstacle_tracking]),
    ])
