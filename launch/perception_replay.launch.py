"""Offline evaluation adapter; original launch/configs are retained unchanged in src/."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def config(package, filename):
    return os.path.join(get_package_share_directory(package), "config", filename)


def generate_launch_description():
    waypoint = LaunchConfiguration("waypoint_csv_path")
    patchwork = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("patchworkpp"), "launch", "patchworkpp.launch.py")),
        launch_arguments={"cloud_topic": "/velodyne_points", "use_sim_time": "true", "visualize": "false"}.items())
    dbscan = Node(package="dbscan_clustering", executable="dbscan_clustering_node",
                  name="dbscan_clustering", output="screen",
                  parameters=[config("dbscan_clustering", "dbscan_params.yaml"), {"use_sim_time": True}])
    filtering = Node(package="obstacle_filtering_real", executable="obstacle_filtering_node",
                     name="obstacle_filtering", output="screen",
                     parameters=[config("obstacle_filtering_real", "obstacle_filtering.yaml"),
                                 {"use_sim_time": True, "waypoint_csv_path": ParameterValue(waypoint, value_type=str)}])
    fitting = Node(package="lshape_fitting", executable="lshape_fitting_node",
                   name="lshape_fitting_node", output="screen",
                   parameters=[config("lshape_fitting", "lshape_params.yaml"), {"use_sim_time": True}])
    tracking = Node(package="obstacle_tracking", executable="obstacle_tracking_node",
                    name="obstacle_tracking_node", output="screen",
                    parameters=[config("obstacle_tracking", "tracker_params.yaml"), {"use_sim_time": True}])
    return LaunchDescription([
        DeclareLaunchArgument("waypoint_csv_path", description="Absolute path to the matching recorded course CSV; required"),
        patchwork, TimerAction(period=2.0, actions=[dbscan]),
        TimerAction(period=4.0, actions=[filtering]), TimerAction(period=6.0, actions=[fitting]),
        TimerAction(period=8.0, actions=[tracking])])
