from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_path = PathJoinSubstitution([FindPackageShare("sllidar_ros2"), "launch", "sllidar_c1_launch.py"])
    params_file_path = PathJoinSubstitution([FindPackageShare("sllidar_ros2"), "config", "obstacle_detector_params.yaml"])

    return LaunchDescription([
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(launch_file_path)
        ),
        Node(
            package="sllidar_ros2",
            executable="sllidar_obstacle_detection",
            parameters=[params_file_path],
            name="sllidar_obstacle_detection",
        )
    ])