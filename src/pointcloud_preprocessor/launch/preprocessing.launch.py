from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("pointcloud_preprocessor"), "config", "preprocessing.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Point-cloud preprocessing parameter file",
            ),
            Node(
                package="pointcloud_preprocessor",
                executable="pointcloud_preprocessor_node",
                name="pointcloud_preprocessor",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
