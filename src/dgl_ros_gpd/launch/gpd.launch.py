from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_parameters = PathJoinSubstitution(
        [FindPackageShare("dgl_ros_gpd"), "config", "gpd_server.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "parameters_file",
                default_value=default_parameters,
                description="ROS parameter file for the GPD action server",
            ),
            DeclareLaunchArgument(
                "gpd_config_path",
                description="Absolute path to the atenpas/gpd configuration file",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="/camera/depth/points_cleaned",
                description="Preprocessed PointCloud2 topic",
            ),
            DeclareLaunchArgument(
                "camera_frame",
                default_value="",
                description="Camera optical frame; empty uses camera_view_point",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use Isaac Sim clock"
            ),
            Node(
                package="dgl_ros_gpd",
                executable="gpd_action_server",
                name="dgl_ros_gpd",
                output="screen",
                parameters=[
                    LaunchConfiguration("parameters_file"),
                    {
                        "gpd_config_path": LaunchConfiguration("gpd_config_path"),
                        "input_topic": LaunchConfiguration("input_topic"),
                        "camera_frame": LaunchConfiguration("camera_frame"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
        ]
    )
