from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "areumii",
            package_name="areum_ii_moveit_config",
        )
        .to_moveit_configs()
    )

    mtc_node = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        name="mtc_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {
                "target_object_id": ParameterValue(
                    LaunchConfiguration("target_object_id"), value_type=str
                ),
                "octomap_rebuild_wait_seconds": ParameterValue(
                    LaunchConfiguration("octomap_rebuild_wait_seconds"), value_type=float
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_object_id", default_value="target_object"),
            DeclareLaunchArgument("octomap_rebuild_wait_seconds", default_value="1.0"),
            mtc_node,
        ]
    )
