from launch import LaunchDescription
from launch_ros.actions import Node
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
        ],
    )

    return LaunchDescription([mtc_node])