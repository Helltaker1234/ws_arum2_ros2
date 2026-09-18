from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    target_size_x = LaunchConfiguration("target_size_x")
    target_size_y = LaunchConfiguration("target_size_y")
    target_size_z = LaunchConfiguration("target_size_z")
    target_offset_x = LaunchConfiguration("target_offset_x")
    target_offset_y = LaunchConfiguration("target_offset_y")
    target_offset_z = LaunchConfiguration("target_offset_z")

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
                "target_size_x": ParameterValue(target_size_x, value_type=float),
                "target_size_y": ParameterValue(target_size_y, value_type=float),
                "target_size_z": ParameterValue(target_size_z, value_type=float),
                "target_offset_x": ParameterValue(target_offset_x, value_type=float),
                "target_offset_y": ParameterValue(target_offset_y, value_type=float),
                "target_offset_z": ParameterValue(target_offset_z, value_type=float),
                "octomap_rebuild_wait_seconds": ParameterValue(
                    LaunchConfiguration("octomap_rebuild_wait_seconds"), value_type=float
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_size_x", default_value="0.08"),
            DeclareLaunchArgument("target_size_y", default_value="0.08"),
            DeclareLaunchArgument("target_size_z", default_value="0.12"),
            DeclareLaunchArgument("target_offset_x", default_value="0.0"),
            DeclareLaunchArgument("target_offset_y", default_value="0.0"),
            DeclareLaunchArgument("target_offset_z", default_value="-0.05"), # GPD 라이브러리 특성상 pre grasp 위치가 위에 살짝 떠있기에, 아래로 offset 살짝 줌.
            DeclareLaunchArgument("octomap_rebuild_wait_seconds", default_value="1.0"),
            mtc_node,
        ]
    )
