from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration



def generate_launch_description():

    declare_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, real_hardware, isaac]",
    )

    hardware_type = LaunchConfiguration("ros2_control_hardware_type")

    moveit_config =( 
        MoveItConfigsBuilder(
            "areumii", 
            package_name="areum_ii_moveit_config"
        )
        .robot_description(
            mappings={"ros2_control_hardware_type" : hardware_type,}
        )
        .to_moveit_configs()
    )

    demo_launch = generate_demo_launch(moveit_config)

    return LaunchDescription(
        [
            declare_hardware_type,
            *demo_launch.entities,
        ]
    )
