import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. MoveItConfigsBuilder를 통해 로봇의 핵심 파라미터 정보 로드
    moveit_config = MoveItConfigsBuilder("A1_URDF", package_name="areum2_moveit_config").to_moveit_configs()

    # 2. RViz 설정 파일 경로 지정 (보통 config/moveit.rviz에 위치)
    rviz_config_file = os.path.join(
        get_package_share_directory("areum2_moveit_config"),
        "config",
        "moveit.rviz"
    )

    # 3. 내장 함수 대신 명시적으로 RViz 노드를 생성하고 파라미터 주입
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([rviz_node])