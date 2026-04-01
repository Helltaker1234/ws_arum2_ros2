from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    # 1. 터미널에서 입력받을 use_sim_time 설정 (기본값: False -> 현실 플랜트 기준)
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    # 2. 이 런치 파일 내에서 실행되는 "모든 노드"에 use_sim_time 파라미터를 일괄 적용
    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    # 3. 기존 MoveIt 설정 로드 및 MoveGroup 런치 생성
    moveit_config = MoveItConfigsBuilder("A1_URDF", package_name="areum2_moveit_config").to_moveit_configs()
    move_group_ld = generate_move_group_launch(moveit_config)

    # 4. 최종 LaunchDescription 조합
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(set_sim_time)

    # generate_move_group_launch가 만들어낸 노드와 설정들을 가져와서 덧붙임
    for entity in move_group_ld.entities:
        ld.add_action(entity)

    return ld