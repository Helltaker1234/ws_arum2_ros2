import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. 경로 설정
    moveit_config_pkg = get_package_share_directory("areum2_moveit_config")
    gazebo_ros_pkg = get_package_share_directory("gazebo_ros")
    
    # 2. MoveIt 설정 로드 (use_sim_time 필수)
    moveit_config = (
        MoveItConfigsBuilder("areum2_robot", package_name="areum2_moveit_config")
        .robot_description(
            file_path="config/A1_URDF.urdf.xacro",
            mappings={"ros2_control_hardware_type": "gazebo"} # 하드웨어 타입을 gazebo로 명시
        )
        .robot_description_semantic(file_path="config/A1_URDF.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # 3. Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gazebo.launch.py")
        ),
    )

    # 4. Gazebo에 로봇 소환 (Spawn)
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "areum2_robot"],
        output="screen",
    )

    # 5. MoveGroup 노드 (use_sim_time 설정 추가)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    # 6. RViz2
    rviz_config = os.path.join(moveit_config_pkg, "launch", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    # 7. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    # 8. 컨트롤러 스포너 (Gazebo가 뜬 후 실행되어야 함)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["areum2_arm_l_controller"],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher,
        move_group_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])