from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    depth_to_pointcloud = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node", # depth 데이터를 x, y, z point cloud 데이터로 변환
        name="point_cloud_xyz",
        namespace="camera/depth",
        output="screen",
        parameters=[
            {
                "queue_size": 5,
            }
        ],
        remappings=[
            (   # point_cloud_xyz_node 가 구독할 토픽을 /camera/camera/aligned_depth_to_color/image_raw 로 리매핑
                "image_rect",
                "/camera/camera/aligned_depth_to_color/image_raw",
            ),
            (   # 리매핑
                "camera_info",
                "/camera/camera/aligned_depth_to_color/camera_info",
            ),
            (   # 리매핑
                "points", 
                "/camera/depth/points"
            ),
        ],
    )

    return LaunchDescription([depth_to_pointcloud])
