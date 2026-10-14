#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
cd /home/juna/ws_arum2_ros2/
source ~/ws_arum2_ros2/install/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/ws_hardware_interface/install/setup.bash

ROS2_BIN=/opt/ros/humble/bin/ros2
pids=()

# Stop the process groups started by this script when it exits.
cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid "$ROS2_BIN" launch areum_ii_moveit_config demo_sim.launch.py \
    ros2_control_hardware_type:=real_hardware \
    capabilities:="move_group/ExecuteTaskSolutionCapability" &
pids+=("$!")

sleep 20

setsid "$ROS2_BIN" launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    rgb_camera.color_profile:=1280x720x15 \
    depth_module.depth_profile:=1280x720x15 \
    align_depth.enable:=true \
    enable_sync:=true \
    pointcloud.enable:=false \
    enable_gyro:=false \
    enable_accel:=false \
    clip_distance:=1.0 \
    spatial_filter.enable:=true &
pids+=("$!")

sleep 2

setsid "$ROS2_BIN" launch areum_ii_moveit_config depth_pointcloud.launch.py &
pids+=("$!")

sleep 2

setsid "$ROS2_BIN" launch pointcloud_preprocessor preprocessing.launch.py &
pids+=("$!")

sleep 2

setsid "$ROS2_BIN" run dgl_ros_models gpd --ros-args \
    -p src_topic0:=/camera/depth/points_cleaned_no_table_removed \
    -p src_frame0:=base_link \
    -p world_frame:=world \
    -p gpd_config_path:=/home/juna/ws_arum2_ros2/src/dgl_ros/dgl_ros_models/config/gpd_config_real.yaml &
pids+=("$!")

wait
