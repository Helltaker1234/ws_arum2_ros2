#!/bin/bash

# Load ~/.bashrc in an interactive Bash so the start_ros alias is available.
exec bash -ic '
set -e
start_ros
cd ~/ws_arum2_ros2/
source ~/ws_arum2_ros2/install/setup.bash
source ~/ws_moveit/install/setup.bash
exec ros2 launch mtc_tutorial mtc.launch.py
'
