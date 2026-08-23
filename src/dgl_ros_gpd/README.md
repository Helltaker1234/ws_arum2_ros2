# dgl_ros GPD action server

이 패키지는 최신 전처리 `PointCloud2`를 저장하고, Action goal이 들어오면
`atenpas/gpd` 추론을 실행해 `/sample_grasp_poses` feedback으로 grasp pose와
cost를 반환한다. Action 형식은 `DeepGraspPose`가 사용하는
`dgl_ros_interfaces/action/SampleGraspPoses`와 호환된다.

## 외부 의존성

GPD는 ROS 패키지가 아니므로 별도로 빌드하고 설치해야 한다.

```bash
git clone https://github.com/atenpas/gpd.git
cd gpd && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

GPD 설정 파일의 `weights_file`, hand geometry, workspace를 실제 로봇과
작업 영역에 맞게 설정해야 한다. `/usr/local` 이외에 설치했다면 colcon에
`--cmake-args -DGPD_ROOT=/absolute/install/prefix`를 전달한다.

## 빌드와 실행

```bash
colcon build --packages-up-to dgl_ros_gpd
source install/setup.bash
ros2 launch dgl_ros_gpd gpd.launch.py \
  gpd_config_path:=/absolute/path/to/gpd/cfg/eigen_params.cfg \
  input_topic:=/camera/depth/points_cleaned \
  camera_frame:=camera_depth_optical_frame
```

`camera_frame`을 지정하면 TF에서 cloud frame 기준 카메라 원점을 구한다.
전처리 cloud가 이미 카메라 좌표계에 있고 원점이 `(0, 0, 0)`이면 생략할 수
있다.

Action 단독 확인:

```bash
ros2 action send_goal --feedback /sample_grasp_poses \
  dgl_ros_interfaces/action/SampleGraspPoses \
  "{action_name: sample_grasp_poses}"
```

GPD score가 클수록 좋은 후보이고 MTC cost는 작을수록 좋으므로 기본 변환은
`cost = -score`이다. `cost_scale`, `cost_offset` 파라미터로 조정할 수 있다.
