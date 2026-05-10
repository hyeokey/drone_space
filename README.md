# drone_space

PX4 ROS 2 실험용 워크스페이스입니다. 현재 팀원이 주로 보면 되는 코드는
`ros2_ws/src/gz_camera_republisher`입니다.

`ros2_ws/src/decision_node`에 있는 코드는 지금 단계에서는 사용하지 않습니다. 빌드나 실행 중에
문제가 생기면 우선 `decision_node`는 무시하고 `gz_camera_republisher`만 확인하면 됩니다.

## 저장소 받기

이 저장소는 PX4 관련 패키지를 submodule로 포함합니다.

```bash
git clone --recurse-submodules git@github.com:hyeokey/drone_space.git
cd drone_space
```

이미 clone한 상태에서 submodule이 비어 있으면:

```bash
git submodule update --init --recursive
```

## 로컬에서 빌드하기

Ubuntu 22.04 + ROS 2 Humble 기준입니다.

필요 패키지:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  python3-colcon-common-extensions \
  libignition-transport11-dev \
  libignition-msgs8-dev \
  ros-humble-rqt-image-view \
  ros-humble-image-transport-plugins
```

빌드:

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_space/ros2_ws
colcon build --packages-select gz_camera_republisher
source install/setup.bash
```

전체 워크스페이스를 빌드하고 싶다면 `decision_node`는 제외하고 빌드하는 것을 권장합니다.

```bash
colcon build --packages-ignore decision_node
```

컨테이너와 로컬을 번갈아 쓰다가 CMake 경로 오류가 나면 기존 빌드 캐시가 다른 경로로 만들어진
상태일 수 있습니다. 이때는 `ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`를 정리한 뒤
다시 빌드하면 됩니다.

## gz_camera_republisher 실행

이 노드는 Gazebo Transport 카메라 토픽을 ROS 2 `sensor_msgs` 토픽으로 다시 publish합니다.

기본 출력 토픽:

- `/camera/image_raw`
- `/camera/camera_info`

실행:

```bash
source /opt/ros/humble/setup.bash
cd ~/drone_space/ros2_ws
source install/setup.bash
ros2 run gz_camera_republisher gz_camera_republisher
```

다른 터미널에서 확인:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /camera/image_raw
```

이미지 보기:

```bash
ros2 run rqt_image_view rqt_image_view
```

Gazebo 쪽 카메라 토픽 이름이 다르면 파라미터로 바꿔 실행할 수 있습니다.

```bash
ros2 run gz_camera_republisher gz_camera_republisher --ros-args \
  -p gz_image_topic:=/world/baylands/model/standard_vtol_mono_cam_0/link/camera_link/sensor/camera/image \
  -p gz_camera_info_topic:=/world/baylands/model/standard_vtol_mono_cam_0/link/camera_link/sensor/camera/camera_info \
  -p ros_image_topic:=/camera/image_raw \
  -p ros_camera_info_topic:=/camera/camera_info
```

## Docker로 실행하기

이미지 빌드:

```bash
cd ~/drone_space
docker build -t docker_drone ./docker_drone
```

GUI 도구를 쓸 수 있게 X11 접근을 허용합니다.

```bash
xhost +local:docker
```

컨테이너 생성:

```bash
docker run -it \
  --name drone_humble \
  --network host \
  --user $(id -u):$(id -g) \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/drone_space:/drone_space \
  -w /drone_space/ros2_ws \
  docker_drone
```

한 번 만든 컨테이너는 지우지 않으면 상태가 유지됩니다. 다음부터는 새로 `docker run` 하지 말고:

```bash
docker start -ai drone_humble
```

다른 터미널에서 같은 컨테이너에 들어가려면:

```bash
docker exec -it drone_humble bash
```

컨테이너 안에서 빌드:

```bash
source /opt/ros/humble/setup.bash
cd /drone_space/ros2_ws
colcon build --packages-select gz_camera_republisher
source install/setup.bash
```

컨테이너 안에서 노드 실행:

```bash
ros2 run gz_camera_republisher gz_camera_republisher
```

컨테이너 안에서 이미지 뷰어 실행:

```bash
ros2 run rqt_image_view rqt_image_view
```

## 권한 문제

마운트된 워크스페이스를 root 컨테이너에서 빌드하면 로컬 파일 소유권이 꼬일 수 있습니다. 그래서
컨테이너 생성 시 `--user $(id -u):$(id -g)` 옵션을 사용합니다.

이미 권한이 꼬였으면 호스트에서 복구합니다.

```bash
sudo chown -R $USER:$USER ~/drone_space
```
