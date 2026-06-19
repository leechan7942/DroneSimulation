# DroneSimulation

PX4 SITL + Gazebo + ROS Noetic 환경에서 IRIS 드론을 사용하여 주어진 CSV 경로를 추종하고, 경로 주변의 장애물을 회피한 뒤 랜딩패드에 자동 착륙하는 드론 시뮬레이션 프로젝트입니다.

본 프로젝트의 경로 추종은 Gazebo waypoint marker를 기준으로 수행하지 않고, 제공된 `practice_path.csv`의 경로점을 기반으로 수행합니다. Gazebo에 표시되는 waypoint marker는 경로 추종 성능을 확인하기 위한 시각적 표시 및 평가용 기준점으로만 사용합니다.

## 실행 환경

* Ubuntu 20.04
* ROS Noetic
* Gazebo
* PX4 SITL
* MAVROS
* Docker 컨테이너: `px4_final_container`

## 주요 기능

* IRIS 드론 자동 이륙
* 목표 고도 도달 후 호버링
* CSV 경로점 기반 Pure Pursuit 경로 추종
* LiDAR 기반 장애물 감지
* Local A* 기반 장애물 회피
* 회피 후 원래 CSV 경로로 복귀
* 회피 후 CSV 경로 index 보정
* 랜딩패드 탐지 및 정렬
* AUTO.LAND 모드 전환을 통한 자동 착륙
* 테스트맵 적용 스크립트를 통한 world/csv 교체 테스트

## 프로젝트 구조

```text
DroneSimulation/
├── README.md
└── drone_practice/
    ├── launch/
    │   └── practice.launch
    ├── mission/
    │   └── practice_path.csv
    ├── scripts/
    │   ├── main.py
    │   ├── local_grid_planner.py
    │   └── landing_detector.py
    ├── worlds/
    │   └── main.world
    ├── models/
    │   ├── start_marker/
    │   ├── waypoint_marker/
    │   └── landing_pad/
    ├── test_maps/
    │   ├── baseline_success/
    │   │   ├── main.world
    │   │   └── practice_path.csv
    │   ├── map_obstacle_dense/
    │   │   ├── main.world
    │   │   └── practice_path.csv
    │   ├── map_straight/
    │   │   ├── main.world
    │   │   └── practice_path.csv
    │   ├── map_landing_precision/
    │   │   ├── main.world
    │   │   └── practice_path.csv
    │   ├── map_final_challenge/
    │   │   ├── main.world
    │   │   └── practice_path.csv
    │   └── map_avoidance_unit/
    │       ├── main.world
    │       └── practice_path.csv
    ├── tools/
    │   └── apply_test_map.sh
    ├── CMakeLists.txt
    └── package.xml
```

## 실행 방법

호스트 Ubuntu 터미널에서 Docker 컨테이너를 실행합니다.

```bash
xhost +local:root
docker start px4_final_container
docker exec -it px4_final_container bash
```

컨테이너 내부에서 ROS/PX4/Gazebo 환경을 적용합니다.

```bash
source ~/.bashrc
```

`mavlink_sitl_gazebo` 패키지가 정상 인식되는지 확인합니다.

```bash
rospack profile
rospack find mavlink_sitl_gazebo
```

정상적으로 설정되어 있으면 다음과 유사한 경로가 출력됩니다.

```text
/root/PX4-Autopilot/Tools/sitl_gazebo
```

Gazebo에서 custom marker model을 찾을 수 있도록 모델 경로를 설정합니다.

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/catkin_ws/src/DroneSimulation/drone_practice/models
```

해당 설정을 매번 적용하지 않으려면 컨테이너 내부의 `~/.bashrc`에 다음 줄을 추가합니다.

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/catkin_ws/src/DroneSimulation/drone_practice/models
```

워크스페이스를 빌드합니다.

```bash
cd ~/catkin_ws
catkin build
source ~/.bashrc
```

시뮬레이션을 실행합니다.

```bash
roslaunch drone_practice practice.launch
```

## 실행 월드 및 경로 파일

실제 `practice.launch`는 아래 파일만 사용합니다.

```text
drone_practice/worlds/main.world
drone_practice/mission/practice_path.csv
```

기존 `practice.world`, `test.world`, `test1.world`는 어떤 월드가 실행되는지 혼동을 줄이기 위해 제거하고, 실행 월드를 `main.world` 하나로 통일했습니다.

## 경로 추종 방식

본 프로젝트의 경로 추종은 waypoint marker를 따라가는 방식이 아니라, `practice_path.csv`에 저장된 경로점을 따라가는 방식입니다.

```text
drone_practice/mission/practice_path.csv
```

`main.py`는 CSV 파일의 경로점을 로드한 뒤, Pure Pursuit 방식으로 다음 목표점을 계산합니다. Gazebo에 표시된 waypoint marker는 제어 입력으로 사용하지 않으며, 경로 추종 여부를 시각적으로 확인하기 위한 표시용 객체입니다.

따라서 다음 방식은 사용하지 않습니다.

```text
- waypoint marker를 카메라로 인식해서 이동
- waypoint 좌표가 담긴 yaml 파일을 읽어서 이동
- waypoint marker 위치를 직접 목표점으로 설정
```

## 장애물 회피 및 경로 복귀 방식

장애물 감지는 LiDAR 데이터를 기반으로 수행합니다. 경로 주변에 장애물이 감지되면 `local_grid_planner.py`의 Local A* 기반 회피 경로를 사용하여 로컬 setpoint를 생성합니다.

회피가 끝난 뒤에는 다시 CSV 경로점 기반 추종으로 복귀합니다. 이때 장애물 회피 중 드론이 원래 CSV 경로에서 옆으로 벗어나면, 기존 CSV index가 뒤쪽에 남아 드론이 정체될 수 있습니다.

이를 보완하기 위해 `main.py`에는 현재 드론 위치 기준으로 가까운 앞쪽 CSV 경로점을 탐색하고, 필요한 경우 경로 index를 보정하는 로직을 추가했습니다.

이 보강은 waypoint marker를 목표점으로 사용하는 방식이 아니라, CSV 경로점 기반 추종을 안정화하기 위한 처리입니다.

## 기본 경로 및 랜딩패드 위치

기본 경로 파일은 다음 CSV를 사용합니다.

```text
drone_practice/mission/practice_path.csv
```

기본 경로의 마지막 경로점은 다음과 같습니다.

```text
12.6, 5.0, 2.5
```

이에 맞춰 `main.world`의 landing pad 위치를 다음과 같이 설정했습니다.

```text
12.6, 5.0, 0
```

랜딩패드는 CSV 마지막 경로점의 바닥에 위치하도록 설정합니다.

## 테스트맵 구조

테스트맵은 `drone_practice/test_maps/` 아래에 저장되어 있습니다. 각 테스트맵은 `main.world`와 `practice_path.csv`를 한 쌍으로 관리합니다.

```text
drone_practice/test_maps/
├── baseline_success
├── map_obstacle_dense
├── map_straight
├── map_landing_precision
├── map_final_challenge
└── map_avoidance_unit
```

각 테스트맵의 목적은 다음과 같습니다.

* `baseline_success`: 기본 성공 맵 보관용
* `map_obstacle_dense`: 장애물 밀집 환경에서 Local A* 회피 성능 확인
* `map_straight`: 회피 후 원래 경로로 복귀하는지 확인
* `map_landing_precision`: 랜딩패드 접근 및 착륙 정렬 안정성 확인
* `map_final_challenge`: CSV 경로, waypoint marker, 장애물 위치, landing pad 위치를 모두 변경한 종합 검증용 맵
* `map_avoidance_unit`: 직선 CSV 경로 위에 장애물을 배치하여 Local A* 회피 및 회피 후 CSV 경로 복귀를 확인하는 단위 검증용 맵

## 테스트맵 적용 방법

테스트맵은 `tools/apply_test_map.sh` 스크립트로 적용합니다. 이 스크립트는 선택한 테스트맵의 `main.world`와 `practice_path.csv`를 실제 실행용 파일인 `worlds/main.world`, `mission/practice_path.csv`에 복사합니다.

컨테이너 내부에서 다음과 같이 실행합니다.

```bash
cd ~/catkin_ws/src/DroneSimulation/drone_practice
./tools/apply_test_map.sh baseline_success
```

다른 테스트맵을 적용하려면 아래처럼 실행합니다.

```bash
./tools/apply_test_map.sh map_obstacle_dense
./tools/apply_test_map.sh map_straight
./tools/apply_test_map.sh map_landing_precision
./tools/apply_test_map.sh map_final_challenge
./tools/apply_test_map.sh map_avoidance_unit
```

테스트 후 기본 실행 상태로 되돌리려면 다음 명령어를 실행합니다.

```bash
./tools/apply_test_map.sh baseline_success
```

그 후 시뮬레이션을 실행합니다.

```bash
cd ~/catkin_ws
source ~/.bashrc
roslaunch drone_practice practice.launch
```

## Final Challenge 테스트맵

`map_final_challenge`는 기존 baseline과 다른 경로, waypoint marker, 장애물 위치, landing pad 위치를 사용한 종합 검증용 맵입니다.

주요 특징은 다음과 같습니다.

* 기존 baseline과 다른 CSV 경로 사용
* 경로 위의 임의 지점에 waypoint marker 6개 배치
* 마지막 waypoint는 landing pad 바로 위가 아니라 착륙 접근 구간에 배치
* landing pad는 CSV 마지막 경로점 바닥에 배치
* 장애물 8개 배치
* 장애물 회피 후 CSV 경로로 복귀하는지 확인
* 충돌 없이 경로 추종 및 착륙 단계 진입 확인

적용 방법은 다음과 같습니다.

```bash
cd ~/catkin_ws/src/DroneSimulation/drone_practice
./tools/apply_test_map.sh map_final_challenge
cd ~/catkin_ws
source ~/.bashrc
roslaunch drone_practice practice.launch
```

테스트 후 기본 맵으로 되돌립니다.

```bash
cd ~/catkin_ws/src/DroneSimulation/drone_practice
./tools/apply_test_map.sh baseline_success
```

## Avoidance Unit 테스트맵

`map_avoidance_unit`은 회피 알고리즘 단위 검증을 위해 만든 테스트맵입니다.

주요 특징은 다음과 같습니다.

* 직선 CSV 경로 사용
* CSV 경로 위에 장애물을 배치하여 회피 필요 상황 구성
* Pure Pursuit만 사용할 경우 충돌 가능성이 있는 구조
* Local A*가 작동하면 장애물을 우회한 뒤 원래 CSV 경로로 복귀
* 회피 후 CSV 경로 index 보정 로직 검증
* waypoint marker는 제어용이 아니라 경로 통과 확인용으로만 배치

적용 방법은 다음과 같습니다.

```bash
cd ~/catkin_ws/src/DroneSimulation/drone_practice
./tools/apply_test_map.sh map_avoidance_unit
cd ~/catkin_ws
source ~/.bashrc
roslaunch drone_practice practice.launch
```

테스트 후 기본 맵으로 되돌립니다.

```bash
cd ~/catkin_ws/src/DroneSimulation/drone_practice
./tools/apply_test_map.sh baseline_success
```

## 실행 확인 결과

다음 항목을 확인했습니다.

* Gazebo GUI 정상 실행
* `mavlink_sitl_gazebo` 패키지 인식
* `LocalGridPlanner` import 오류 해결
* Pure Pursuit 경로 로드
* OFFBOARD 모드 전환 성공
* ARM 성공
* Takeoff detected 확인
* CSV 경로 추종 완료
* Local A* 회피 동작 확인
* 회피 후 CSV 경로 복귀 확인
* 회피 후 CSV 경로 index 보정 동작 확인
* 랜딩패드 탐지 및 정렬 확인
* AUTO.LAND 모드 전환 성공

확인 로그 예시는 다음과 같습니다.

```text
Pure Pursuit 경로 로드 완료
OFFBOARD 모드 전환 요청 성공
기체 ARM 요청 성공
Takeoff detected
Local A* 회피 경로 추종
CSV 경로 복귀 인덱스 보정
Pure Pursuit 경로 추종 완료
랜딩패드 탐지 활성화
랜딩패드 정렬 완료: 하강 중
AUTO.LAND 모드 전환 요청 성공
```

## 종료 방법

시뮬레이션 실행 터미널에서 다음 키를 입력합니다.

```text
Ctrl + C
```

종료가 정상적으로 되지 않을 경우 컨테이너 내부에서 다음 명령어를 사용할 수 있습니다.

```bash
pkill -f roslaunch
pkill -f gzserver
pkill -f gzclient
pkill -f mavros
pkill -f px4
```

## Gazebo GUI 오류 대응

Gazebo GUI가 실행되지 않고 `No protocol specified` 또는 `gzclient process has died` 오류가 발생하면, 호스트 Ubuntu 터미널에서 다음 명령어를 실행한 뒤 다시 시도합니다.

```bash
xhost +local:root
```

컨테이너 내부에서는 다음 환경변수가 설정되어 있어야 합니다.

```bash
export QT_X11_NO_MITSHM=1
```

## `mavlink_sitl_gazebo` 패키지 오류 대응

다음과 같은 오류가 발생할 수 있습니다.

```text
Resource not found: mavlink_sitl_gazebo
```

이 경우 PX4 Gazebo package path가 현재 터미널에 적용되지 않은 것이므로 다음 명령어를 실행합니다.

```bash
source ~/.bashrc
rospack profile
rospack find mavlink_sitl_gazebo
```

정상 출력 예시는 다음과 같습니다.

```text
/root/PX4-Autopilot/Tools/sitl_gazebo
```

## Gazebo model URI 오류 대응

다음과 같은 오류가 발생할 수 있습니다.

```text
Unable to find uri[model://start_marker]
Unable to find uri[model://waypoint_marker]
Unable to find uri[model://landing_pad]
```

이 경우 Gazebo가 custom model 경로를 찾지 못한 것이므로 다음 명령어를 실행합니다.

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/catkin_ws/src/DroneSimulation/drone_practice/models
```

정상 적용 여부는 다음 명령어로 확인할 수 있습니다.

```bash
echo $GAZEBO_MODEL_PATH
```

출력에 다음 경로가 포함되어야 합니다.

```text
/root/catkin_ws/src/DroneSimulation/drone_practice/models
```

## 참고 사항

* 평가 시에는 제공된 공식 world/csv 파일을 기준으로 `main.world`, `practice_path.csv`를 교체하여 사용해야 합니다.
* 평가용 파일은 무단 수정하지 않아야 합니다.
* 테스트맵은 연습 및 검증용입니다.
* waypoint marker는 제어용 목표점이 아니라 평가 및 시각화용 표시입니다.
* 최종 경로 추종은 CSV 파일의 경로점을 기준으로 수행됩니다.
* 장애물 회피는 고도를 높이는 방식이 아니라 수평 방향 우회 방식으로 수행합니다.
* 최종 실행은 단일 명령어로 수행됩니다.

```bash
roslaunch drone_practice practice.launch
```
