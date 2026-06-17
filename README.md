# DroneSimulation

PX4 SITL + Gazebo + ROS Noetic 환경에서 IRIS 드론을 사용하여 주어진 경로를 추종하고, 장애물을 회피한 뒤 랜딩패드에 자동 착륙하는 드론 시뮬레이션 프로젝트입니다.

## 실행 환경

* Ubuntu 20.04
* ROS Noetic
* Gazebo
* PX4 SITL
* MAVROS
* Docker 컨테이너: `px4_final_container`

## 주요 기능

* IRIS 드론 자동 이륙
* Pure Pursuit 기반 경로 추종
* Local A* 기반 장애물 회피
* 카메라 이미지 기반 랜딩패드 탐지 및 정렬
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
    │   └── map_landing_precision/
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

컨테이너 내부에서 ROS 환경을 적용합니다.

```bash
source ~/.bashrc
```

워크스페이스를 빌드합니다.

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
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

## 경로 및 랜딩패드 위치

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

## 테스트맵 구조

테스트맵은 `drone_practice/test_maps/` 아래에 저장되어 있습니다. 각 테스트맵은 `main.world`와 `practice_path.csv`를 한 쌍으로 관리합니다.

```text
drone_practice/test_maps/
├── baseline_success
├── map_obstacle_dense
├── map_straight
└── map_landing_precision
```

각 테스트맵의 목적은 다음과 같습니다.

* `baseline_success`: 기본 성공 맵 보관용
* `map_obstacle_dense`: 장애물 밀집 환경에서 Local A* 회피 성능 확인
* `map_straight`: 회피 후 원래 경로로 복귀하는지 확인
* `map_landing_precision`: 랜딩패드 접근 및 착륙 정렬 안정성 확인

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
```

테스트 후 기본 실행 상태로 되돌리려면 다음 명령어를 실행합니다.

```bash
./tools/apply_test_map.sh baseline_success
```

그 후 시뮬레이션을 실행합니다.

```bash
cd ~/catkin_ws
roslaunch drone_practice practice.launch
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
* 경로 추종 완료
* Local A* 회피 동작 확인
* 랜딩패드 탐지 및 정렬 확인
* AUTO.LAND 모드 전환 성공

확인 로그 예시는 다음과 같습니다.

```text
Pure Pursuit 경로 로드 완료
OFFBOARD 모드 전환 요청 성공
기체 ARM 요청 성공
Takeoff detected
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

## 참고 사항

* 평가 시에는 제공된 공식 world/csv 파일을 기준으로 `main.world`, `practice_path.csv`를 교체하여 사용해야 합니다.
* 평가용 파일은 무단 수정하지 않아야 합니다.
* 테스트맵은 연습 및 검증용입니다.
* 최종 실행은 단일 명령어로 수행됩니다.

```bash
roslaunch drone_practice practice.launch
```
