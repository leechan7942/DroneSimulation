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
* 랜딩패드 탐지 및 정렬
* AUTO.LAND 모드 전환을 통한 자동 착륙

## 프로젝트 구조

```text
DroneSimulation/
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

## 월드 파일

실행 월드는 다음 파일 하나로 통일했습니다.

```text
drone_practice/worlds/main.world
```

기존 `practice.world`, `test.world`, `test1.world`는 혼동을 줄이기 위해 제거했습니다.

## 경로 및 랜딩패드

경로 파일은 다음 CSV를 사용합니다.

```text
drone_practice/mission/practice_path.csv
```

CSV 마지막 경로점은 다음과 같습니다.

```text
12.6, 5.0, 2.5
```

이에 맞춰 `main.world`의 landing pad 위치를 다음과 같이 설정했습니다.

```text
12.6, 5.0, 0
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

## 참고 사항

Gazebo GUI가 실행되지 않고 `No protocol specified` 또는 `gzclient process has died` 오류가 발생하면, 호스트 Ubuntu 터미널에서 다음 명령어를 실행한 뒤 다시 시도합니다.

```bash
xhost +local:root
```
