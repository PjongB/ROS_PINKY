# ROS Pinky — 자율 순찰 및 정밀 도킹

ROS 2 Jazzy와 Gazebo를 이용해 Pinky 로봇이 창고의 B, C, D, E 지점을
자율 순찰하고 A로 복귀한 뒤, LiDAR 기반 제어로 박스에 정밀 도킹하는
개인 원데이 프로젝트입니다.

최종 동작은 A 도킹 완료 자세에서 시작하며 다음 순서로 자동 실행됩니다.

```text
A 도킹 자세에서 생성
→ B → C → D → E
→ A 도킹 진입점 복귀
→ APPROACH → ALIGN_IN_PLACE → FINAL_APPROACH
→ MISSION_COMPLETE
→ 프로젝트 노드 자동 종료
```

## 1. 저장소 받기 및 최초 실행

### 개발 환경

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim 8
- Nav2
- Python 3
- `colcon`, `rosdep`

ROS 2 Jazzy가 설치되어 있다는 전제에서 진행합니다.

### 저장소 클론

```bash
cd ~
git clone https://github.com/PjongB/ROS_PINKY.git pinky
cd ~/pinky
```

### 의존성 설치

```bash
sudo rosdep init  # 이미 초기화했다면 생략
rosdep update
cd ~/pinky
rosdep install --from-paths src --ignore-src -r -y
```

`colcon`이 없다면 먼저 설치합니다.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

### 전체 워크스페이스 빌드

```bash
cd ~/pinky
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Bash alias 등록

`~/.bashrc`에 다음 내용을 추가합니다.

```bash
# Pinky ROS 2 workspace
alias pj_ws='cd ~/pinky && source /opt/ros/jazzy/setup.bash && source install/setup.bash'

# Gazebo — A 도킹 완료 자세에서 생성
alias pj_sim='pj_ws && ros2 launch pinky_project_1 sim.launch.xml'

# 프로젝트 지도와 파라미터를 사용하는 Nav2
alias pj_nav='pj_ws && ros2 launch pinky_project_1 nav.launch.py'

# RViz2
alias pj_rviz='pj_ws && ros2 launch pinky_navigation gz_nav2_view.launch.xml'

# 자율 순찰 및 최종 도킹
alias pj_project='pj_ws && ros2 launch pinky_project_1 project.launch.py'

# 개인 프로젝트 패키지만 다시 빌드
alias pj_build='cd ~/pinky && source /opt/ros/jazzy/setup.bash && colcon build --packages-select pinky_project_1 --symlink-install && source install/setup.bash'

# 수동 도킹 시험 및 취소
alias pj_docking='pj_ws && ros2 topic pub --once /docking/start std_msgs/msg/Bool "{data: true}"'
alias pj_docking_stop='pj_ws && ros2 topic pub --once /docking/start std_msgs/msg/Bool "{data: false}"'
```

등록 후 현재 터미널에 적용합니다.

```bash
source ~/.bashrc
```

## 2. 실행 순서

각 명령은 별도의 터미널에서 순서대로 실행합니다.

### 터미널 1 — Gazebo

```bash
pj_sim
```

로봇이 A의 박스에 도킹된 자세로 생성되는지 확인합니다.

### 터미널 2 — Nav2와 지도

```bash
pj_nav
```

Map Server, AMCL, planner, controller 등 Nav2 노드가 활성화될 때까지 기다립니다.

### 터미널 3 — RViz2

```bash
pj_rviz
```

Gazebo와 RViz의 로봇 위치가 A에서 일치하는지 확인합니다. AMCL 초기 자세가
자동 적용되므로 RViz의 `2D Pose Estimate`를 수동으로 지정할 필요가 없습니다.

### 터미널 4 — 전체 임무

```bash
pj_project
```

임무가 성공하면 다음 로그가 출력되고 `pj_project`는 자동 종료됩니다.

```text
Patrol and final docking complete. State: MISSION_COMPLETE
Mission complete. Shutting down pj_project.
```

Gazebo, Nav2, RViz는 계속 실행됩니다. 종료할 때는 RViz → Nav2 → Gazebo
순서로 각 터미널에서 `Ctrl+C`를 누릅니다.

## 3. 프로젝트 기획

### 목표

- A 도킹 완료 자세에서 동일한 조건으로 시작
- B, C, D, E 지점을 빠짐없이 자율 방문
- 순찰 후 A 도킹 진입점으로 자동 복귀
- 벽과 충돌하지 않고 목표 거리까지 정밀 도킹
- 파라미터와 실험 결과를 재현할 수 있도록 기록

### 완료 기준

- 명확한 실행 순서로 전체 시스템을 시작할 수 있음
- 모든 웨이포인트 방문과 A 복귀 성공
- 최종 거리와 평행 자세를 만족하며 도킹 성공
- 성공 시 프로젝트 노드가 안전하게 정지하고 자동 종료

## 4. 동작 시나리오

```text
WAITING_NAV
→ B → C → D → E
→ A
→ FINAL_DOCKING
   ├─ APPROACH
   ├─ ALIGN_IN_PLACE
   └─ FINAL_APPROACH
→ MISSION_COMPLETE
→ SHUTDOWN
```

- `WAITING_NAV`: Nav2 action server 준비 및 다음 목표 전송 대기
- `B`~`E`: 각 순찰 지점으로 이동
- `A`: 도킹 진입 좌표로 복귀하고 후방이 박스를 향하도록 정렬
- `APPROACH`: 거리와 각도를 함께 보정하며 약 0.10m까지 접근
- `ALIGN_IN_PLACE`: 선속도를 0으로 두고 박스 면과 평행하게 정렬
- `FINAL_APPROACH`: 각도를 고정하고 거리 PID로 직선 후진
- `MISSION_COMPLETE`: 정지 및 성공 상태 출력
- `SHUTDOWN`: `patrol_manager` 종료 후 launch 전체 종료

목표 이동에 실패하면 설정된 횟수만큼 재시도합니다. 센서 데이터가 없거나
오래되면 속도를 발행하지 않으며, 제한 시간을 넘으면 실패 상태로 정지합니다.

## 5. 프로젝트 구조

```text
pinky/
├── README.md
├── src/
│   ├── pinky_pro/                 # Pinky 모델, Gazebo, Nav2 기반 패키지
│   ├── pinky_edu/                 # 학습용 Nav2 스크립트
│   └── pinky_project_1/           # 개인 프로젝트 패키지
│       ├── launch/
│       │   ├── sim.launch.xml
│       │   ├── nav.launch.py
│       │   └── project.launch.py
│       ├── config/
│       │   ├── nav2_params.yaml
│       │   └── project_params.yaml
│       ├── map/
│       │   ├── tutorial_map.pgm
│       │   └── tutorial_map.yaml
│       ├── media/
│       │   └── *.webm
│       └── pinky_project_1/
│           ├── patrol_manager.py
│           ├── docking_controller.py
│           └── pid_controller.py
├── build/                         # 생성 파일, Git 제외
├── install/                       # 생성 파일, Git 제외
└── log/                           # 생성 파일, Git 제외
```

### 주요 파일

- `sim.launch.xml`: A 도킹 완료 자세로 로봇을 생성하고 Gazebo bridge 실행
- `nav.launch.py`: 프로젝트 지도와 Nav2 파라미터로 upstream bringup 실행
- `project.launch.py`: 순찰·도킹 노드 실행 및 임무 완료 시 전체 종료
- `patrol_manager.py`: NavigateToPose action, 순찰 순서, 재시도, 상태 관리
- `docking_controller.py`: LaserScan, PCA 직선 피팅, 거리·각도 제어
- `pid_controller.py`: `dt`, 출력 제한, 적분 포화 방지를 포함한 PID 계산
- `project_params.yaml`: 웨이포인트와 도킹 제어 파라미터
- `nav2_params.yaml`: AMCL, goal checker, costmap과 Nav2 설정

## 6. 좌표와 초기 자세

### 웨이포인트

| 지점 | x (m) | y (m) | yaw (deg) |
|---|---:|---:|---:|
| A | 1.40 | 0.03 | 180.0 |
| B | -1.62 | 1.05 | 0.0 |
| C | 1.40 | 1.05 | 0.0 |
| D | -1.62 | -0.02 | 0.0 |
| E | 1.40 | -1.10 | 0.0 |

A는 최종 도킹 자세가 아니라 Nav2가 사용하는 도킹 진입 좌표입니다.

### 시작 자세

Gazebo world 좌표와 AMCL map 좌표는 서로 다른 검증값을 사용합니다.

```yaml
# Gazebo
spawn_x: 1.547
spawn_y: -0.024
spawn_z: 0.1
spawn_yaw: -3.134

# AMCL
initial_pose:
  x: 1.516
  y: -0.024
  z: 0.0
  yaw: -3.134

# Mission
start_docked: true
shutdown_on_complete: true
```

## 7. Nav2 최종 설정

| 파라미터 | 값 | 목적 |
|---|---:|---|
| `xy_goal_tolerance` | 0.05m | 웨이포인트 도착 정확도 향상 |
| `yaw_goal_tolerance` | 0.05rad | A 복귀 방향 오차 축소 |
| `inflation_radius` | 0.30m | 벽과 선반에서 여유 확보 |
| `cost_scaling_factor` | 2.0 | 장애물 비용을 더 먼 거리까지 유지 |
| `use_sim_time` | `true` | Gazebo, AMCL, costmap, Nav2 시간 통일 |

## 8. 정밀 도킹 제어

Pinky의 LiDAR 프레임은 차체 기준 180도 회전되어 있어 LaserScan의 0도가
로봇 후방을 향합니다. 후방 점군에서 박스 면을 추출하고 PCA 직선 피팅으로
기울기를 계산합니다.

### 최종 파라미터

| 구분 | 파라미터 | 값 |
|---|---|---:|
| 거리 | `target_distance` | 0.06m |
| 거리 | `distance_tolerance` | 0.005m |
| 거리 PID | `Kp / Ki / Kd` | 0.4 / 0.0 / 0.05 |
| 거리 | `max_linear_speed` | 0.08m/s |
| 정렬 | `alignment_distance` | 0.10m |
| 정렬 | `angle_tolerance` | 0.087rad (약 5°) |
| 정렬 | `required_alignment_cycles` | 5회 |
| 정렬 | `alignment_min_duration` | 2.0초 |
| PCA | `alignment_fit_window_deg` | 25.0° |
| PCA | `alignment_range_tolerance` | 0.05m |
| PCA | `alignment_min_points` | 10개 |
| 각도 P | `Kp / Ki / Kd` | 0.2 / 0.0 / 0.0 |
| 각도 | `max_angular_speed` | 0.05rad/s |
| 안전 | `docking_timeout` | 45.0초 |

도킹 완료 경계는 `target_distance + distance_tolerance`, 즉 약 0.065m입니다.

## 9. 테스트 기록

### Nav2 자율주행

| 테스트 | 변경 내용 | 결과 |
|---|---|---|
| 1 | 기본 허용 오차와 작은 inflation | A 정렬 실패, 목표 중단 |
| 2 | 위치·각도 허용 오차 축소, inflation 확대 | B~E 순찰 및 A 복귀 성공 |

### PID 및 각도 정렬

| 테스트 | 제어 방식 | 결과 |
|---|---|---|
| 1 | 거리 PID만 사용 | 거리 성공, 평행 정렬 실패 |
| 2 | 각도 `Kp=0.5`, 최대 0.15rad/s | 평행도 개선, 진동과 시간 초과 |
| 3 | `Kp=0.2`, 최대 0.05rad/s, 중앙값 필터 | 흔들림 감소, 완료 판정 시간 초과 |
| 4 | 0.08m 이하에서 각도 보정 중지 | 완료 판정 성공, 자세 실패 |
| 5 | PCA와 3단계 접근·정렬·후진 | 거리와 평행 자세 모두 성공 |

### 최종 통합 테스트

- A 도킹 완료 자세에서 정상 생성
- Gazebo와 RViz 초기 위치 일치
- 초기 A 이동·도킹을 생략하고 B부터 출발
- B, C, D, E 모든 지점 도착
- A 자동 복귀 및 최종 평행 도킹 성공
- 최종 LiDAR 거리 약 0.050~0.067m
- 충돌과 시간 초과 없음
- 최종 상태 `MISSION_COMPLETE`

시험 영상은 [`src/pinky_project_1/media`](src/pinky_project_1/media)에 있습니다.

## 10. 문제 해결 기록

### RViz Local Costmap — `No map received`

토픽은 발행됐지만 AMCL, controller, local/global costmap의 시간 기준이 달라
`map → odom` TF 변환이 실패한 문제였습니다. 프로젝트 전용
`nav2_params.yaml`에서 관련 노드를 모두 `use_sim_time: true`로 통일한 뒤
Local Costmap과 전체 자율주행이 정상 동작했습니다.

### AMCL 초기 위치가 `(0, 0, 0)`으로 표시되는 경우

ROS 2 Jazzy AMCL은 초기 자세를 다음 구조로 받아야 합니다.

```yaml
initial_pose:
  x: 1.516
  y: -0.024
  z: 0.0
  yaw: -3.134
```

리스트 형식인 `initial_pose: [x, y, yaw]`를 사용하면 초기값이 적용되지 않을 수
있습니다. 설정 변경 후 Nav2와 RViz를 다시 실행합니다.

### 코드 변경 후 적용

```bash
pj_build
```

실행 중인 관련 노드를 종료한 후 Gazebo → Nav2 → RViz → 프로젝트 순서로
다시 실행합니다.

## 11. 수동 도킹 시험

전체 임무와 별도로 도킹만 시험할 때 사용합니다. 로봇 후방이 박스를 향하고
LiDAR가 박스 면을 감지하는 상태에서 실행해야 합니다.

```bash
pj_docking
```

상태 확인:

```bash
ros2 topic echo /docking/status
```

도킹 취소:

```bash
pj_docking_stop
```

## 12. 문서와 출처

- [프로젝트 Notion 문서](https://app.notion.com/p/3bce8d3fa9438036af15e9a859080ffc)
- 개인 프로젝트 패키지: `src/pinky_project_1`
- Pinky 기반 패키지와 라이선스: `src/pinky_pro`

`src/pinky_pro`의 기존 라이선스와 저작권 고지는 해당 디렉터리의
[`LICENSE`](src/pinky_pro/LICENSE)를 따릅니다.
