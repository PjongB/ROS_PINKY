# pinky_project_1

ROS 2 Jazzy 기반 자율 순찰 및 LiDAR 정밀 도킹 패키지입니다.

이 패키지는 Pinky를 A 도킹 완료 자세에서 시작시키고 B → C → D → E → A
경로를 순찰한 뒤, PCA 기반 평행 정렬과 거리 PID로 최종 도킹합니다. 임무가
완료되면 `pj_project` launch가 자동 종료됩니다.

설치 방법, alias, 전체 실행 순서, 파라미터와 테스트 기록은 저장소 루트의
[`README.md`](../../README.md)를 참고하세요.

## 직접 실행

```bash
cd ~/pinky
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch pinky_project_1 sim.launch.xml
ros2 launch pinky_project_1 nav.launch.py
ros2 launch pinky_navigation gz_nav2_view.launch.xml
ros2 launch pinky_project_1 project.launch.py
```

위 명령은 Gazebo, Nav2, RViz, 프로젝트 순서로 각각 별도 터미널에서
실행합니다.
