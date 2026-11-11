# 2.5D 지형 기반 UGV 순찰 경로 계획 제어기

고도 정보가 있는 격자 지형에서 무인 지상 차량(UGV)의 순찰 경로를 계획하는 제어기를, 실제 차량 없이 ROS 2 시뮬레이션으로 검증하는 소프트웨어공학 프로젝트입니다.

## 무엇을 시험하나

| 시험 | 확인하는 것 |
|---|---|
| 지형 경로 계획 | 등판 한계(30°)를 넘는 경사와 장애물을 피하는 최소 비용 경로를 A\*로 만드는가 |
| 순찰 순서 | 가장 오래 방문하지 않은 지점을 먼저 가서 모든 지점을 고르게 감시하는가 |
| 장애물 재계획 | 순찰 중 경로 위에 새 장애물이 나타나면 0.5초 안에 새 경로를 만드는가 |
| 배터리 복귀 | 복귀에 필요한 에너지가 부족해지기 전에 순찰을 멈추고 기지로 돌아오는가 |
| 비정상 입력 | 지도 밖 좌표나 갈 수 없는 목표에도 멈추지 않고 경보를 내는가 |

인식 센서, 배터리, 구동부는 ROS 2 시뮬레이션 노드로 대체하고, 결과는 OpenCV 화면과 RViz2로 확인합니다.

## 실행 방법 (Ubuntu 24.04 + ROS 2 Jazzy, WSL2)

```bash
bash scripts/setup_ros2_jazzy.sh          # 처음 한 번: ROS 2 설치
bash scripts/build.sh                     # ~/ros2_ws에 빌드
source ~/ros2_ws/install/setup.bash
ros2 launch ugv_patrol_planner patrol.launch.py              # 다른 터미널에서: bash scripts/demo.sh
ros2 launch ugv_patrol_planner patrol.launch.py scenario:=$(ros2 pkg prefix ugv_patrol_planner)/share/ugv_patrol_planner/scenarios/low_battery.yaml
python3 -m pytest                         # 단위 시험 (ROS 2 없이도 실행 가능)
```

지형은 아직 실제 DEM 대신 `tools/make_synthetic_dem.py`로 만든 시험용 지도(언덕 + 30°를 넘는 절벽)를 씁니다.

## 문서

- [report1.md](report1.md) — 과제1: 프로젝트 관리 (범위, 이해관계자, 규모 산정, WBS)
- [report2.md](report2.md) — 과제2: 요구사항 (FR 19개, NFR 7개, 검증 방법)
- [report3.md](report3.md) — 과제3: 설계 (아키텍처, UML, 유닛 5개 상세 설계)
