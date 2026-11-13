# 2.5D 지형 기반 UGV 순찰 경로 계획 제어기

[![CI](https://github.com/jjaewon-Hong/ugv_patrol_planner/actions/workflows/ci.yml/badge.svg)](https://github.com/jjaewon-Hong/ugv_patrol_planner/actions/workflows/ci.yml)

고도 정보가 있는 격자 지형에서 무인 지상 차량(UGV)의 순찰 경로를 계획하는 제어기를, 실제 차량 없이 ROS 2 시뮬레이션으로 검증하는 소프트웨어공학 프로젝트입니다. 시험용 지형에서 검증한 뒤, **실제 지역(영남대 경산캠퍼스)의 항공 정사영상에서 장애물을 인식해 만든 지도**로 순찰하도록 확장했습니다. 경로 계획 핵심은 C++17로도 구현해 Python 버전과 같은 결과를 내는지 자동으로 확인합니다.

```
항공 정사영상 → 이어 붙이기·자르기 → 좌표 보정(OSM) → 장애물 인식(PyTorch 분할 모델) → 장애물 격자 ┐
DEM(고도)     → 같은 10 m 격자로 리샘플링 ──────────────────────────────────────────→ 고도 격자   ┴→ ROS 2 순찰 제어기 (A*, 재계획, 기지 복귀)
```

## 진행 현황

| 단계 | 내용 | 상태 |
|---|---|---|
| 1 | 요구사항·설계 (report1\~3), 핵심 유닛 U1\~U5, ROS 2 노드, 단위·시나리오 시험 | 완료 |
| 2 | 캠퍼스 정사영상 확보, 이어 붙이기·자르기 (`tools/prepare_ortho.py`) | 완료 |
| 3 | 장애물 인식: PyTorch DeepLabV3를 LoveDA로 학습해 캠퍼스 추론 (`notebooks/`), OSM 정답 지도와 IoU 비교 (`tools/campus_map.py`) | 완료 (NFR-08 기준 미달, 아래 검증 결과 참고) |
| 4 | 좌표 보정, 고도 데이터 정합, 깃발 무작위 배치, 캠퍼스 시나리오 S5 | 완료 (`campus`: 모델 지도, `campus_osm`: OSM 지도) |
| 5 | 핵심 알고리즘 C++17 포팅 (`cpp/`), Docker·GitHub Actions 자동 시험 | 완료 |

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
bash scripts/setup_ros2_jazzy.sh     # 처음 한 번: ROS 2 설치
bash scripts/build.sh                # ~/ros2_ws에 빌드
bash scripts/run.sh                  # 제어기 + 시뮬레이션 + 2D 화면 + RViz2 실행 (run.sh low_battery, run.sh campus, run.sh campus_osm)
bash scripts/demo.sh                 # 다른 터미널에서: 순찰 지점 등록 후 시작 (run.sh와 같은 이름, 예: demo.sh campus)
python3 -m pytest                    # 단위 시험 59개 (ROS 2 없이도 실행 가능)
source ~/ros2_ws/install/setup.bash && python3 scripts/scenario_test.py   # 시나리오 시험 S1~S5
bash scripts/ci.sh                   # CI와 같은 전체 시험 (Python, C++, C++ vs Python, 시나리오)
```

Docker로 실행하면 ROS 2를 설치하지 않아도 됩니다. GitHub Actions도 push마다 같은 이미지로 전체 시험을 돌립니다.

```bash
docker build -t ugv-patrol . && docker run --rm ugv-patrol
```

기본 지형은 `tools/make_synthetic_dem.py`로 만든 시험용 지도(언덕 + 30°를 넘는 절벽)이고, `campus`는 영남대 캠퍼스 1 km × 1 km를 10 m 칸 100×100으로 만든 지도입니다. 캠퍼스에서는 기지에서 갈 수 있는 빈 칸에 깃발 3개를 시드로 무작위 배치합니다.

## 데이터

| 데이터 | 출처 | 비고 |
|---|---|---|
| 항공 정사영상 (2025, 25 cm, 도엽 35804061·35804071) | 국토지리정보원 국토정보플랫폼 | 원본은 용량과 이용 조건 때문에 저장소에 넣지 않음. `tools/prepare_ortho.py`로 캠퍼스 1.02 km × 1.53 km를 잘라 사용 |
| 고도 (Copernicus GLO-30 DEM) | © DLR e.V. 2010-2014 and © Airbus Defence and Space GmbH 2014-2018, provided under COPERNICUS by the European Union and ESA | 캠퍼스 범위를 10 m 격자로 리샘플링해 `data/campus_osm/dem.npy`에 포함 (`tools/campus_world.py`) |
| 건물·물·숲 윤곽 | © OpenStreetMap contributors (ODbL) | 좌표 보정과 정답 지도에 사용, 10 m 장애물 격자를 `data/campus_osm/obstacles.npy`에 포함 (`tools/osm_mask.py`) |
| 학습 데이터 (LoveDA) | Wang et al., NeurIPS 2021, Zenodo 5706578 | Kaggle 노트북에서 내려받아 학습에만 사용 |
| 시험용 지형 | `tools/make_synthetic_dem.py` | 저장소의 `data/`에 포함 |

## 검증 결과

| 항목 | 결과 |
|---|---|
| 단위 시험 (U1\~U5, 깃발 배치, 좌표 변환) | 59개 통과 |
| 시나리오 시험 (실제 ROS 2 실행) | S1 정상 순찰, S2 장애물 발견, S3 배터리 부족, S4 비정상 입력, S5 캠퍼스 순찰 모두 통과 |
| 정사영상 좌표 보정 | 도엽 번호로 위치를 계산한 뒤 OSM 건물 윤곽에 맞춰 13 m 보정 |
| 분할 모델 학습 (LoveDA 검증 세트) | mIoU 0.511 (15에폭, Kaggle T4 2시간 18분), 건물 0.63, 물 0.69, 숲 0.38 |
| 캠퍼스 인식 vs OSM (NFR-08, 기준 0.6) | 건물+숲 IoU 0.44, 건물 0.37, 숲 0.43 → **미달** |
| 캠퍼스 모델 지도 순찰 (S5) | 장애물 칸 48 %, 깃발 3개 모두 방문, 장애물 칸 진입 0회 |
| 재계획 시간 (100×100 지도, ROS 2 실행 중) | 평균 약 50 ms (요구사항 0.5초 이내) |
| C++ A\* vs Python A\* (지도 3종 × 200쌍) | 600개 경로가 칸 단위로 모두 같음, 비용 차이 5×10⁻¹⁰ 이하 |
| C++ A\* 속도 | 평균 0.15\~0.24 ms, 최대 1.0 ms (Python 2.8\~5.6 ms, 최대 32 ms) → 약 20배 |
| C++ 단위 시험 (GoogleTest) | 11개 통과 (Dijkstra 기준 최적성, 등판 한계, 장애물, 복귀 에너지, .npy 읽기) |

NFR-08 미달 원인 분석 (`tools/campus_map.py eval` 결과를 나눠 본 값):
- **정답 지도(OSM)가 불완전함:** 모델이 찾은 숲의 48 %는 가로수·작은 숲으로 OSM에 없고, 모델이 찾은 건물의 49 %도 OSM에 없는 곳(누락 건물, 지붕 가장자리)입니다. OSM 숲 안의 27 %는 실제로는 풀밭·빈터입니다.
- **학습 데이터와 지역 차이:** LoveDA는 중국 도시 영상이라, OSM 건물 면적 중 모델이 찾은 비율(재현율)은 0.58에 그칩니다.
- **남은 위치 오차:** 예측과 OSM 건물 사이에 약 7 m 어긋남이 남아 있으며, 보정하면 건물 IoU가 0.37에서 0.40으로 오릅니다.

다음 단계: 캠퍼스 일부를 직접 라벨링한 검증 영역으로 정답을 보완하고, 국내 항공영상 데이터로 미세 조정합니다.

## C++17 코어 (`cpp/`)

ROS에 의존하지 않는 라이브러리로 U1 지형(경사 한계·이동 비용), U2 A\*, U4 복귀 에너지 지도(역방향 Dijkstra)를 옮겼습니다.

- **같은 결과 보장:** 이웃 순서와 동점 처리 규칙(f, h, 삽입 순서)을 Python과 똑같이 맞췄습니다. `tools/compare_cpp.py`가 같은 지도·같은 출발/도착 쌍으로 두 구현을 돌려 경로가 칸 단위로 같은지 확인합니다.
- **속도:** 칸 좌표 대신 정수 인덱스와 연속 배열(`std::vector`)을 쓰고, 4방향 이동 비용을 미리 계산해 탐색 중에는 조회만 합니다.
- **빌드:** CMake + GoogleTest, `-Wall -Wextra -Wpedantic -Werror`.

```bash
cmake -S cpp -B build_cpp && cmake --build build_cpp -j && (cd build_cpp && ctest)
echo "0 0 99 99" | build_cpp/ugv_plan data/dem_100x100.npy - 30    # 비용, 계획 시간, 경로 출력
```

## ROS 2 통신과 DDS QoS

| 토픽 | 메시지 | QoS | 이유 |
|---|---|---|---|
| `/controller/path`, `/mission/status`, `/viz/markers` | Path, String(JSON), MarkerArray | Reliable + **Transient Local**, depth 1 | 상태성 데이터라 마지막 값만 의미가 있음. 늦게 켠 화면·RViz2·시험 노드도 접속 즉시 현재 경로와 상태를 받음 |
| `/operator/command`, `/mission/alarm` | String | Reliable, Volatile, depth 10 | 명령·경보는 하나도 빠지면 안 되고, 지난 명령을 나중에 다시 받으면 안 됨 |
| `/perception/obstacles` | PoseArray (헤더 시각 포함) | Reliable, depth 10 | 장애물 보고를 놓치면 충돌 위험. 헤더 시각으로 재계획 지연(0.5초)을 측정 |
| `/ugv/pose`, `/battery/level`, `/controller/next_cell` | Point, Float32 | Reliable, depth 10 | 시뮬레이션에서는 시험 결과가 매번 같도록 Reliable 사용 |

- 실제 센서를 붙이면 `/ugv/pose`, `/battery/level`처럼 주기가 빠르고 최신 값만 중요한 토픽은 **Best Effort**가 맞습니다. 이때 구독자가 Reliable이면 발행자(Best Effort)와 QoS가 맞지 않아 메시지를 받지 못하므로, 구독 쪽도 함께 바꿔야 합니다.
- 시나리오 시험은 `ROS_DOMAIN_ID=77`로 따로 돌려, 같은 PC에서 데모를 띄워 둬도 DDS 통신이 섞이지 않습니다.

## 개발 방식

요구사항, 설계(아키텍처·유닛 설계), 시험 기준은 직접 정의했고, 구현에는 AI 코딩 도구(Claude Code)를 활용했습니다. 모든 기능은 위의 단위 시험과 시나리오 시험으로 검증했으며, 시험 과정에서 발견한 성능 문제(재계획 지연)와 버그는 원인을 분석해 수정했습니다.

## 문서

- [report1.md](report1.md) — 과제1: 프로젝트 관리 (범위, 이해관계자, 규모 산정, WBS)
- [report2.md](report2.md) — 과제2: 요구사항 (FR 22개, NFR 8개, 변경 이력 v1.1, 검증 방법)
- [report3.md](report3.md) — 과제3: 설계 (아키텍처, UML, 유닛 5개 상세 설계, 구현 단계 설계 보완)
