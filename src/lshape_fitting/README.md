# lshape_fitting

기본적으로 `/obstacle_in`(`cluster_id` 포함 PointCloud2)를 입력으로 받아 클러스터별 L-shape(직사각형) 피팅을 수행합니다. 필요하면 파라미터로 다른 입력 토픽도 지정할 수 있습니다.

- 입력: `sensor_msgs/msg/PointCloud2`
- 출력1: `visualization_msgs/msg/MarkerArray` (`/target/markers`)
- 출력2: `lshape_fitting/msg/Detection2DArray` (`/target/poses`)
- 출력3: `geometry_msgs/msg/PoseArray` (`/target/poses_vis`, RViz에서 yaw/방향 확인용)

## 알고리즘

Xiao et al. (2017)의 search-based rectangle fitting을 기반으로 합니다.

1. 클러스터별 XY 투영
2. `theta`를 `[0, 90deg)` 범위에서 탐색
3. 각 `theta`에서 projection으로 rectangle 후보 계산
4. 기준함수(`closeness`/`variance`/`area`/`hybrid`) 최대값 선택
5. 길이/폭/높이/종횡비 게이트로 최종 박스 필터링

## 빌드/실행

```bash
colcon build --packages-select lshape_fitting
source install/setup.bash
ros2 launch lshape_fitting lshape_fitting.launch.py
```

## 튜닝 포인트

- `config/lshape_params.yaml`
  - `delta_theta_deg`: 작게 할수록 heading 정밀도↑, 연산량↑
  - `score_mode`: 기본 `closeness`; 노이즈가 많으면 `variance`도 시험
  - `d0`: 너무 작으면 근접점이 과도하게 점수 지배
  - `use_fixed_output_height`, `fixed_output_height_m`: 마커 높이 고정(요청 기본 1.0m)
  - `pose_vis_topic`: 커스텀 msg 대신 RViz에서 yaw를 확인할 `PoseArray` 토픽
  - `publish_text_markers`: 박스 위에 `id/yaw/length/width` 텍스트 표시
  - `fit_vehicle_spec_only`, `vehicle_*`: ERP42 규격 기반 통과 게이트
  - `min/max_length,width,height`: 오검출 제거에 가장 효과적
  - `max_aspect_ratio`: 비차량형 길쭉 클러스터 제거

- `src/lshape_fitting_node.cpp`
  - `search_best_fit()`: 기준함수/점수결합 로직
  - `fit_cluster_to_box()`: 크기 게이트/종횡비 게이트
  - `cloud_callback()`: cluster field 파싱(`cluster_id`, `cluster` fallback)

## 품질 개선을 위한 연계 튜닝 (기존 패키지)

- `src/dbscan_clustering/config/dbscan_params.yaml`
  - `eps`, `min_points`, `max_neighbors`를 먼저 안정화해야 L-shape 품질이 올라갑니다.
  - 군집이 분할되면 `eps`↑ 또는 `min_points`↓
  - 군집이 합쳐지면 `eps`↓ 또는 ROI/ego box 강화

- `src/dbscan_clustering/src/dbscan_clustering_node.cpp`
  - `cluster_xy_only`가 `true`이면 수직 구조물 분리에는 유리/불리 케이스가 갈립니다.
  - 자차 잔여점이 많으면 ego box(`ego_*`)를 조정해야 L-shape 왜곡이 줄어듭니다.
