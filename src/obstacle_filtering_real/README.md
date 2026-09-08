# obstacle_filtering_real

`obstacle_filtering_real`는 `/pointcloud/clustered`의 클러스터를 평균 대표점으로 요약하고,
UTM waypoint CSV를 ENU(map)로 변환해 바운더리 내부 여부를 판정하는 ROS2 C++ 노드입니다.

## 1. 이번 버전 핵심 변경점
- 대표점 계산: **클러스터 평균점으로 고정** (center topic 옵션 제거)
- 좌표계: **map=ENU** 고정
- ego pose: `localization_tf`가 발행하는 `map -> base_link` TF를 cloud stamp 기준으로 lookup
- map 원점: waypoint CSV 첫 `R1` UTM 기준
- 바운더리 판정: 대표점과 각 포인트의 `min(dist_to_left, dist_to_right)` 히스테리시스
- fail-closed: TF 없음/100ms 초과 stale이면 TF 의존 출력을 비워서 false track 생성을 차단
- 출력 정렬: ego 거리 가까운 순으로 정렬 후 `id=0,1,2...` 재할당

## 2. 입력/출력
### 입력
- `/pointcloud/clustered` (`sensor_msgs/msg/PointCloud2`)
- TF: `map -> base_link` (`localization_tf`에서 발행)

### 출력
- `/obstacle_in` (`sensor_msgs/msg/PointCloud2`)  
  `/pointcloud/clustered`와 동일 레이아웃, 단 `cluster_id`는 가까운 순으로 `0..N-1` 재할당
- `/obstacle_out` (`sensor_msgs/msg/PointCloud2`)  
  `/pointcloud/clustered`와 동일 레이아웃, 단 `cluster_id`는 가까운 순으로 `0..N-1` 재할당
- `/obstacle_in_debug` (`std_msgs/msg/String`)
- `/obstacle_out_debug` (`std_msgs/msg/String`)
- `/obstacle_in_repr` (`sensor_msgs/msg/PointCloud2`, 대표점 디버그)
- `/obstacle_out_repr` (`sensor_msgs/msg/PointCloud2`, 대표점 디버그)

## 3. 동작 알고리즘
1. waypoint CSV(UTM)를 읽고 첫 `R1` UTM을 ENU 원점으로 캐시
2. `/pointcloud/clustered.header.stamp` 시점의 `map -> base_link` TF lookup
3. TF age `<=50ms`는 정상, `50~100ms`는 warning 후 사용, `>100ms`/없음은 fail-closed
4. pointcloud를 cluster_id별로 묶고 평균 대표점 계산
5. 대표점을 `lidar->base` 고정 오프셋으로 base 좌표화 후, TF pose로 map(ENU) 투영
6. 차량 주변 waypoint window에서 좌/우 경계선까지의 최근접 거리로 in/out 판정
7. fail-closed 시 `/obstacle_in`, `/obstacle_out`을 비워 TF 불일치 프레임을 차단
8. in/out 분류 후 ego 거리순 정렬, `id=0..N-1` 부여
9. 분류 결과를 원본 포인트클라우드 레이아웃으로 재구성해 `/obstacle_in`, `/obstacle_out` 발행
10. 디버그 문자열은 `/obstacle_in_debug`, `/obstacle_out_debug`로 분리 발행

## 4. waypoint CSV 요구사항
기본적으로 아래 열 중 left/right UTM X/Y는 반드시 필요합니다.
- left: `L1_UTM_X`, `L1_UTM_Y` (또는 `LEFT_UTM_X`, `LEFT_UTM_Y`)
- right: `R1_UTM_X`, `R1_UTM_Y` (또는 `RIGHT_UTM_X`, `RIGHT_UTM_Y`)
- optional: `INDEX`, `L1_ALT`, `R1_ALT`

`waypoint_csv_path`만 바꾸면 재컴파일 없이 다른 경로를 바로 사용할 수 있습니다.

## 5. 파라미터
`config/obstacle_filtering.yaml`
- `waypoint_csv_path`: waypoint CSV 경로
- `front_waypoint_count`, `rear_waypoint_count`: 차량 주변 window
- `waypoint_interp_step_m`: 경계 보간 간격
- `max_detection_range_m`: ego 기준 최대 탐지 범위
- `min_obstacle_distance_m`: 근접 자기반사 제거
- `in_radius_m`, `out_radius_m`: 좌/우 경계 최근접 거리 기반 히스테리시스
- `inside_point_ratio_threshold`: 클러스터 내부 포인트 비율 기준
- `waypoint_window_use_lidar_roi`, `waypoint_roi_*`: waypoint window ROI
- `base_to_lidar_*`: base->lidar 고정 오프셋
- `publish_debug_text`, `publish_debug_cloud`: 디버그 토픽 on/off

## 6. 실행
```bash
cd <racing_ws>
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select obstacle_filtering_real
source install/setup.bash
ros2 launch obstacle_filtering_real obstacle_filtering.launch.py
```

## 7. rosbag 확인 사항
`<bag_dir>/metadata.yaml` 기준으로 아래 토픽 존재 확인:
- `/fix` (`sensor_msgs/msg/NavSatFix`)
- `/imu/data` (`sensor_msgs/msg/Imu`)
- `/tf` 중 `map -> base_link`

현재 노드는 `/fix`, `/imu/data`, `/vel`을 직접 구독하지 않습니다.

## 8. MATLAB 디버깅
파일:
- `matlab/visualize_obstacle_filtering.m`
- `matlab/tune_yaw_offset_from_bag.m`
- `matlab/README_TUNING.md`

기능:
- waypoint 경계(left/right) 표시
- `/obstacle_in_repr`, `/obstacle_out_repr` 대표점 표시
- `/tf`의 `map->base_link` ego 위치 표시

MATLAB에서 바꿔가며 볼 항목:
- `cfg.waypointCsv`
- `cfg.inTopic`, `cfg.outTopic`
- TF 상태, `base_to_lidar_*`, waypoint window 크기
