# MATLAB 빠른 튜닝 절차

## 1) IMU yaw 보정값 자동 추정
MATLAB에서 실행:
```matlab
cd('<racing_ws>/src/erp42_racing_perception/external/obstacle_filtering_real/matlab')
best = tune_yaw_offset_from_bag('<bag_dir>');
```

출력된 `best` 값을 `config/obstacle_filtering.yaml`의
`imu_yaw_correction_deg`에 반영.

## 2) 실시간 시각화
ROS2 시스템 실행 후:
```matlab
visualize_obstacle_filtering('<waypoint_csv>')
```

표시:
- 좌/우 경계선
- in/out 대표점
- ego(map->base_link)

## 3) 자주 조정하는 파라미터
- `imu_yaw_correction_deg`
- `base_to_ant1_x_m`, `base_to_ant1_y_m`
- `base_to_lidar_x_m`, `base_to_lidar_y_m`
- `front_waypoint_count`, `rear_waypoint_count`
- `max_detection_range_m`, `min_obstacle_distance_m`
