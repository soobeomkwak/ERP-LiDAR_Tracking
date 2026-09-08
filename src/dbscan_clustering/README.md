# dbscan_clustering

간단한 설명
----------------
`dbscan_clustering`는 GPU 가속 이웃 쿼리를 사용해 LiDAR 포인트클라우드에 DBSCAN 클러스터링을 수행하고, 각 포인트에 클러스터 ID(`cluster_id`, int32)와 RGB 색상(`rgb`, float32 packed)을 추가하여 ROS2의 `sensor_msgs/PointCloud2`로 퍼블리시하는 패키지입니다. NaN/Inf 좌표는 전처리 단계에서 제거(컴팩트)하여 알고리즘과 시각화 문제를 줄였습니다.

핵심 설계(한 문장 계약)
- 입력: PointCloud2 (필수 필드 `x`,`y`,`z`) — 노이즈(Inf/NaN) 점은 필터링됨
- 출력: PointCloud2 (유효점만 퍼블리시). 각 점은 `x,y,z`(float32), `cluster_id`(int32), `rgb`(packed float32)를 가짐
- 성능: 이웃 검색은 CUDA 커널(장치)에서 수행, DBSCAN 확장(클러스터 형성)은 CPU에서 수행

요구사항
- ROS 2 (패키지 빌드 및 실행 환경)
- CUDA 툴킷 및 GPU (이웃 쿼리 가속용)
- colcon / ament_cmake 빌드 툴

기본 알고리즘
----------------
1. PointCloud2에서 `x,y,z` 필드를 읽어들임 (FLOAT32/64 지원).
2. NaN/Inf인 포인트를 제거해 유효 포인트만 모아 'compact' 배열을 만듦(원래 인덱스는 `index_map`에 보관).
3. CUDA 커널로 각 포인트에 대해 반경 `eps` 내의 이웃들을 최대 `max_neighbors`개까지 찾아 `neighbors` 리스트와 `neighbor_counts`를 반환.
4. CPU에서 DBSCAN 확장 단계 수행(큐 기반 BFS): 이웃 수가 `min_points` 이상이면 핵심점으로 판단하여 클러스터 확장.
5. compact 결과(`labels_compact`)를 원래 인덱스 공간으로 다시 매핑해 `labels_full`을 만들고, 유효 포인트들만 `cluster_id`와 `rgb`를 붙여 퍼블리시.

출력 필드
- `x` (FLOAT32)
- `y` (FLOAT32)
- `z` (FLOAT32)
- `cluster_id` (INT32)  <-- tracker 노드에서 이 이름을 찾습니다
- `rgb` (FLOAT32) : RGB 24bit를 32bit float로 memcpy한 값 (RViz의 `RGB8` 컬러 트랜스포머로 사용)

주요 파라미터 (노드에서 선언된 기본값)
----------------
- `input_topic` (string) : 기본 `/pointcloud/ground_removed`
- `output_topic` (string) : 기본 `/pointcloud/clustered`
- `eps` (double) : 반경 탐색 거리, 기본 `0.5`
- `min_points` (int) : 핵심점 기준 최소 이웃 수, 기본 `5`
- `max_neighbors` (int) : 커널에서 저장할 최대 이웃 수, 기본 `256`
- `max_points` (int) : 프레임당 최대 처리 포인트(성능 제한), 기본 `200000`
- `queue_size` (int) : subscription QoS 큐 사이즈, 기본 `2`

설정 파일 및 런치
- 파라미터 YAML: `config/dbscan_params.yaml` (런치파일에서 로드 가능)
- 런치파일: `launch/dbscan_clustering.launch.py` — 런치 시 YAML을 로드해 노드를 시작합니다.

빌드 및 실행 방법
----------------
워크스페이스 루트에서:
```bash
colcon build --packages-select dbscan_clustering
source install/setup.bash
# 런치 파일 사용
ros2 launch dbscan_clustering dbscan_clustering.launch.py
```

직접 노드 실행(간단 테스트):
```bash
ros2 run dbscan_clustering dbscan_clustering_node  # (노드 이름/실행 파일이 다를 경우 경로에 맞춰 실행)
```

메시지 검사 스크립트
- 고수준: `scripts/inspect_clustered.py` — `sensor_msgs_py.point_cloud2.read_points`로 (x,y,z,cluster_id)를 출력
- 로우바이트: `scripts/inspect_raw_hex.py` — `rgb` 필드의 raw hex 값을 출력하여 packing 확인
- 클러스터 카운팅: `scripts/count_clusters.py` — 각 `cluster_id`별 포인트 수 집계

RViz 설정
----------------
PointCloud2 Display에서:
- Color Transformer: `RGB8`
- Value Field: `rgb`

이렇게 설정해야 `rgb` 필드로 퍼블리시한 색상이 RViz에 반영됩니다. 기본값(예: `Intensity`)이면 색상이 모두 동일하게 보일 수 있습니다.

튜닝 방법 (실전 팁)
----------------
- eps 조정: 라이다 스캔의 단위(미터)에 맞춰 결정하세요. 차량/사람 수준의 클러스터를 잡으려면 보통 0.2~1.0 m 범위에서 실험합니다.
- min_points: 밀도 요구치. 희박한 객체(예: 사람)는 작은 값, 대형 구조물은 큰 값 권장.
- max_neighbors: GPU 메모리/성능과 관련. 매우 조밀한 장면에서는 이 값을 늘리면 더 정확하지만 메모리/시간 증가.
- max_points: 프레임당 처리 한계. 실시간 요구가 클 때는 낮춰서 subsampling 하세요.
- NaN/Inf: 입력에 NaN/Inf가 많으면 node는 유효 포인트만 퍼블리시합니다 — 트래커가 원본 인덱스를 필요로 한다면(또는 원본 길이를 보존해야 하면) `orig_index` 같은 추가 필드를 퍼블리시하도록 코드를 확장해야 합니다.
- 색상 균일 문제: RViz를 RGB8로 설정했음에도 한 색(예: 빨간색)만 보이면 두 가지 원인 가능:
  1) 실제 클러스터 분포가 편향되어 하나의 cluster_id가 대부분의 포인트를 차지
  2) 색상 팔레트(id→rgb 매핑)가 충돌하거나 너무 유사한 색을 생성
  해결: `scripts/count_clusters.py`로 분포를 확인하고, 필요하면 `src/dbscan_clustering_node.cpp`의 `id_to_color()` 팔레트 함수를 더 넓은 색상 분포로 변경하세요.

트래커(ultralytics_ros)와의 연동 주의사항
----------------
- `tracker_with_cloud_node.cpp`는 입력 PointCloud2에서 `cluster_id` (int32) 필드를 찾습니다. 본 패키지에서 출력되는 필드명이 `cluster_id`로 맞춰져 있으므로 기본적으로 읽을 수 있습니다.
- 현재 DBSCAN 노드는 NaN/Inf를 제거한 유효 포인트만 퍼블리시합니다(즉, 퍼블리시된 cloud의 width는 원래 입력보다 작아짐). 트래커 쪽에서 원본 인덱스를 기대하거나 원본 크기를 유지해야 한다면 아래 중 하나를 적용하세요:
  - 트래커를 compact된 cloud로 사용하도록 변경(권장: 간단)
  - DBSCAN 노드에 `orig_index`(int32) 필드를 추가해 각 퍼블리시된 포인트가 원래 입력에서 어떤 인덱스였는지 전달
  - 또는 full-size output을 유지하면서 noise에 대해 `cluster_id=-1`로 채워서 퍼블리시

권장 후속 개선
----------------
- `orig_index` 필드 추가: 트래커가 원본 인덱스에 의존한다면 간단히 추가하면 호환성 문제가 사라집니다.
- Full-cloud 모드: 원본 인덱스와 동일한 포인트 수를 유지하면서 `cluster_id`를 원래 인덱스 위치에 채워넣는 옵션을 추가
- 더 나은 색 팔레트: 많은 서로 다른 클러스터를 식별하기 위한 색상생성 개선(예: 색상 테이블 + 색 충돌 방지)

문제 해결 체크리스트
----------------
1. 토픽이 올바른가? (`input_topic` / `lidar_topic` 일치)
2. 출력 PointCloud2에 `cluster_id`가 있는가? (`inspect_raw_hex.py`로 확인)
3. RViz에서 Color Transformer가 `RGB8`인지 확인
4. 클러스터 분포가 한 쪽으로 치우쳐 있지 않은가? (`count_clusters.py`)

문의 및 기여
----------------
문제가 있거나 tracker 연동을 위해 `orig_index` 추가 등을 원하시면 요청해 주세요 — 제가 코드(그리고 간단한 테스트)를 직접 추가해 드리겠습니다.

---
작업 폴더: `dbscan_clustering/`
