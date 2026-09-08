# Perception source scope

Target: **Ubuntu 22.04 / ROS2 Humble**. CUDA toolkit and GPU architecture must match the deployment machine.

## Included editable packages

| Package | Purpose |
| --- | --- |
| `dbscan_clustering` | Point-cloud ROI/voxel processing, CUDA neighbor search and DBSCAN clustering |
| `obstacle_filtering_real` | Waypoint/TF-aware obstacle filtering |
| `lshape_fitting` | Shape fitting, detections and Detection2D interfaces |
| `obstacle_tracking` | Multi-object tracking, Track2D interfaces and existing analysis/visualization scripts |
| `erp42_racing_perception` | Launch integration for the perception pipeline |

C++/CUDA, package messages, launch files, configuration and package-local inspection tools are included. `launch/perception_replay.launch.py` is a separate offline adapter with simulated time and an explicit waypoint CSV path. Algorithm sources remain identical to the extracted initial snapshot; file hashes are in `source_snapshot.json`.

## Actual source pipeline and interfaces

```text
/velodyne_points (PointCloud2)
  -> external Patchwork++ -> /patchworkpp/nonground
  -> CUDA DBSCAN -> /pointcloud/clustered
  -> waypoint/TF filtering -> /obstacle_in
  -> L-shape -> /target/detections (lshape_fitting/Detection2DArray)
  -> tracking -> /perception/obstacles (obstacle_tracking/Track2DArray)
```

Recorded TF must provide consistent `map -> base_link -> velodyne` transforms. `/vel` is `geometry_msgs/msg/TwistWithCovarianceStamped`; the tracker uses it for initial-speed branching and diagnostics. `/localization/ego_state` is an optional evaluation input, not a tracker subscription. Provide a waypoint CSV matching the bag's map origin through `waypoint_csv_path`; the empty original default otherwise looks for a planning-package resource that is not included here.

The active config disables `/tracked/objects`; use `/perception/obstacles` in existing analysis scripts. Track `map_pose/map_twist` are map-frame values; `pose/twist` are base-frame values even though the array header says map. Velocity is an absolute velocity expressed in the respective axes, not automatically ego-relative velocity. TF-aligned output stamps are not measured processing latency. Existing README risk-estimation ideas do not establish a validated TTC output in this source snapshot.

## Dependencies and build status

Provide the external `patchworkpp` ROS package with the intended local modifications/calibration. Its ROS source depends on the sibling `cpp/` directory; copying `ros/` alone is insufficient. Upstream is credited in `../THIRD_PARTY_NOTICES.md`. A reproducible public upstream pin is still pending, so this repository alone is not a complete end-to-end baseline environment.

In a prepared Humble/CUDA shell, after building/sourcing the compatible external Patchwork++ underlay:

```bash
source /opt/ros/humble/setup.bash
# Source the verified Patchwork++ underlay as well.
colcon build --base-paths src --packages-up-to erp42_racing_perception --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Full ROS/CUDA package build has **not been verified** in the Agent execution environment. Known initial-source packaging issues: `lshape_fitting` uses `tf2_ros` without declaring/finding/linking it; filtering omits the `ament_index_cpp` manifest dependency; some bag-analysis scripts need `rosbag2_py`. These are recorded rather than silently changed during this source-only export. No CPU DBSCAN replacement or algorithm tuning was performed.

The initial source configurations also need runtime verification of simulated time, TF freshness and QoS. The separate replay adapter has not been run against ROS. No rosbag, GT, timing collector or measured baseline results are included; data is pending. The upstream C++ core and waypoint object compiled separately, which is not a ROS pipeline build.

## Excluded

Localization/GNSS/Velodyne driver source, planning/control/serial/simulator code, waypoint CSV, bag/GT data, private research logs, whole-team audit documents and build outputs are excluded. This public source branch contains only the perception source and the small amount of documentation needed to use and attribute it.
