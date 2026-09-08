# ERP-LiDAR_Tracking

**ROS2 Humble-based 3D LiDAR multi-object tracking for autonomous racing with the ERP42 platform.**

This project was developed for autonomous racing at approximately **30 km/h** using a **32-channel 3D LiDAR**. The system detects and tracks opponent vehicles and obstacles, estimates their relative position and velocity, and computes collision-risk metrics such as **Time-to-Collision (TTC)** using ego-vehicle localization.

## Current Pipeline

```text
32-ch 3D LiDAR
      ↓
ROI Filtering
      ↓
Patchwork++ Ground Removal
      ↓
DBSCAN Clustering
      ↓
Vehicle / Obstacle Filtering
      ↓
Kalman Filter Tracking
      ↓
Mahalanobis Gating
      ↓
Hungarian Assignment
      ↓
Relative Position / Velocity / TTC
```

## Key Features

- ROS2 Humble-based real-time perception pipeline
- 32-channel 3D LiDAR processing for autonomous racing
- Ground removal using **Patchwork++**
- DBSCAN-based obstacle clustering
- Vehicle-aware cluster filtering
- Kalman Filter-based multi-object tracking
- **Mahalanobis-distance gating** for uncertainty-aware association
- **Hungarian assignment** for global one-to-one matching
- Relative distance and velocity estimation using ego-vehicle localization
- Time-to-Collision (TTC) estimation

## Motivation

High-speed racing environments introduce several perception challenges. LiDAR point density decreases with distance, ground removal can partially remove distant vehicles on uneven terrain, and missed detections or ambiguous associations can destabilize velocity estimates.

The goal of this project is to build a robust and efficient racing perception pipeline that goes beyond frame-by-frame detection by incorporating temporal tracking, vehicle geometry, track information, and motion priors.

## Related Works

This project is being extended with reference to several open-source and real-world autonomous-racing tracking systems.

| Work | Key Idea | Relevance to This Project |
|---|---|---|
| [Shin et al., 2025 — Multi-Vehicle ARRC](https://doi.org/10.1016/j.jestch.2025.102164) | Multi-vehicle racing with 16-channel LiDAR, EKF-based orientation compensation, and Frenet-frame planning | Reference for robust vehicle orientation estimation and track-aware racing perception on an ERP42-class platform |
| [DATMO](https://github.com/kostaskonkk/datmo) | Adaptive Breakpoint clustering, rectangle/L-shape fitting, CV and coordinated-turn tracking | Reference for range-adaptive clustering and vehicle geometry estimation; concepts will be adapted from 2D ROS to 3D ROS2 |
| [F1TENTH ICRA Race Stack](https://github.com/vaithak/f1tenth-icra-race) | Adaptive Breakpoint obstacle clustering, bounding-box extraction, EKF velocity tracking, and real-time C++ implementation | Reference for replacing fixed DBSCAN parameters with lightweight range-adaptive clustering |
| [TUMFTM FusionTracking](https://github.com/TUMFTM/FusionTracking) | CTRV-EKF object tracking, multi-modal fusion, and perception-delay compensation for autonomous racing | Reference for motion-model-based prediction, CTRV tracking, and future latency compensation |

Rather than directly combining all of these methods, this repository will evaluate each component incrementally against the current **DBSCAN + CV Kalman Filter + Mahalanobis/Hungarian** baseline using the same racing data.

## Roadmap

The current system will be extended step by step while keeping the existing tracker as a baseline.

### Perception

- [ ] Waypoint / track-aware dynamic ROI
- [ ] Range-adaptive clustering to replace fixed-parameter DBSCAN
- [ ] L-shape / Oriented Bounding Box (OBB) vehicle fitting
- [ ] Stable heading estimation using shape, temporal motion, and track-direction priors

### Tracking

- [ ] Compare Constant Velocity (CV) and CTRV motion models
- [ ] CTRV-EKF tracking with stabilized heading estimates
- [ ] CV + CTRV **Interacting Multiple Model (IMM)** tracking
- [ ] Evaluate JPDA for closely spaced or ambiguous vehicle detections

### Track-aware Risk Estimation

- [ ] Frenet-coordinate representation `(s, d)`
- [ ] Track-relative distance and velocity estimation
- [ ] TTC estimation along the racing direction
- [ ] Cut-in / lane-intrusion risk estimation

## Target Pipeline

```text
32-ch 3D LiDAR
      ↓
Waypoint-aware ROI
      ↓
Patchwork++
      ↓
Range-adaptive Clustering
      ↓
L-shape / OBB
      ↓
Heading Estimation
(shape + motion + track prior)
      ↓
CV / CTRV IMM
      ↓
Mahalanobis Gating
      ↓
Hungarian / JPDA
      ↓
Frenet Representation
      ↓
Relative Position / Velocity / TTC / Cut-in Risk
```

## Environment

- **OS / Middleware:** Ubuntu 22.04 + ROS2 Humble
- **Platform:** ERP42 Racing
- **Sensor:** 32-channel 3D LiDAR
- **Language:** C++

## Development Strategy

Each new component will be evaluated against the existing baseline using the same recorded ROS2 bag data. Planned evaluation metrics include runtime, detection stability, ID switches, velocity estimation error, heading stability, and TTC consistency.

## Published Perception Source

The editable perception packages are now under [`src/`](src/): CUDA DBSCAN clustering, waypoint-aware obstacle filtering, L-shape fitting, obstacle tracking, and the perception launch wrapper. Their messages, configurations, launch files and existing package-local analysis tools are included.

See [the source scope and build limitations](docs/PERCEPTION_SCOPE.md) for the actual topic/frame contracts and dependency requirements. Patchwork++ is used as an [attributed external dependency](THIRD_PARTY_NOTICES.md). The team localization/planning/control stack, private research records and datasets are not included.

**Source snapshot status:** complete ROS/CUDA build and rosbag baseline measurements are pending. The current snapshot already contains L-shape fitting; the roadmap above describes further development. Do not interpret the introductory risk-estimation description as validation of TTC or relative-velocity outputs in this initial source snapshot.
