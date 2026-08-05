# LILI-SAM Architecture Notes

## mapOptimization implementation layout

`mapOptimization` is implemented as a single class split across multiple translation units:

- Declarations/state: [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- Constructor and LiDAR callback orchestration: [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp)
- Map management/representation: [src/mapOptimization/mapOptimization_map.cpp](src/mapOptimization/mapOptimization_map.cpp)
- Scan alignment/optimization: [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp)
- GPS fusion: [src/mapOptimization/mapOptimization_gps.cpp](src/mapOptimization/mapOptimization_gps.cpp)
- Loop closure: [src/mapOptimization/mapOptimization_loop.cpp](src/mapOptimization/mapOptimization_loop.cpp)
- TF/odometry publishing: [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
- Entry point: [src/mapOptimization/main.cpp](src/mapOptimization/main.cpp)

## Frame architecture (detailed)

This section describes the runtime frame model used by the current floating-anchor GPS integration.
It follows REP-105 frame intent ([map/odom/base_link](https://www.ros.org/reps/rep-0105.html#map)) while splitting responsibilities across `mapFrameLocal` and `odometryFrame`.

### TLDR frame tree (active Ouster profile examples)

```text
----------------- gps-enabled frames
ECEFframe      (e.g., "earth")
└── mapFrameEnu    (e.g., "map_enu")
  ├── mapFrameNed   (e.g., "map_ned")
------------------ ¡always-active local frames!
  └── mapFrameLocal (e.g., "map")
    └── odometryFrame (e.g., "odom")
      ├── "lidar_link"   (compatibility branch)
      └── baselinkFrame (e.g., "os_sensor")
---------------- if lidarFrame != baselinkFrame:
        └── ... TF chain ...
          └── lidarFrame (e.g., "os_lidar")
```

### Frame roles

- `ECEFframe` (default `earth`): global Earth-Centered Earth-Fixed frame, used only when GPS datum is available.
- `mapFrameEnu` (default `map`): local ENU tangent frame at the first accepted GPS datum.
- `mapFrameLocal` (default `map_local`): SLAM local optimization frame (the frame in which the internal lidar mapping trajectory is estimated).
- `odometryFrame` (default `odom`): convenience odometry/output frame used by odometry and map products consumed by existing downstream tools.
- `baselinkFrame` / `lidarFrame`: robot body and sensor frames.

### Transform chain

When initialized, the system publishes this chain:

`ECEFframe -> mapFrameEnu -> mapFrameLocal -> odometryFrame -> baselinkFrame` (and `odometryFrame -> "lidar_link"` for compatibility).

Two-layer view:

- Global/georeferencing: `ECEFframe -> mapFrameEnu -> mapFrameLocal`
- Local motion/output: `mapFrameLocal -> odometryFrame -> baselinkFrame` (+ compatibility branch `odometryFrame -> "lidar_link"`)

### Frame hierarchy diagram

```text
ECEFframe (e.g., "earth")
 |
 +-- mapFrameEnu (e.g., "map")
 |    |
 |    +-- mapFrameNed
 |    |
 |    +-- [optional branch when GPS anchor is ready]
 |    |    mapFrameLocal (e.g., "map_local")
 |         |
 |         +== Drift absorber joint: mapFrameLocal -> odometryFrame
 |             - absorbs graph-level global corrections
 |               (loop closure and GPS anchor updates)
 |             - may change discontinuously
 |         |
 |         +-- odometryFrame (e.g., "odom")
 |              |
 |              +== Continuous joints (normal operation)
 |              |   - odometryFrame -> "lidar_link"
 |              |   - odometryFrame -> baselinkFrame
 |              |   - driven by incremental LiDAR odometry
 |              |
 |              +-- "lidar_link"  [compatibility output branch]
 |              |
 |              +-- baselinkFrame (e.g., "base_link")
 |                   |
 |                   +-- ... unknown/intermediate robot TF elements ...
 |                   |
 |                   +-- lidarFrame (e.g., "os_lidar")
```

Notes:

- `"lidar_link"` in this node is a compatibility child frame emitted directly from `odometryFrame`.
- `lidarFrame` is the sensor frame from the robot TF tree and may be connected through intermediate links under `baselinkFrame`.
- They are semantically different frame names even when their resulting pose can coincide in a particular setup.

### Who publishes what

- `mapOptimization` publishes:
  - `ECEFframe -> mapFrameEnu` from GPS datum projection,
  - `mapFrameEnu -> mapFrameLocal` from optimized floating-anchor state $T_{G_L}$,
  - `mapFrameLocal -> odometryFrame` from local-vs-incremental odometry alignment,
  - `odometryFrame -> "lidar_link"` and `odometryFrame -> baselinkFrame` as smooth robot pose outputs,
  - LiDAR odometry topics:
    - `lili/mapping/odometry` in `mapFrameLocal -> "lidar_link"`
    - `lili/mapping/odometry_incremental` in `odometryFrame -> "lidar_link"`
  - base-link odometry topics:
    - `lili/mapping/baselink_odometry` in `mapFrameLocal -> baselinkFrame`
    - `lili/mapping/baselink_odometry_incremental` in `odometryFrame -> baselinkFrame`
  - GPS-fused base-link odometry topics:
    - `lili/mapping/baselink_gps_enu_odometry` in `mapFrameEnu -> baselinkFrame`
    - `lili/mapping/baselink_gps_ned_odometry` in `mapFrameNed -> baselinkFrame`

When `lidarFrame != baselinkFrame`, `mapOptimization` derives `odometryFrame -> baselinkFrame` by applying the inverse of the looked-up `lidarFrame -> baselinkFrame` transform to the computed base pose so TF directionality remains parent-to-child and tree-consistent.
- `imuPreintegration` / `TransformFusion` continue producing incremental and fused odometry on existing topics; this does not replace TF ownership above.

### Publication behavior and gating

- No placeholder global transform is published before valid GPS initialization.
- After initialization, the latest valid global transforms continue to be published even if GPS is temporarily unavailable.

### Topic/frame conventions

Legacy LiDAR odometry topics are intentionally split across the two local layers:

- `lili/mapping/odometry_incremental` carries smooth LiDAR motion in `odometryFrame` and keeps `child_frame_id = "lidar_link"` for IMU-preintegration compatibility.
- `lili/mapping/odometry` carries graph-optimized LiDAR pose in `mapFrameLocal` and keeps `child_frame_id = "lidar_link"`.

Map-local map products derived from the optimized trajectory (trajectory cloud/path/registered map clouds) are published in `mapFrameLocal`, while smooth robot motion is represented through the `odometryFrame -> ...` TF and incremental odometry topics.

At the current stage, `mapFrameLocal` and `odometryFrame` are effectively the same in nominal operation (identity-initialized and typically near-identity), but they remain separated in the model to preserve future extensibility.

### Configuration parameters

The active frame parameters are defined in [include/utility.h](include/utility.h):

- `odometryFrame`
- `mapFrameLocal`
- `mapFrameEnu`
- `ECEFframe`

Default values are set in dataset YAMLs under [config](config).
