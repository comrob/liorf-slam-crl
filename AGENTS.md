# AGENTS.md

This file is a working guide for future LLM/code agents modifying this repository.

## 1) Project purpose

`liorf` is a ROS 2 Jazzy LiDAR–IMU(-GPS) SLAM package (LIORF/LIO-SAM style) with three core C++ executables:

- `liorf_imageProjection`
- `liorf_imuPreintegration`
- `liorf_mapOptmization`

The runtime architecture is topic-driven and split across front-end deskewing, IMU preintegration, and map optimization.

---

## 2) Repository structure and what is defined where

### Build/package metadata

- [CMakeLists.txt](CMakeLists.txt)
  - Defines package build, dependencies, generated interfaces, and three node executables.
  - Builds:
    - `src/imageProjection.cpp`
    - `src/imuPreintegration.cpp`
    - `src/mapOptmization.cpp`
  - Generates ROS interfaces:
    - [msg/CloudInfo.msg](msg/CloudInfo.msg)
    - [srv/SaveMap.srv](srv/SaveMap.srv)

- [package.xml](package.xml)
  - ROS package metadata + dependencies.

- [README.md](README.md)
  - Setup/build/run instructions (Ubuntu 24.04 + ROS 2 Jazzy).

### Config

- [config/](config)
  - Per-dataset/per-sensor parameter YAML files.
  - Commonly edited files:
    - [config/lio_sam_default.yaml](config/lio_sam_default.yaml)
    - [config/kitti.yaml](config/kitti.yaml)
    - [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
    - [config/lio_sam_livox.yaml](config/lio_sam_livox.yaml)
    - [config/mulran.yaml](config/mulran.yaml)

### Launch

- [launch/liorf.launch.py](launch/liorf.launch.py)
  - General launch entry point (params + optional RViz).

- Dataset launchers (same node trio, different default params), e.g.:
  - [launch/run_lio_sam_default.launch.py](launch/run_lio_sam_default.launch.py)
  - [launch/run_kitti.launch.py](launch/run_kitti.launch.py)

### Core C++ source files

- [include/utility.h](include/utility.h)
  - Shared definitions used by all nodes:
    - `ParamServer` parameter loader
    - sensor/enum definitions (`SensorType`, `TranslationPredictionSource`)
    - IMU conversion utility `imuConverter()`
    - helper functions (`publishCloud()`, `ROS_TIME()`, QoS builder `QosPolicy()`)
  - Primary location for adding/changing global parameters.

- [src/imageProjection.cpp](src/imageProjection.cpp)
  - Defines `ImageProjection` node.
  - Input: raw LiDAR + IMU + incremental odom.
  - Responsibilities:
    - sensor point-type normalization
    - deskew preparation from IMU/odom
    - point filtering/projection
    - publish deskewed cloud + `CloudInfo`

- [src/imuPreintegration.cpp](src/imuPreintegration.cpp)
  - Defines:
    - `IMUPreintegration` node (GTSAM IMU factor integration / high-rate incremental odometry)
    - `TransformFusion` node (fuses LiDAR mapping odometry with IMU incremental updates)
  - Publishes both incremental odom for deskew loop and fused odom/path.

- [src/mapOptmization.cpp](src/mapOptmization.cpp)
  - Defines `mapOptimization` node.
  - Responsibilities:
    - consumes deskewed cloud + cloud info
    - scan-to-map optimization
    - keyframe management
    - factor graph optimization (GTSAM / ISAM2)
    - loop closure (including Scan Context support)
    - GPS integration path (`NavSatFix` -> local odom)
    - map/trajectory/odometry/TF publication
    - `liorf/save_map` service implementation

### Loop-closure support and utilities

- [include/Scancontext.h](include/Scancontext.h) and [include/Scancontext.cpp](include/Scancontext.cpp)
  - Scan Context manager implementation used by `mapOptmization`.

- [include/KDTreeVectorOfVectorsAdaptor.h](include/KDTreeVectorOfVectorsAdaptor.h)
  - KD-tree adaptor utilities used by Scan Context logic.

- [include/nanoflann.hpp](include/nanoflann.hpp)
  - Header-only nearest-neighbor library dependency.

- [lib/common_lib.h](lib/common_lib.h), [lib/common_lib.cpp](lib/common_lib.cpp)
  - Small shared helper library (distance helpers, package banner output).

### ROS interfaces

- [msg/CloudInfo.msg](msg/CloudInfo.msg)
  - Inter-node data contract carrying deskewed cloud and state hints.

- [srv/SaveMap.srv](srv/SaveMap.srv)
  - Service request/response contract for map export.

### Visualization/assets

- [rviz/mapping.rviz](rviz/mapping.rviz): RViz defaults.

### Container and docs helpers

- [docker/](docker): Dockerfile/compose/entrypoint and helper docs.
- [config/doc/kitti2bag/kitti2bag.py](config/doc/kitti2bag/kitti2bag.py): KITTI conversion helper.

### Non-source build artifacts (do not edit manually)

- [docker/cache/](docker/cache): cached build/install/log artifacts.

---

## 3) Runtime data flow (important)

### Main topic pipeline

1. Raw sensors in:
   - LiDAR: `pointCloudTopic`
   - IMU: `imuTopic`
   - GPS (optional): `gpsTopic`

2. `ImageProjection` ([src/imageProjection.cpp](src/imageProjection.cpp))
   - Subscribes:
     - LiDAR raw cloud
     - IMU
     - `odomTopic + "_incremental"`
   - Publishes:
     - `liorf/deskew/cloud_deskewed`
     - `liorf/deskew/cloud_info`

3. `mapOptimization` ([src/mapOptmization.cpp](src/mapOptmization.cpp))
   - Subscribes:
     - `liorf/deskew/cloud_info`
     - GPS (`gpsTopic`)
     - optional loop topic `lio_loop/loop_closure_detection`
   - Publishes (key ones):
     - `liorf/mapping/odometry`
     - `liorf/mapping/odometry_incremental`
     - `liorf/mapping/path`
     - map/trajectory clouds
     - `liorf/mapping/gps_odom`
     - `liorf/gps_origin`

4. `IMUPreintegration` ([src/imuPreintegration.cpp](src/imuPreintegration.cpp))
   - Subscribes:
     - IMU raw
     - `liorf/mapping/odometry_incremental`
   - Publishes:
     - `odomTopic + "_incremental"`

5. `TransformFusion` ([src/imuPreintegration.cpp](src/imuPreintegration.cpp))
   - Subscribes:
     - `liorf/mapping/odometry`
     - `odomTopic + "_incremental"`
   - Publishes:
     - final/fused `odomTopic`
     - `liorf/imu/path`

### Feedback loop

`mapOptimization` -> `odometry_incremental` -> `IMUPreintegration` -> `odomTopic_incremental` -> `ImageProjection` deskew assistance.

This loop is intentional and is the first place to inspect when timing or drift behavior changes.

---

## 4) Where to edit for common tasks

### Add/change parameters

1. Add in `ParamServer` declarations/loading in [include/utility.h](include/utility.h).
2. Set defaults in relevant YAML under [config/](config).
3. Verify launch file uses expected YAML.

### Change subscribed/published topics

- `ImageProjection`: [src/imageProjection.cpp](src/imageProjection.cpp)
- `IMUPreintegration` / `TransformFusion`: [src/imuPreintegration.cpp](src/imuPreintegration.cpp)
- `mapOptimization`: [src/mapOptmization.cpp](src/mapOptmization.cpp)

### Adjust deskewing/front-end filtering

- [src/imageProjection.cpp](src/imageProjection.cpp)
  - `cachePointCloud()`
  - `deskewInfo()`
  - `projectPointCloud()`

### Adjust back-end optimization / loop closure / GPS behavior

- [src/mapOptmization.cpp](src/mapOptmization.cpp)
  - scan-to-map optimization functions
  - keyframe/factor-graph update logic
  - `gpsHandler()` and GPS gating behavior
  - loop closure threads and scan-context integration

### Adjust IMU propagation and fusion behavior

- [src/imuPreintegration.cpp](src/imuPreintegration.cpp)
  - `IMUPreintegration::imuHandler()` and `odometryHandler()`
  - `TransformFusion::imuOdometryHandler()`

### Add/modify message/service fields

1. Edit [msg/CloudInfo.msg](msg/CloudInfo.msg) or [srv/SaveMap.srv](srv/SaveMap.srv).
2. Update producers/consumers in C++.
3. Rebuild interfaces and dependent targets.

---

## 5) Operational guidance for future agents

1. Prefer minimal, localized changes.
2. Keep topic names and frame IDs coherent across YAML + launch + C++.
3. If changing frame transforms/TF ownership, inspect `mapOptimization` TF publication first.
4. Avoid editing cached files under [docker/cache/](docker/cache).
5. If changing architecture-level flow, update this file and changelog in the same change.

---

## 6) Mandatory change logging policy

Every repository modification **must** be recorded in [CHANGELOG.md](CHANGELOG.md).

Required for each entry:

- Date (YYYY-MM-DD)
- Short title
- Files changed
- Summary of behavior impact
- Notes for migration/runtime risk (if any)

For the active development cycle, do **not** append a new entry for every small iteration.
Instead, keep updating the current top entry in [CHANGELOG.md](CHANGELOG.md) until a clear milestone/release boundary is reached.

Use newest-first order (latest entry at top).

---

## 7) Quick run/build pointers

- Build/package config: [CMakeLists.txt](CMakeLists.txt), [package.xml](package.xml)
- Main launch: [launch/liorf.launch.py](launch/liorf.launch.py)
- Typical default launch: [launch/run_lio_sam_default.launch.py](launch/run_lio_sam_default.launch.py)

---

## 8) Current working context (priority)

For the current development context, treat the following as the primary runtime entrypoint and parameter set:

- Primary launch file: [launch/run_lio_sam_ouster.launch.py](launch/run_lio_sam_ouster.launch.py)
- Primary config file: [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)

When making iterative changes, prefer validating behavior against this launch/config pair first unless the task explicitly targets another dataset/sensor profile.

---

## 9) Commit message guidelines (mandatory)

Use modern, review-friendly commit messages (Conventional-Commit style is preferred).

Required structure:

1. Header line: short, meaningful, and explicit about scope + change.
2. Empty line.
3. Body: concise bullet list focused on what changed, especially algorithmic or architectural deltas.

Required formatting rules:

- Header format (preferred): `<type>(<scope>): <change summary>`
- Header examples:
  - `feat(map-export): write T_enu_local orientation as quaternion`
  - `refactor(cmake): isolate map exporter into dedicated library target`
- Keep header concise (target <= 72 chars when possible), use new line if out of space.
- Body bullets must state concrete technical changes, not vague intent.
- Body line length must be <= 72 characters per line.
- Keep bullets concise and scannable.

Commit template:

```text
<type>(<scope>): <short summary>

- <algorithmic/architectural change 1>
  <second line of bullet if needed>
- <algorithmic/architectural change 2>
- <behavior/runtime impact or compatibility note>
```

Notes:

- Do not omit the blank line between header and body.
- Avoid `\n` escape sequences inside a single `git commit -m` string.
- Prefer `git commit -F <file>` or multiple `-m` flags (header/body)
  to preserve real newlines and bullet formatting.
- Prefer imperative phrasing in header (`add`, `refactor`, `fix`, etc.).
- If no algorithm/architecture changes were made, state that explicitly in one bullet.
- If the change is small, only header can be enough.
