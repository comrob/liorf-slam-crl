# AGENTS.md

This file is a working guide for future LLM/code agents modifying this repository.

## 1) Project purpose

`lili` is the ROS 2 Jazzy package for **LILI-SAM**, a LiDAR–IMU(-GPS) SLAM system in the LIORF/LIO-SAM lineage, with three core C++ executables:

- `lili_imageProjection`
- `lili_imuPreintegration`
- `lili_mapOptimization`

The runtime architecture is topic-driven and split across front-end deskewing, IMU preintegration, and map optimization.

---

## 2) Repository structure and what is defined where

### Build/package metadata

- [CMakeLists.txt](CMakeLists.txt)
  - Defines package build, dependencies, generated interfaces, and three node executables.
  - Builds:
    - `src/imageProjection.cpp`
    - `src/imuPreintegration.cpp`
    - `src/mapOptimization/main.cpp`
    - `src/mapOptimization/mapOptimization_core.cpp`
    - `src/mapOptimization/mapOptimization_map.cpp`
    - `src/mapOptimization/mapOptimization_scan.cpp`
    - `src/mapOptimization/mapOptimization_degeneracy.cpp`
    - `src/mapOptimization/mapOptimization_gps.cpp`
    - `src/mapOptimization/mapOptimization_loop.cpp`
    - `src/mapOptimization/mapOptimization_publish.cpp`
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
    - [config/lili_default.yaml](config/lili_default.yaml)
    - [config/datasets/kitti.yaml](config/datasets/kitti.yaml)
    - [config/lili_ouster.yaml](config/lili_ouster.yaml)
    - [config/datasets/lio_sam_livox.yaml](config/datasets/lio_sam_livox.yaml)
    - [config/datasets/mulran.yaml](config/datasets/mulran.yaml)

### Launch

- [launch/lili.launch.py](launch/lili.launch.py)
  - General launch entry point (params + optional RViz).

- Dataset launchers (same node trio, different default params), e.g.:
  - [launch/run_lili_ouster.launch.py](launch/run_lili_ouster.launch.py)
  - [launch/datasets/run_kitti.launch.py](launch/datasets/run_kitti.launch.py)

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

- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
  - Declares the `mapOptimization` node class and shared state used across split translation units.

- [src/mapOptimization/](src/mapOptimization)
  - Split implementation of `mapOptimization` node (same class, multiple `.cpp` files).
  - File roles:
    - [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp): node constructor, memory setup, LiDAR callback orchestration.
    - [src/mapOptimization/mapOptimization_map.cpp](src/mapOptimization/mapOptimization_map.cpp): local/global map management, map extraction/cache, save-map service hooks.
    - [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp): scan alignment and optimization (`scan2MapOptimization`, LM, pose update).
    - [src/mapOptimization/mapOptimization_degeneracy.cpp](src/mapOptimization/mapOptimization_degeneracy.cpp): perturbation-based degeneracy detection orchestration and complementary odometry compensation (input handler, extrinsics resolution, state override).
    - [src/mapOptimization/mapOptimization_gps.cpp](src/mapOptimization/mapOptimization_gps.cpp): datum init, GPS fusion factors, GPS outputs.
    - [src/mapOptimization/mapOptimization_loop.cpp](src/mapOptimization/mapOptimization_loop.cpp): RS/SC loop closure and loop visualization.
    - [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp): TF/odometry/frame publication + geometry helpers.
    - [src/mapOptimization/main.cpp](src/mapOptimization/main.cpp): executable entry point.

### Loop-closure support and utilities

- [include/Scancontext.h](include/Scancontext.h) and [include/Scancontext.cpp](include/Scancontext.cpp)
  - Scan Context manager implementation used by `mapOptimization` (loop closure path in [src/mapOptimization/mapOptimization_loop.cpp](src/mapOptimization/mapOptimization_loop.cpp)).

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
     - `lili/deskew/cloud_deskewed`
     - `lili/deskew/cloud_info`

3. `mapOptimization` ([src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp), [src/mapOptimization/mapOptimization_map.cpp](src/mapOptimization/mapOptimization_map.cpp), [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp), [src/mapOptimization/mapOptimization_gps.cpp](src/mapOptimization/mapOptimization_gps.cpp), [src/mapOptimization/mapOptimization_loop.cpp](src/mapOptimization/mapOptimization_loop.cpp), [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp))
   - Subscribes:
     - `lili/deskew/cloud_info`
     - GPS (`gpsTopic`)
     - optional loop topic `lio_loop/loop_closure_detection`
   - Publishes (key ones):
     - `lili/mapping/odometry`
     - `lili/mapping/odometry_incremental`
     - `lili/mapping/path`
     - map/trajectory clouds
     - `lili/mapping/gps_odom`
     - `lili/gps_origin`

4. `IMUPreintegration` ([src/imuPreintegration.cpp](src/imuPreintegration.cpp))
   - Subscribes:
     - IMU raw
     - `lili/mapping/odometry_incremental`
   - Publishes:
     - `odomTopic + "_incremental"`

5. `TransformFusion` ([src/imuPreintegration.cpp](src/imuPreintegration.cpp))
   - Subscribes:
     - `lili/mapping/odometry`
     - `odomTopic + "_incremental"`
   - Publishes:
     - final/fused `odomTopic`
     - `lili/imu/path`

### Feedback loop

`mapOptimization` -> `odometry_incremental` -> `IMUPreintegration` -> `odomTopic_incremental` -> `ImageProjection` deskew assistance.

This loop is intentional and is the first place to inspect when timing or drift behavior changes.

---

## 4) Where to edit for common tasks

Update this file when new, relevant, and non-obvious insights are discovered.
Ask the user for consent before adding such insights.

### Add/change parameters

1. Add in `ParamServer` declarations/loading in [include/utility.h](include/utility.h).
2. Set defaults in relevant YAML under [config/](config).
3. Verify launch file uses expected YAML.

### Change subscribed/published topics

- `ImageProjection`: [src/imageProjection.cpp](src/imageProjection.cpp)
- `IMUPreintegration` / `TransformFusion`: [src/imuPreintegration.cpp](src/imuPreintegration.cpp)
- `mapOptimization`: [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp), [src/mapOptimization/mapOptimization_map.cpp](src/mapOptimization/mapOptimization_map.cpp), [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp), [src/mapOptimization/mapOptimization_gps.cpp](src/mapOptimization/mapOptimization_gps.cpp), [src/mapOptimization/mapOptimization_loop.cpp](src/mapOptimization/mapOptimization_loop.cpp), [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)

### Adjust deskewing/front-end filtering

- [src/imageProjection.cpp](src/imageProjection.cpp)
  - `cachePointCloud()`
  - `deskewInfo()`
  - `projectPointCloud()`

### Adjust back-end optimization / loop closure / GPS behavior

- [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp)
- [src/mapOptimization/mapOptimization_map.cpp](src/mapOptimization/mapOptimization_map.cpp)
- [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp)
- [src/mapOptimization/mapOptimization_gps.cpp](src/mapOptimization/mapOptimization_gps.cpp)
- [src/mapOptimization/mapOptimization_loop.cpp](src/mapOptimization/mapOptimization_loop.cpp)
- [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
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

1. Prefer minimal, localized changes, as long as this does not create spaghetti code.
2. Keep functionality in files where it logically belongs, so file ownership remains clear.
3. Keep topic names and frame IDs coherent across YAML + launch + C++.
4. If changing frame transforms/TF ownership, inspect `mapOptimization` TF publication first.
5. Avoid editing cached files under [docker/cache/](docker/cache).
6. If changing architecture-level flow, update this file and changelog in the same change.
7. If a solution seems bloated or hacky instead of an elegant addition, ask for user consent before implementation, explain the concerns, and suggest alternatives.

---

## 6) Mandatory change logging policy

Every repository modification **must** be recorded in [CHANGELOG.md](CHANGELOG.md).

Required for each entry:

- Date (YYYY-MM-DD)
- Short title
- Files changed
- Summary of behavior impact
- Notes for migration/runtime risk (if any)

Session policy (mandatory):

- Use **one changelog entry per development session**.
- If the same feature is continued in a **new session**, create a **new entry** for that new session.
- Within one session, do **not** append multiple micro-entries; keep editing the session's top entry to reflect the current final state of that session.
- Prefer concise, deduplicated summaries over raw iterative history.

Use newest-first order (latest entry at top).

---

## 7) Quick run/build pointers

- Build/package config: [CMakeLists.txt](CMakeLists.txt), [package.xml](package.xml)
- Main launch: [launch/lili.launch.py](launch/lili.launch.py)
- Typical default launch: [launch/run_lili_ouster.launch.py](launch/run_lili_ouster.launch.py)

---

## 8) Current working context (priority)

For the current development context, treat the following as the primary runtime entrypoint and parameter set:

- Primary launch file: [launch/lili.launch.py](launch/lili.launch.py)
- Primary config file: [config/lili_ouster.yaml](config/lili_ouster.yaml)

When relevant parameters are changed, also update [config/anymal.yaml](config/anymal.yaml) if it is in scope for the user's task.

When making iterative changes, prefer validating behavior against this launch/config pair first unless the task explicitly targets another dataset/sensor profile.

---

## 9) Commit message guidelines (mandatory) - only if asked to commit

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
