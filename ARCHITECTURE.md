# LIORF Architecture Notes

## Frame architecture (detailed)

This section describes the runtime frame model used by the current floating-anchor GPS integration.
It follows REP-105 frame intent ([map/odom/base_link](https://www.ros.org/reps/rep-0105.html#map)) while splitting responsibilities across `mapFrameLocal` and `odometryFrame`.

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
- Local motion/output: `mapFrameLocal -> odometryFrame -> baselinkFrame` (+ compatibility branch `odometryFrame -> lidar_link`)

### Who publishes what

- `mapOptimization` publishes:
  - `ECEFframe -> mapFrameEnu` from GPS datum projection,
  - `mapFrameEnu -> mapFrameLocal` from optimized floating-anchor state $T_{G_L}$,
  - `mapFrameLocal -> odometryFrame` from local-vs-incremental odometry alignment,
  - `odometryFrame -> "lidar_link"` and `odometryFrame -> baselinkFrame` as robot pose outputs.

When `lidarFrame != baselinkFrame`, `mapOptimization` derives `odometryFrame -> baselinkFrame` by applying the inverse of the looked-up `lidarFrame -> baselinkFrame` transform to the computed base pose so TF directionality remains parent-to-child and tree-consistent.
- `imuPreintegration` / `TransformFusion` continue producing incremental and fused odometry on existing topics; this does not replace TF ownership above.

### Publication behavior and gating

- No placeholder global transform is published before valid GPS initialization.
- After initialization, the latest valid global transforms continue to be published even if GPS is temporarily unavailable.

### Topic/frame conventions

For backward compatibility, key map products are still published in `odometryFrame` (trajectory cloud/path/registered map clouds), while global semantics are represented through the additional upstream transforms (`ECEFframe`, `mapFrameEnu`, `mapFrameLocal`).

At the current stage, `mapFrameLocal` and `odometryFrame` are effectively the same in nominal operation (identity-initialized and typically near-identity), but they remain separated in the model to preserve future extensibility.

### Configuration parameters

The active frame parameters are defined in [include/utility.h](include/utility.h):

- `odometryFrame`
- `mapFrameLocal`
- `mapFrameEnu`
- `ECEFframe`

Default values are set in dataset YAMLs under [config](config).
