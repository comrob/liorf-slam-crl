# CHANGELOG

All notable changes to this repository should be documented in this file.

## Entry format

For every change, add a section with:

- Date: `YYYY-MM-DD`
- Title
- Files changed
- Behavior impact
- Migration/runtime risk notes (if applicable)

Keep entries in reverse chronological order (newest first).

Session rule: keep one entry per development session.
If the same feature continues in a new session, create a new entry for that session.
Within one session, update that session entry in place instead of appending micro-entries.

Scope rule: update this changelog only for changes that affect SLAM runtime
functionality, algorithmic behavior, interfaces, or logging/diagnostics data
production. Do not add entries for visualization-only changes (plots, RViz
layout, marker styling, or similar display-only updates) unless they also
change SLAM/logging functionality.

---

## 2026-07-28 - Rework complementary-odom with unit scale and lagged visualization

### Files changed

- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [src/mapOptimization/mapOptimization_degeneracy.cpp](src/mapOptimization/mapOptimization_degeneracy.cpp)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Removed complementary-odometry scale-estimator state/history and helper
	functions from map-optimization degeneracy handling.
- Complementary odometry translation is always applied with scale `1.0` when
	generating the prediction used for degenerate-direction correction.
- Added a fixed-size LiDAR baseline buffer (size =
	`complementaryOdom.scaleBaselineFrameLag + 1`) and switched baseline
	selection to `oldest` (lagged) and `newest` (current) LiDAR poses for
	visualization only.
- Added explicit size capping for the complementary-odometry message buffer.
- Baseline displacement vectors (LiDAR and complementary odometry) are now
	computed from lagged-to-current local-frame deltas and visualized from the
	lagged LiDAR anchor pose.
- Published lag-pair timing diagnostics for the selected displacement pair on
	the complementary-odom scale debug topic:
	`dt_scale_lidar_interval_s` and `dt_scale_odom_pair_interval_s` now report
	the selected lagged LiDAR and matched additional-odometry intervals.
- Disabled immediate-pair complementary-odom dt debug publishing to avoid
	mixed-interval values on the same debug topic.
- Complementary-odometry lagged displacement debug vectors now publish
	regardless of degeneracy status; only state correction remains degeneracy
	gated.
- Added throttled terminal warnings on complementary-odom match failures that
	report closest timestamp gaps and queue state.
- Restored core degenerate-state correction to use the immediate frame baseline
	(with unit scale), preventing lagged-baseline coupling from affecting SLAM
	state updates.
- Finalized immediate-vs-lagged separation in code flow:
	immediate LiDAR/odom pair drives correction; lagged LiDAR/odom pair drives
	visualization only.
- Immediate-pair matching now uses the literal previous LiDAR timestamp from
	class state (`timeLastProcessing`) when valid, with only dt-based startup
	fallback (`prev = cur - dt_scan`); buffer fallback was removed.
- Fixed ordering in degeneracy override so lag-buffer size cap is defined
	before use while still inflating the lag buffer before potential early
	returns.
- Removed redundant early write of `timeLastProcessing` in the LiDAR callback
	processing branch; only final post-processing update remains.
- Changed lag-buffer insertion order to store only the final effective LiDAR
	pose for the frame (corrected when active, otherwise optimized), instead of
	pre-inserting an optimized pose before correction.
- Added per-step lagged complementary-odometry translational speed publication
	to the scale-debug topic via a dedicated field
	`lagged_complementary_lin_speed_mps`.
- Added lagged orientation-drift compensation for complementary-odometry
	visualization vectors by comparing lagged LiDAR and complementary rotation
	increments and inverse-rotating the complementary translation by the
	estimated drift.
- Added publication of lagged relative yaw drift (degrees) on the
	complementary-odom scale debug topic.
- Added a dedicated uncorrected lagged complementary-displacement arrow to the
	complementary-odom correction-direction marker stream for side-by-side
	comparison against the drift-compensated vector.
- Enabled lagged displacement-vector marker publication regardless of
	degeneracy status (when lagged LiDAR/odom matching succeeds), so LiDAR,
	uncorrected complementary, and drift-compensated complementary vectors are
	always visualized.
- Added lag-window path-arrow debug visualization with per-LiDAR-frame
	matching against complementary odometry and three anchored paths on a
	dedicated topic: LiDAR path, original complementary path, and reconstructed
	forward-only path (LiDAR heading + complementary speed), using signed LiDAR
	`+x/-x` reconstruction direction selected from the actual LiDAR step-motion
	projection.
- Added lagged projected-scale debug publication
	(via `scale_instant_raw`) computed as the ratio between the projected
	LiDAR non-degenerate translation norm and the complementary non-degenerate
	translation norm used in lagged visualization.
- Fixed complementary displacement marker/path mismatch by forcing both
	uncorrected and drift-corrected displacement arrows to use the same lagged
	matched anchor and endpoint pair as the lagged complementary path.
- Updated marker naming so `complementary_odom_original` corresponds to the
	uncorrected vector and `complementary_odom_drift_corrected` corresponds to
	the drift-corrected vector.
- Fixed lagged projection inconsistency by recomputing non-degenerate
	projection vectors from the same lagged path-aligned endpoints used by the
	complementary displacement arrows and path, and publishing
	`scale_instant_raw` from that path-aligned projection when available.
- Updated lagged projection basis to use the latest-pose frame (consistent
	with degeneracy basis definition), then re-express projected vectors on the
	lagged anchor frame for visualization.
- Corrected projected-vector visualization to preserve latest-pose orientation
	and only substitute lagged-pose translation for marker anchoring.
- Scale debug outputs remain published for compatibility and report unit-scale
	behavior (estimator disabled, applied scale fixed to `1.0`).

### Migration/runtime risk notes

- Low-medium. Lagged baseline data is now debug-only; SLAM correction path is
	no longer gated by lag-buffer fill state.

## 2026-07-28 - Add shadow-mode complementary-odom scale estimation

### Files changed

- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [src/mapOptimization/mapOptimization_degeneracy.cpp](src/mapOptimization/mapOptimization_degeneracy.cpp)
- [msg/ComplementaryOdomScaleDebug.msg](msg/ComplementaryOdomScaleDebug.msg)
- [include/liorf_diagnostics.h](include/liorf_diagnostics.h)
- [src/liorf_diagnostics.cpp](src/liorf_diagnostics.cpp)
- [include/utility.h](include/utility.h)
- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
- [config/anymal.yaml](config/anymal.yaml)
- [config/docker_override.yaml](config/docker_override.yaml)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Complementary-odometry scale estimation now runs every frame using the same
	observability gate, even during non-degenerate periods (shadow mode).
- Degenerate compensation and pose override are now mode-gated:
	estimator runs in both modes, but state override is applied only in active
	(degenerate) mode.
- Added explicit mode/status diagnostics for complementary-odom scale:
	`estimator_mode` (`0=shadow`, `1=active`),
	`scale_estimate_updated`, and `scale_applied_to_state` in both online topic
	and CSV logs.
- Legacy scale smoother now updates from newly observable samples only; when
	gate is closed it reuses history/fallback instead of pushing new samples.
- Added fixed (non-adaptive) baseline pairing parameter
	`complementaryOdom.scaleBaselineFrameLag` (clamped to minimum `1`) to
	control how many LiDAR frame intervals back the "previous" timestamp is
	selected for complementary-odometry matching.
- Kept existing observability gate definition and existing scalar smoothing
	(`smoothFromNumDen`) behavior unchanged.
- Implemented LiDAR-first fixed-lag baseline ownership (Option C): LiDAR
	displacement for scale estimation is now always computed from LiDAR poses
	separated by `complementaryOdom.scaleBaselineFrameLag` processed frames, and
	complementary odometry is matched to those LiDAR timestamps.
- Added complementary-odom scale debug field
	`dt_scale_odom_interval_s` to explicitly expose the scale-estimation
	interval that corresponds to the same matched baseline as `dt_odom_s`.
- Added explicit debug/CSV interval telemetry for all three relevant pairs:
	`dt_scale_lidar_interval_s` (LiDAR pair used for scale estimation),
	`dt_scale_odom_pair_interval_s` (odometry pair used for scale estimation),
	and `dt_immediate_odom_pair_interval_s` (immediate odometry pair used for
	scaled projection/correction).

### Migration/runtime risk notes

- Medium-low. Estimation telemetry now appears in non-degenerate periods and
	active compensation behavior is now explicitly separated from estimator mode.
- Consumers of `ComplementaryOdomScaleDebug.msg` should regenerate interfaces
	due to added fields.

## 2026-07-27 - Parameterize complementary-odom fallback scale window

### Files changed

- [include/utility.h](include/utility.h)
- [src/mapOptimization/mapOptimization_degeneracy.cpp](src/mapOptimization/mapOptimization_degeneracy.cpp)
- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
- [config/anymal.yaml](config/anymal.yaml)
- [config/docker_override.yaml](config/docker_override.yaml)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Added new ROS parameter:
	`complementaryOdom.fallbackScaleSmoothingWindowSize` (default `30`,
	clamped to minimum `1`).
- Replaced hardcoded fallback moving-average window in complementary-odometry
	scale fallback smoothing with the new configurable parameter.
- Exposed the new parameter in active runtime configs
	(`lio_sam_ouster.yaml`, `anymal.yaml`, `docker_override.yaml`).

### Migration/runtime risk notes

- Low. Default runtime behavior remains unchanged (`30`) unless users tune the
	new parameter.

## 2026-07-23 - Class-owned complementary-odom scale smoothing + config window

### Files changed

- [include/utility.h](include/utility.h)
- [include/liorf_diagnostics.h](include/liorf_diagnostics.h)
- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [msg/ComplementaryOdomScaleDebug.msg](msg/ComplementaryOdomScaleDebug.msg)
- [src/liorf_diagnostics.cpp](src/liorf_diagnostics.cpp)
- [src/mapOptimization/mapOptimization_degeneracy.cpp](src/mapOptimization/mapOptimization_degeneracy.cpp)
- [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
- [CMakeLists.txt](CMakeLists.txt)
- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
- [config/anymal.yaml](config/anymal.yaml)
- [scripts/plot_complementary_odom_scale_diagnostics.py](scripts/plot_complementary_odom_scale_diagnostics.py)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Replaced function-static complementary-odom scale smoothing queue with
	class-owned smoothing state (`complementaryOdomScaleHistory` + running sum),
	so smoothing lifecycle is explicit and managed by node instance state.
- Added configurable parameter:
	`complementaryOdom.scaleSmoothingWindowSize` (clamped to minimum 1),
	and exposed it in primary runtime configs.
- Scale application now uses the smoothed value when scale estimation is
	enabled, while preserving `1.0` application when disabled.
- Extended complementary-odom scale diagnostics fields with
	`scale_instant`, `scale_smoothed`, and `scale_smoothing_window`.
- Added direct PlotJuggler-friendly CSV emission for complementary odometry
	metrics in run directory:
	- `complementary_odom_scale.csv`
	- `complementary_odom_twist.csv`
- Complementary odometry CSV emission is now independent from event logging;
	rows are written via dedicated diagnostics CSV APIs with a leading monotonic
	`time` column followed by scalar fields only.
- CSV headers now use slash-separated hierarchical names
	(e.g. `complementary_odom_scale/scale_applied`) so PlotJuggler groups
	the series similarly to ROS message subfields.
- Removed backward-compatibility fallback fields for complementary odometry
	CSV extraction (`add_nondeg_m`, `lidar_lin_speed_reproject_mps`) and deleted
	the legacy plotting helper so diagnostics are consumed from direct CSV.
- Suppressed terminal log output for degeneracy detection and complementary
	odom twist diagnostics while preserving their CSV diagnostics emission.
- Expanded complementary-odom CSV schema with explicit LiDAR/odom sample
	timestamps, odom sample span/count metadata, projected and unprojected raw
	scale candidates, and body-frame plus observable non-degenerate displacement
	components for per-frame diagnosability.
- Fixed an Eigen compile error in complementary-odom scale diagnostics by
	replacing a mixed-expression ternary with explicit vector assignment.
- Reworked complementary-odometry CSV logging to pass typed debug structs
	directly to diagnostics writers, removing event-style string serialization
	and key-value parsing from these odometry-specific logging paths.
- Reorganized complementary-odometry CSV column names into deeper grouped
	hierarchies (stamp/alignment/gate/scale/body/magnitude/smoothing/speed/dt)
	to improve PlotJuggler field navigation and readability.
- Added a second complementary-odometry scale buffer that keeps a fixed
	30-sample moving average from observable frames and uses that value as the
	fallback scale in unobservable directions, replacing the previous hardcoded
	`1.0` fallback.
- Added a packed custom debug topic
	`liorf/mapping/complementary_odom/scale_debug`
	(`liorf/msg/ComplementaryOdomScaleDebug`) so the same runtime values logged
	to CSV can also be plotted online in RViz 2D plot / PlotJuggler without
	managing multiple scalar topics.
- Clarified complementary-odom scale semantics across runtime, topic, and CSV
	with explicit three-stage values:
	`scale_instant_raw` (direct estimate, gate-independent),
	`scale_instant_raw_safe` (raw-or-fallback by gate), and
	`scale_smooth` (final applied value).
- Updated `ComplementaryOdomScaleDebug.msg` and complementary-odom CSV columns
	to use the transparent naming flow above, plus explicit
	`scale_instant_raw_clamped`, `scale_fallback_history`,
	`gate_observable`, and `scale_estimation_enabled` fields.
- Reworked complementary-odometry displacement markers to publish seven
	explicit vector/pose arrows from a single base pose:
	complementary-on-nondeg, lidar-on-nondeg, lidar-nondeg projected onto
	complementary, corrected(no-scale), corrected(with-scale), complementary
	original, and complementary scaled.
- Added `complementaryOdom.ignore_dz` (bool, default `false`) to optionally
	force `dz=0` on both LiDAR and complementary-odometry displacement vectors
	during scale determination only (raw/gated/smoothed scale path), while
	keeping the rest of the compensation pipeline unchanged.
- Added `complementaryOdom.smoothFromNumDen` (bool, default `false`) as an
	alternative smoothing mode that aggregates numerator/denominator over the
	smoothing window and computes scale as ratio-of-sums.
- Extended online complementary-odom scale debug topic and CSV with both
	smoothing outputs for A/B comparison:
	`scale_smooth_legacy`, `scale_smooth_ratio`, selected `scale_smooth`, and
	mode flag `smooth_from_numden_enabled`.
- Numerator (`lidar_nondeg_projected_m`) and denominator
	(`complementary_nondeg_m`) were already logged in CSV, so no duplicate
	offline-only columns were added.

### Migration/runtime risk notes

- Low. Default smoothing behavior remains equivalent with window size 20.
- Users can tune smoothing responsiveness via
	`complementaryOdom.scaleSmoothingWindowSize`.

## 2026-07-22 - Enable docker override for make slam and persist .ros logs

### Files changed

- [launch/liorf.launch.py](launch/liorf.launch.py)
- [launch/run_lio_sam_ouster.launch.py](launch/run_lio_sam_ouster.launch.py)
- [docker/mk/dev.mk](docker/mk/dev.mk)
- [docker/docker-compose.yaml](docker/docker-compose.yaml)
- [docker/.env](docker/.env)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Added optional `config_override` launch argument support to the core launch
	path (`run_lio_sam_ouster.launch.py` -> `liorf.launch.py`).
- Updated `make slam` and `make prod` launch commands to pass
	`config_override:=/home/dev/ros2_ws/install/liorf/share/liorf/config/docker_override.yaml`,
	so settings in `config/docker_override.yaml` are now applied in those flows.
- Added bind mounts for host ROS user directory (`HOST_ROS_PATH`, defaulting to
	`/home/seva/.ros` in `.env`) into `/home/dev/.ros` for `liorf_vscode`,
	`liorf_dev`, `liorf_run`, and `run_slam`, so runtime logs/results persist on
	host.

### Migration/runtime risk notes

- Low. Launch behavior is unchanged unless `config_override` is provided.
- `make slam`/`make prod` now intentionally apply docker override parameters;
	if unexpected behavior appears, inspect `config/docker_override.yaml` first.
- If `HOST_ROS_PATH` is invalid or missing, container startup will fail until
	the path is corrected.

## 2026-07-22 - Group log outputs under log.* and add odom TUM trajectory export

### Files changed

- [include/utility.h](include/utility.h)
- [include/liorf_diagnostics.h](include/liorf_diagnostics.h)
- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [src/liorf_diagnostics.cpp](src/liorf_diagnostics.cpp)
- [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp)
- [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
- [config/anymal.yaml](config/anymal.yaml)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Added unified log/output parameter tree under `log.*`:
	- `log.base_dir`, `log.run_suffix`
	- `log.diagnostics.write_files`
	- `log.diagnostics.enable_stats`
	- `log.diagnostics.enable_event`
	- `log.diagnostics.enable_warnings`
	- `log.diagnostics.enable_telemetry`
	- `log.diagnostics.enable_time_deltas`
	- `log.diagnostics.enable_frame_metrics`
	- `log.trajectory.odom.enabled`
- Refactored runtime parameter storage to a single flat `LogOutputConfig`
	structure in `ParamServer` while preserving diagnostics behavior.
- Added optional continuous TUM export of the incremental odometry trajectory
	in odom frame (`odom -> lidar_link`) with format:
	`timestamp tx ty tz qx qy qz qw`.
- TUM export is disabled by default and only active when
	`log.trajectory.odom.enabled` is true.
- When enabled, trajectory output is always written to the current run
	diagnostics directory as `trajectory_odom.tum` (alongside diagnostics files)
	and does not depend on `log.diagnostics.write_files`.
- Trajectory file ownership and formatting are handled in
	`LiorfDiagnostics`; map optimization only forwards odometry samples.
- `LiorfDiagnostics` does not read ROS parameters directly for trajectory
	logging; the enable flag is passed from `ParamServer` via constructor,
	consistent with other diagnostics output flags.
- Replaced the long diagnostics constructor bool-list with explicit
	`DiagnosticsOutputPolicy` and `TrajectoryOutputPolicy` argument structs,
	making the independence of odom-trajectory export from diagnostics-file
	master switch explicit in the API.
- Replaced diagnostics trajectory API input from
	`nav_msgs::msg::Odometry` to a small ROS-free POD (`TumPoseSample`), with
	ROS-message-to-POD conversion performed at mapOptimization publish call site
	to reduce diagnostics compile-time coupling.
- Added legacy fallback support for existing `diagnostics_write_*` keys, with
	a deprecation warning when legacy overrides are detected.

### Migration/runtime risk notes

- Low. Default behavior is unchanged.
- Existing configs should migrate diagnostics settings to `log.diagnostics.*`.
	Legacy diagnostics keys are still accepted as fallback for now.

## 2026-07-21 - Centralize source-first RViz config resolution in launch files

### Files changed

- [launch/rviz_config_resolver.py](launch/rviz_config_resolver.py)
- [launch/liorf.launch.py](launch/liorf.launch.py)
- [launch/anymal.launch.py](launch/anymal.launch.py)
- [launch/run_lio_sam_ouster.launch.py](launch/run_lio_sam_ouster.launch.py)
- [launch/datasets/run_kitti.launch.py](launch/datasets/run_kitti.launch.py)
- [launch/datasets/run_M2DGR.launch.py](launch/datasets/run_M2DGR.launch.py)
- [launch/datasets/run_mulran.launch.py](launch/datasets/run_mulran.launch.py)
- [launch/datasets/run_lio_sam_livox.launch.py](launch/datasets/run_lio_sam_livox.launch.py)
- [launch/datasets/run_lio_sam_identity.launch.py](launch/datasets/run_lio_sam_identity.launch.py)
- [launch/datasets/run_ubran_hongkong.launch.py](launch/datasets/run_ubran_hongkong.launch.py)
- [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Added a shared launch helper (`default_rviz_config_path`) and removed duplicated
	per-file RViz default-path logic.
- Default `rviz_config` resolution now consistently prefers source-tree
	`rviz/mapping.rviz` when discoverable (via `LIORF_SOURCE_DIR`, current working
	directory ancestry, or launch-file-relative path), and falls back to the
	installed share path otherwise.
- Behavior is now consistent across the main launch file and all dataset/wrapper
	launch entrypoints.
- Added explicit `sys.path` setup in top-level launch entrypoints so installed
	launches can reliably import the shared resolver module
	(`rviz_config_resolver`) without `ModuleNotFoundError`.
- KD-tree plane-normal debug arrows are now anchored at the projected point on
	the fitted plane (`point_map + residual_vector_map`) instead of the
	off-plane scan correspondence point.
- KD-tree residual arrows remain anchored at the correspondence scan point in
	map frame and point toward its plane projection.

### Migration/runtime risk notes

- Low. This changes only default RViz config path selection; users can still
	override `rviz_config` explicitly on the command line.

## 2026-07-21 - KD-tree backend plane-point and normal RViz debug outputs

### Files changed

- [include/scanAlignment/IMappingBackend.hpp](include/scanAlignment/IMappingBackend.hpp)
- [include/scanAlignment/KdTreeLmBackend.hpp](include/scanAlignment/KdTreeLmBackend.hpp)
- [include/scanAlignment/VoxelPkoBackend.hpp](include/scanAlignment/VoxelPkoBackend.hpp)
- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [src/scanAlignment/KdTreeLmBackend.cpp](src/scanAlignment/KdTreeLmBackend.cpp)
- [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp)
- [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Added a new backend debug interface to expose accepted planar correspondence
	points and fitted normals from the last surf-optimization iteration:
	`getLastPlaneNormalSamples()`.
- KD-tree backend now captures per-point map-frame correspondences and fitted
	unit normals for accepted planar constraints at each iteration; after
	alignment, the buffer corresponds to the last executed surf optimization.
- Added RViz debug topics for KD-tree backend in map optimization publishing:
	- `liorf/mapping/kdtree_plane_points` (`sensor_msgs/PointCloud2`)
	- `liorf/mapping/kdtree_plane_normals` (`visualization_msgs/MarkerArray`)
	- `liorf/mapping/kdtree_plane_residuals` (`visualization_msgs/MarkerArray`)
- Each KD-tree sample now also stores a point-to-plane residual vector in map
	frame; residual arrows are published from correspondence point toward its
	projection on the fitted plane.
- Publishing is gated to `backend_type == "kdtree_lm"` and subscription
	presence to avoid unnecessary overhead when disabled.

### Migration/runtime risk notes

- Low. This change adds debug/visualization outputs and a backend interface
	extension without modifying scan-to-map optimization math or map update logic.

## 2026-07-21 - Complementary-odom telemetry + structured parameter refactor

### Files changed

- [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp)
- [src/mapOptimization/mapOptimization_graph.cpp](src/mapOptimization/mapOptimization_graph.cpp)
- [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp)
- [src/mapOptimization/mapOptimization_degeneracy.cpp](src/mapOptimization/mapOptimization_degeneracy.cpp)
- [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [CMakeLists.txt](CMakeLists.txt)
- [AGENTS.md](AGENTS.md)
- [include/utility.h](include/utility.h)
- [config/anymal.yaml](config/anymal.yaml)
- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
- [scripts/plot_complementary_odom_scale_diagnostics.py](scripts/plot_complementary_odom_scale_diagnostics.py)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Extended `[COMPLEMENTARY_ODOM_SCALE]` logging payload with additional speed metrics
	for data-backed analysis of scale behavior:
	- `complementary_odom_lin_speed_orig_mps`: linear speed from original additional
		odometry twist (`||xi_add_lidar.linear||`).
	- `lidar_lin_speed_nondeg_mps`: LiDAR non-degenerate linear speed from
		reprojection (`||t_lidar_nondeg|| / dt_scan`).
	- `complementary_odom_lin_speed_nondeg_mps`: complementary-odometry non-degenerate
		linear speed (`||t_add_nondeg|| / dt_scan`).
	- `lidar_lin_speed_proj_scale1_mps`: LiDAR linear speed from the projected
		first-correction motion (`T_proj_motion`) under scale fixed to 1.0.
	- `lidar_lin_speed_after_scale_mps`: LiDAR-target speed implied by scaled
		complementary-odometry non-degenerate displacement
		(`(||t_add_nondeg|| * scale_applied) / dt_scan`).
- Plotter now consumes the logged non-degenerate LiDAR/complementary-odometry
	speed fields directly (with backward-compatible fallback), instead of
	recomputing LiDAR non-degenerate speed from displacement fields.
- Complementary odometry integration parameters are now grouped under a
	dedicated `complementaryOdom` structure in code and YAML, mirroring the
	`degeneracyDetection` organization.
- Compensation mode selection was moved from
	`complementaryOdom.degeneracyMode` to
	`degeneracyDetection.compensation_source`.
- `complementaryOdomTopic` remains a top-level topic parameter next to other
	topics; only integration/extrinsic settings moved under
	`complementaryOdom.*`.
- Updated `anymal.yaml` and `lio_sam_ouster.yaml` to the new structured keys,
	including default complementary-odom values in `lio_sam_ouster.yaml`.
- Split degeneracy detection and complementary odometry handling into a
  dedicated translation unit `mapOptimization_degeneracy.cpp`:
  - moved `applyDegeneracyStateOverride(...)` from `mapOptimization_scan.cpp`,
  - moved `complementaryOdomHandler(...)` and
    `resolveComplementaryOdomExtrinsics(...)` from `mapOptimization_core.cpp`,
  - extracted the inline degeneracy orchestration block from
    `scan2MapOptimization()` into a new method
    `runDegeneracyDetectionAndCompensation()`.
- Moved `publishComplementaryOdomDisplacementDebug(...)` from
  `mapOptimization_scan.cpp` to `mapOptimization_publish.cpp` so publishing
  stays owned by the publish translation unit.
- Renamed the keyframe-gating predicate from `saveFrame()` to
	`shouldSaveFrame()` and marked it `const` to reflect non-modifying intent;
	no runtime behavior change.
- Fixed const-correctness for the moved `shouldSaveFrame()` implementation by
	marking `pclPointToAffine3f(...)` as `const` in both declaration and
	definition, resolving build failure without changing behavior.
- Refactored `applyDegeneracyStateOverride(...)` into cohesive helper methods
	(`prepareDegeneracyOrthoBasis`, `inferComplementaryOdomTwist`,
	`buildScaledComplementaryPrediction`, `projectDegenerateCorrection`, and
	`writeAffineToTransformTobeMapped`) to improve maintainability while
	preserving existing compensation behavior and telemetry output.

- Low to medium. Algorithmic behavior is unchanged, but configuration keys for
	complementary-odom integration changed from flat names to
	`complementaryOdom.*` and require updated YAML.

---

## 2026-07-20 - Scale complementary-odom prediction via non-degenerate translation subspace

### Files changed

- [include/degeneracyDetection/TwistManipulation.hpp](include/degeneracyDetection/TwistManipulation.hpp)
- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [include/utility.h](include/utility.h)
- [config/anymal.yaml](config/anymal.yaml)
- [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp)
- [scripts/plot_complementary_odom_scale_diagnostics.py](scripts/plot_complementary_odom_scale_diagnostics.py)
- [scripts/pyproject.toml](scripts/pyproject.toml)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Added `projectOntoBasisTranslation(...)` helper: projects a 3D translation
  onto the translation subspace spanned by the linear parts of a twist basis
  (with internal Gram-Schmidt re-orthonormalization of the linear parts).
- `applyDegeneracyStateOverride(...)` now estimates an online scale for the
  complementary odometry displacement by comparing the LiDAR-measured and
  complementary-odom-predicted translation components in the non-degenerate subspace,
  where scan matching is trusted.
- The scale is applied to the translation of the complementary-odom displacement only
  (rotation kept as-is) before projecting the correction onto the degenerate
  subspace. No temporal smoothing: the scale reacts instantly (e.g. slippage).
- New ROS parameters:
  - `complementaryOdomScaleEstimationEnabled` (bool, default `true`): toggles applying
    the estimated scale (metrics are still computed and logged when off).
  - `complementaryOdomScaleMinNonDegenerateSpeed` (double, default `0.05` m/s):
    observability gate; both non-degenerate translation components must
    exceed this speed over `dt_scan`, otherwise scale = 1.
  Scale is clamped to `[0.2, 5.0]`.
- Data-backed telemetry: every degeneracy frame with a complementary-odom prediction
  writes an `[COMPLEMENTARY_ODOM_SCALE]` event (unthrottled diagnostics event log +
  1 Hz throttled console log) containing applied scale, raw norm-ratio scale,
  least-squares reference scale, direction mismatch angle `theta_deg`
  between the non-degenerate components, both component norms, the gate
  threshold, and gate/enable states.
- Added plotting utility `plot_complementary_odom_scale_diagnostics.py` for event logs:
	- loads `[COMPLEMENTARY_ODOM_SCALE]` / `[COMPLEMENTARY_ODOM_TWIST]` from `event.txt`
	- plots applied scale, raw ratio scale, raw LS scale
	- plots `theta_deg` and gate/enable states
	- plots observable non-degenerate translational speed traces against the
		configured minimum speed threshold
	- plots complementary odometry twist linear/angular norms
	The generated figure is saved as `complementary_odom_scale_diagnostics.png` in the
	selected run directory by default.
- New debug visualization in `liorf/mapping/complementary_odom/correction_direction`
  markers: green arrow = non-degenerate LiDAR displacement component,
  magenta arrow = non-degenerate complementary-odom displacement component (map frame,
  published only when the observability gate passes).

### Migration/runtime risk notes

- Only active when `complementaryOdomDegeneracyMode == "complementary_odom"` and degeneracy is
  detected; no behavior change otherwise.
- Core assumption: complementary-odom error is an isotropic scale error, so the scale
  observed in non-degenerate directions transfers to degenerate ones.

---

## 2026-07-20 - Re-anchor degeneracy optimization markers to corrected pose

### Files changed

- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp)
- [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Degeneracy perturbation scans remain published in their original perturbation/alignment frames for faithful process visualization.
- Degeneracy perturbation pose and marker products are now rigidly re-anchored to the post-override corrected pose when `applyDegeneracyStateOverride(...)` updates the frame pose.
- Re-anchor transform computation is now owned by the publishing path (`publishPerturbationDebugProducts`) using two passed anchors (pre-override and post-override pose), keeping scan optimization free of visualization-delta math.
- Re-anchoring applies to:
	- `perturbed_pose_{0,1,2}` and `aligned_pose_{0,1,2}` outputs
	- displacement arrows in `liorf/mapping/degeneracy/displacements`
	- optimization path line strips and step arrows in `liorf/mapping/degeneracy/optimization_paths`
- Relative geometry of perturbation products is preserved; only the visualization anchor changes to reduce visible jumpiness.

### Migration/runtime risk

Low. This is a visualization-only change for perturbation pose/marker products; scan-alignment/optimization logic is unchanged.

## 2026-07-20 - Group degeneracy detection runtime parameters into a single config structure

### Files changed

- [include/utility.h](include/utility.h)
- [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp)
- [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Refactored `ParamServer` degeneracy runtime settings into a single `DegeneracyDetectionParameters` structure.
- Added explicit nested groups:
	- `jacobianBased` (`compute`, `log`, `threshold`)
	- `perturbationBased` (existing perturbation tuning parameters)
- Updated map-optimization call sites to read from the new grouped structure.
- ROS parameter keys and defaults remain unchanged, so existing YAML configs continue to work without migration.

### Migration/runtime risk

Low. This is a structural refactor of in-code parameter organization with unchanged parameter names and defaults.

## 2026-07-17 - Degeneracy perturbation map input and transform-update consistency fixes

### Files changed

- [include/degeneracyDetection/DegeneracyDetector.hpp](include/degeneracyDetection/DegeneracyDetector.hpp)
- [src/degeneracyDetection/DegeneracyDetector.cpp](src/degeneracyDetection/DegeneracyDetector.cpp)
- [include/scanAlignment/IMappingBackend.hpp](include/scanAlignment/IMappingBackend.hpp)
- [include/scanAlignment/KdTreeLmBackend.hpp](include/scanAlignment/KdTreeLmBackend.hpp)
- [src/scanAlignment/KdTreeLmBackend.cpp](src/scanAlignment/KdTreeLmBackend.cpp)
- [include/scanAlignment/ScanAligner.hpp](include/scanAlignment/ScanAligner.hpp)
- [src/scanAlignment/ScanAligner.cpp](src/scanAlignment/ScanAligner.cpp)
- [include/scanAlignment/VoxelPkoBackend.hpp](include/scanAlignment/VoxelPkoBackend.hpp)
- [src/scanAlignment/VoxelPkoBackend.cpp](src/scanAlignment/VoxelPkoBackend.cpp)
- [include/utility.h](include/utility.h)
- [include/liorf_diagnostics.h](include/liorf_diagnostics.h)
- [src/liorf_diagnostics.cpp](src/liorf_diagnostics.cpp)
- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp)
- [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp)
- [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
- [scripts/plot_jacobian_perturbation_degeneracy.py](scripts/plot_jacobian_perturbation_degeneracy.py)
- [scripts/pyproject.toml](scripts/pyproject.toml)
- [config/anymal.yaml](config/anymal.yaml)
- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
- [rviz/mapping.rviz](rviz/mapping.rviz)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Degeneracy perturbation detection now uses the active backend local map cloud instead of the current scan as the map proxy.
- Added a safe fallback to use the scan cloud only when the backend local map is unavailable or empty.
- Removed duplicate `transformUpdate()` invocation in scan-to-map optimization so IMU blending and constraints are applied once per frame.
- Ensured `transformUpdate()` still executes on low-feature frames to keep pose constraints and incremental state updates consistent.
- Updated the perturbation descriptive-number default threshold from `0.5` to `0.1` to match the reference defaults.
- Added publication of three perturbation scan point clouds for degeneracy debugging:
	- `liorf/mapping/degeneracy/perturbed_scan_0`
	- `liorf/mapping/degeneracy/perturbed_scan_1`
	- `liorf/mapping/degeneracy/perturbed_scan_2`
- Added publication of three aligned (post-alignment) perturbation scan point clouds:
	- `liorf/mapping/degeneracy/aligned_scan_0`
	- `liorf/mapping/degeneracy/aligned_scan_1`
	- `liorf/mapping/degeneracy/aligned_scan_2`
- Added per-hypothesis pose outputs as `PoseStamped` for both perturbed and aligned hypotheses:
	- `liorf/mapping/degeneracy/perturbed_pose_{0,1,2}`
	- `liorf/mapping/degeneracy/aligned_pose_{0,1,2}`
- Added displacement arrows (`MarkerArray`) from perturbed pose to aligned pose:
	- `liorf/mapping/degeneracy/displacements`
- Added per-perturbation optimization path visualization from LM iteration traces using colored line strips plus small step arrows:
	- `liorf/mapping/degeneracy/optimization_paths`
- Bootstrapped RViz with a dedicated `Degeneracy Perturbation Debug` group that overlays the three perturbed clouds, three aligned clouds, and displacement markers with distinct colors and default enabled visibility.
- Refactored perturbation-based degeneracy runtime configuration into a dedicated parameter structure in `ParamServer`.
- Added configurable perturbation aligner iteration budget (`max_icp_steps`).
- Refactored mapping backend alignment API to accept an optional per-call override config object, removing degeneracy-specific branching from optimizer internals.
- Added backend config cloning flow (`getAlignmentConfig()` -> local override edits -> `align(..., overrideConfig)`) so temporary behavior changes are explicit and stateless.
- Added full Jacobian-based degeneracy telemetry payload (eigenvalues/eigenvectors/thresholds/zeroed-modes/correspondence count) to alignment metrics for both KD-tree and voxel aligners.
- Added exactly two Jacobian switches:
	- `liorf.degeneracyDetection.jacobianBased.compute`
	- `liorf.degeneracyDetection.jacobianBased.log`
- Added configurable Jacobian degeneracy eigenvalue threshold parameter:
	- `liorf.degeneracyDetection.jacobianBased.threshold`
- Jacobian telemetry is now emitted every scan (including non-degenerate and non-computed cases with reason codes) in parallel with perturbation-based diagnostics.
- Added persistent Jacobian telemetry output file:
	- `jacobian_degeneracy_metrics.csv`
- Removed the consistency-stats concept from perturbation degeneracy detection and scan-loop diagnostics.
- Added per-scan perturbation degeneracy telemetry output file for deterministic correlation with Jacobian signals:
	- `perturbation_degeneracy_metrics.csv`
- Added plotting utility to visualize all 6 Jacobian eigenvalues and perturbation detection timeline from run logs:
	- `scripts/plot_jacobian_perturbation_degeneracy.py`
- Enhanced Jacobian/perturbation plotting utility with log-scale eigenvalue visualization (default) and optional dedicated smallest-eigenvalue subplot for high dynamic-range diagnostics.
- Plotting utility now also always exports a standalone smallest-eigenvalue figure (`jacobian_min_eigen_plot.png`) for quick focused inspection.
- Visualization now includes a max/min eigenvalue proportion plot (`max eigenvalue / min eigenvalue`) to highlight Jacobian conditioning trends.
- Exposed perturbation-based degeneracy parameters as ROS parameters under:
	- `liorf.degeneracyDetection.perturbationBased.n_multiplier`
	- `liorf.degeneracyDetection.perturbationBased.max_icp_steps`
	- `liorf.degeneracyDetection.perturbationBased.max_perturbation_angle_deg`
	- `liorf.degeneracyDetection.perturbationBased.descriptive_number_threshold`
	- `liorf.degeneracyDetection.perturbationBased.eigen_value_threshold`
	- `liorf.degeneracyDetection.perturbationBased.verbose`

### Migration/runtime risk

Low to medium. Degeneracy detection sensitivity will increase due to the lower threshold, and map-based perturbation scaling may alter when degeneracy is triggered compared to prior behavior.

## 2026-07-13 - Complementary odometry degeneracy aiding fixes and debug topics

### Files changed

- [include/utility.h](include/utility.h)
- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp)
- [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp)
- [config/anymal.yaml](config/anymal.yaml)
- [CHANGELOG.md](CHANGELOG.md)

### Behavior impact

- Renamed feature surface from external odometry to complementary odometry in code and configuration.
- Fixed degeneracy override timing to use scan delta (`curTimeDiff`) instead of a value that could collapse to zero.
- Corrected motion-frame conjugation for complementary-odometry delta projection into LiDAR frame.
- Updated correction logic to keep the optimized pose in non-degenerate directions and apply only the projected optimized-to-predicted displacement in degenerate directions.
- Added correction-direction marker visualization topic for complementary-odometry degeneracy correction:
	- `liorf/mapping/complementary_odom/correction_direction`
- Added throttled runtime logging of inferred complementary-odometry twist (linear/angular components and norms).
- Added a staleness gate to skip outdated complementary-odometry samples during correction.
- Changed complementary-odometry sample pairing to nearest timestamp matching against previous/current LiDAR stamps (instead of min-delta-time pairing).
- Removed unused complementary-odometry Odometry/Path debug topics to keep only direction-focused visualization.

### Migration/runtime risk

Low. Runtime behavior is unchanged for users already using `addOdom*` keys; old `extOdom*` keys are no longer supported.

## 2026-04-09 — Publish LiDAR-estimated GPS fix + ENU orientation

### Files changed

- Build and packaging:
	- [CMakeLists.txt](CMakeLists.txt)
	- [package.xml](package.xml)
- Core SLAM and runtime:
	- [include/utility.h](include/utility.h)
	- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
	- [include/degeneracyDetection/DegeneracyDetector.hpp](include/degeneracyDetection/DegeneracyDetector.hpp)
	- [src/degeneracyDetection/DegeneracyDetector.cpp](src/degeneracyDetection/DegeneracyDetector.cpp)
	- [src/mapOptimization/main.cpp](src/mapOptimization/main.cpp)
	- [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp)
	- [src/mapOptimization/mapOptimization_map.cpp](src/mapOptimization/mapOptimization_map.cpp)
	- [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp)
	- [src/mapOptimization/mapOptimization_graph.cpp](src/mapOptimization/mapOptimization_graph.cpp)
	- [src/mapOptimization/mapOptimization_gps.cpp](src/mapOptimization/mapOptimization_gps.cpp)
	- [src/mapOptimization/mapOptimization_loop.cpp](src/mapOptimization/mapOptimization_loop.cpp)
	- [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
	- [src/imuPreintegration.cpp](src/imuPreintegration.cpp)
	- [src/imageProjection.cpp](src/imageProjection.cpp)
- Export, interfaces, and tooling:
	- [srv/SaveMap.srv](srv/SaveMap.srv)
	- [include/export/MapExporter.hpp](include/export/MapExporter.hpp)
	- [include/export/map_types.hpp](include/export/map_types.hpp)
	- [src/export/MapExporter.cpp](src/export/MapExporter.cpp)
	- [scripts/save_map.sh](scripts/save_map.sh)
	- [scripts/saved_map_output_README.md](scripts/saved_map_output_README.md)
	- [scripts/visualize_saved_map_satellite.py](scripts/visualize_saved_map_satellite.py)
- Diagnostics, docs, and policy:
	- [include/liorf_diagnostics.h](include/liorf_diagnostics.h)
	- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
	- [README.md](README.md)
	- [ARCHITECTURE.md](ARCHITECTURE.md)
	- [AGENTS.md](AGENTS.md)
	- [CHANGELOG.md](CHANGELOG.md)


### Behavior impact

- Refactored map optimization into split translation units under [src/mapOptimization](src/mapOptimization) with explicit role separation (core/map/scan/gps/loop/publish/graph), preserving runtime behavior while improving maintainability and build isolation.
- Added and hardened observability across scan-to-map and runtime health:
	- stage-level scan matching metrics and debug clouds,
	- diagnostics CSV/event telemetry improvements,
	- prediction safety logs with timestamp context,
	- throttled summaries for optimization and timing slices.
- Improved prediction and frame-processing robustness:
	- safer constant-velocity translation prediction guards,
	- non-positive frame-delta skip handling,
	- stale-frame/backlog protection and queue-depth controls.
- Improved runtime TF/frame handling:
	- dedicated runtime TF coordination module,
	- safer lidar/baselink frame resolution and startup behavior,
	- conflict checks for externally parented baselink frames.
- Expanded map export pipeline and tooling:
	- structured output layout and richer save metadata,
	- ENU/local artifact support and georeference cleanup,
	- save service/CLI/launch tooling alignment,
	- satellite visualization helper updates.
- GPS fusion and publication flow was strengthened:
	- floating-anchor based global-local alignment pipeline,
	- improved GPS-LiDAR association gating and diagnostics,
	- better publication readiness behavior for GPS-derived outputs.
- Added additional runtime products for debugging and consumers:
	- keyframe downsampled cloud exports and debug companion stream,
	- matched-feature and colored surface debug visualizations,
	- baselink and GPS-fused odometry publication variants.
- Integrated degeneracy direction workflow in scan-to-map:
	- basis extraction now runs before visualization publish,
	- detector-owned consistency accounting (hit-rate/streaks),
	- throttled detection/consistency log events now reflect detector state.
- Build and toolchain hardening:
	- diagnostics linkage/dependency cleanup,
	- ODR/warning cleanup and guarded compiler-warning suppression,
	- exporter target isolation to reduce unnecessary recompilation.

### Migration/runtime risk

Low risk. Behavior is intentionally delayed until at least two accepted GPS factors so rotational alignment is better constrained before publishing fused outputs.

---

## 2026-04-08 — Agent documentation and working-context setup

### Files changed

- [AGENTS.md](AGENTS.md)
- [ARCHITECTURE.md](ARCHITECTURE.md)
- [CHANGELOG.md](CHANGELOG.md)
- [src/mapOptmization.cpp](src/mapOptmization.cpp)
- [include/liorf_diagnostics.h](include/liorf_diagnostics.h)
- [include/tictoc.h](include/tictoc.h)
- [README.md](README.md)
- [include/utility.h](include/utility.h)
- [launch/liorf.launch.py](launch/liorf.launch.py)
- [config/lio_sam_default.yaml](config/lio_sam_default.yaml)
- [config/lio_sam_identity.yaml](config/lio_sam_identity.yaml)
- [config/lio_sam_livox.yaml](config/lio_sam_livox.yaml)
- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
- [config/kitti.yaml](config/kitti.yaml)
- [config/M2DGR.yaml](config/M2DGR.yaml)
- [config/mulran.yaml](config/mulran.yaml)
- [config/ubran_hongkong.yaml](config/ubran_hongkong.yaml)
- [launch/run_lio_sam_ouster.launch.py](launch/run_lio_sam_ouster.launch.py)
- [launch/run_lio_sam_default.launch.py](launch/run_lio_sam_default.launch.py)
- [launch/run_lio_sam_identity.launch.py](launch/run_lio_sam_identity.launch.py)
- [launch/run_lio_sam_livox.launch.py](launch/run_lio_sam_livox.launch.py)
- [launch/run_kitti.launch.py](launch/run_kitti.launch.py)
- [launch/run_M2DGR.launch.py](launch/run_M2DGR.launch.py)
- [launch/run_mulran.launch.py](launch/run_mulran.launch.py)
- [launch/run_ubran_hongkong.launch.py](launch/run_ubran_hongkong.launch.py)
- [launch/run_lio_sam_bench.py](launch/run_lio_sam_bench.py)
- [scripts/plot_time_slicing_stats.py](scripts/plot_time_slicing_stats.py)

### Behavior impact

- Added repository architecture and data-flow guidance for future LLM/code agents.
- Added mandatory policy to record all future modifications in this changelog.
- Documented that current iteration priority should use:
	- [launch/run_lio_sam_ouster.launch.py](launch/run_lio_sam_ouster.launch.py)
	- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
- Implemented floating-anchor GPS fusion in map optimization:
	- replaced direct `GPSFactor` insertion with a custom floating-anchor factor tied to persistent `T_GL` key,
	- added `earth -> map` TF publication from optimized `T_GL`,
	- added `liorf/enu_to_local_offset` PoseWithCovariance topic,
	- switched map-save GPS metadata output from `map_origin.txt` to structured georeference YAML including datum + ENU-to-local transform (now exposed as `T_enu_local`).
- Updated README GPS section to describe floating-anchor behavior, related topics/TF, and map metadata output.
- Added [ARCHITECTURE.md](ARCHITECTURE.md) with a detailed frame model section (frame roles, TF chain, ownership, gating, and parameter mapping), and added a one-sentence frame summary in [README.md](README.md) that links to it.
- Restructured [README.md](README.md) to add a standalone frame-model chapter (separate from GPS integration notes) and explicitly list GPS-enabled frames there.
- Refined frame documentation in [README.md](README.md) and [ARCHITECTURE.md](ARCHITECTURE.md): added REP-105 citation, clarified `mapFrameLocal` ordering before `odometryFrame`, documented always-published hardcoded `odometryFrame->lidar_link` compatibility branch, and documented the inverse `lidar->baselink` compensation used to keep TF directionality/tree consistency.
- Added explicit note in [README.md](README.md) and [ARCHITECTURE.md](ARCHITECTURE.md) that, for now, `mapFrameLocal` and `odometryFrame` are effectively the same in nominal operation while remaining logically separated for future extensions.
- Added configurable GPS frame parameters with defaults:
	- parameter schema updated to `odometryFrame`, `mapFrameLocal` (local SLAM frame), `mapFrameEnu` (global ENU), `ECEFframe`.
	- dataset YAML defaults set to `odometryFrame: odom`, `mapFrameLocal: map_local`, `mapFrameEnu: map`, `ECEFframe: earth`.
	- removed static `map->odom` identity publisher from launch files; `map_local->odom` is now node-published.
	- map optimization TF hierarchy now publishes: `ECEFframe->mapFrameEnu`, `mapFrameEnu->mapFrameLocal`, `mapFrameLocal->odometryFrame`, and keeps `odometryFrame->base_link` in the same node location.
	- `mapFrameEnu->mapFrameLocal` is now always published; before anchor estimation it is identity, then it transitions to the optimized floating-anchor transform.
	- once initialized, last valid global transforms continue publishing through GPS denial.
- Restored compatibility for visualization/output topics by keeping key map products in `odometryFrame` (`trajectory`, local map clouds, registered clouds, and path), while retaining the internal/global TF hierarchy.
- Added centralized diagnostics infrastructure:
	- new [include/liorf_diagnostics.h](include/liorf_diagnostics.h) logger/telemetry module creates timestamped run directories under `~/.ros/liorf_logs/run_YYYYMMDD_HHMMSS/`, writes `timing_stats.csv`, `events.log`, and startup `run_parameters.yaml` dump, and publishes 1 Hz JSON telemetry on `/liorf/diagnostics`.
	- extended [include/tictoc.h](include/tictoc.h) with non-printing `double toc()` to support silent elapsed-time sampling in milliseconds.
	- integrated diagnostics ownership in [include/utility.h](include/utility.h) and initialization/use in [src/mapOptmization.cpp](src/mapOptmization.cpp), including per-stage timing slices around the core LiDAR pipeline, LiDAR/GPS freshness tracking, and redirecting the constant-velocity translation prediction throttle message from console to `events.log`.
- Launch/replay usability improvements:
	- added optional `use_sim_time` launch argument handling so launch-time override applies only when explicitly provided; otherwise YAML values are preserved.
	- reduced launch-file duplication by making dataset-specific launchers delegate to [launch/liorf.launch.py](launch/liorf.launch.py) and forward arguments.
	- documented replay best practice in [README.md](README.md): use `ros2 bag play ... --clock` and set `use_sim_time=true` via launch argument or parameter file for diagnostics during bag playback.
- Added diagnostics post-processing helper [scripts/plot_time_slicing_stats.py](scripts/plot_time_slicing_stats.py) to plot per-stage time-slicing statistics from `timing_stats.csv` (latest run or user-provided path), with optional smoothing and PNG output; documented usage in [README.md](README.md).
- Diagnostics logging now also updates a stable symlink `~/.ros/liorf_logs/latest` to the newest run directory for easier tooling and scripting.
- Updated the plotting helper defaults: when no input is provided it now reads latest diagnostics by default, and it now always both saves and displays the generated plot.
- Added a hybrid rolling local-map path in [src/mapOptmization.cpp](src/mapOptmization.cpp) using a spatial hash grid keyed by integer voxels:
	- local map generation now uses `manageLocalMap()` in the LiDAR pipeline,
	- periodic full rebuilds are triggerable while non-rebuild cycles prune/fill directly from the hash map,
	- incremental insertion of newly optimized scan points is done via `updateRollingMap()` after pose correction.
- Added map rebuild trigger controls in [include/utility.h](include/utility.h): `rebuild_on_loop_closure` and `rebuild_on_gps_jump` (both default `true`), and wired them into loop/GPS factor flow in [src/mapOptmization.cpp](src/mapOptmization.cpp).
- Added runtime diagnostics logging for rolling-map behavior in [src/mapOptmization.cpp](src/mapOptmization.cpp):
	- explicit `events.log` entries when map rebuilds are triggered (`loop_closure_factor`, `gps_periodic_60s`),
	- per-cycle local-map statistics logging (voxel count, local-map points, scan points, keyposes, radius/leaf settings) to the diagnostics log folder.
- Optimized local-map update policy in [src/mapOptmization.cpp](src/mapOptmization.cpp): local-map maintenance is now keyframe/rebuild-driven (`localMapDirty`), so non-keyframe frames reuse the cached local map; KD-tree refresh is similarly gated by map dirtiness.
- Reduced spatial-hash insertion overhead in [src/mapOptmization.cpp](src/mapOptmization.cpp) by pre-allocating `voxelHashMap` buckets via `reserve()` before bulk insertion loops (both rebuild and rolling-update paths), avoiding repeated mid-loop rehashing.
- Added finer timing slices in [src/mapOptmization.cpp](src/mapOptmization.cpp) to pinpoint runtime sources in diagnostics logs:
	- `manageLocalMap` sub-slices (rebuild extract/hash insert, incremental prune/materialize),
	- `updateRollingMap` sub-slices (transform + hash insert),
	- `scan2MapOptimization` sub-slices (`setInputCloud`, aggregated `surfOptimization`/`combineOptimizationCoeffs`/`LMOptimization`, and `transformUpdate`), plus throttled per-cycle iteration summaries.
- Enhanced [scripts/plot_time_slicing_stats.py](scripts/plot_time_slicing_stats.py) to display detailed per-stage timing statistics (count, mean, median, p90, p95, min, max, std), support summary sorting options, and optionally export the summary as CSV.
- Refined [scripts/plot_time_slicing_stats.py](scripts/plot_time_slicing_stats.py) to prioritize a more detailed time-slicing figure (faceted per-stage view by default) while disabling console stats print by default; textual stats are now opt-in via `--show-summary`.
- Updated [scripts/plot_time_slicing_stats.py](scripts/plot_time_slicing_stats.py) so default plotting now shows two plots side-by-side: full-stage overlay (previous view) and a dedicated map-handling-components panel (`manageLocalMap*`, `updateRollingMap*`).
- Refined [scripts/plot_time_slicing_stats.py](scripts/plot_time_slicing_stats.py) side-by-side view so the left panel now emphasizes only high-level pipeline stages (`updateInitialGuess`, `manageLocalMap`, `downsampleCurrentScan`, `scan2MapOptimization`, `saveKeyFramesAndFactor`, `correctPoses`, `updateRollingMap`) instead of all detailed slices.
- Updated [scripts/plot_time_slicing_stats.py](scripts/plot_time_slicing_stats.py) right-side panel stage selection to also include extraction dissection slices (`extractSurroundingKeyFrames*`, `extractNearby*`, `extractCloud*`) so the newly instrumented extraction path is plotted alongside map-handling slices.
- Added deeper timing dissection inside [src/mapOptmization.cpp](src/mapOptmization.cpp) for `extractSurroundingKeyFrames` path, including slices for `extractNearby` (radius search, pose collection/downsampling/remap, recent-pose append, extract call) and `extractCloud` (fuse/transform, downsample, cache maintenance).
- Updated transformed-keyframe cache policy in [src/mapOptmization.cpp](src/mapOptmization.cpp): cache entries are now pruned to retain only the last 0.5 seconds of keyframe-transformed clouds (instead of large-count eviction), and diagnostics now logs cached-cloud count in local-map stats plus throttled cache-prune events.
- Made transformed-cloud cache retention configurable via new parameter `transformed_cloud_cache_max_age_sec` loaded in [include/utility.h](include/utility.h), used by [src/mapOptmization.cpp](src/mapOptmization.cpp), and set in all primary dataset YAML profiles under [config/](config).
- Added explicit anti-backlog controls for LiDAR processing in [include/utility.h](include/utility.h), [src/mapOptmization.cpp](src/mapOptmization.cpp), and all primary [config/*.yaml](config):
	- `cloud_info_queue_depth` (applied to `liorf/deskew/cloud_info` subscription QoS keep-last depth),
	- `drop_stale_lidar_frames`,
	- `max_lidar_processing_lag_sec` (drops stale LiDAR frames in callback when lag exceeds threshold),
	which together bound post-playback buffering and favor frame dropping over accumulated localization lag.
- Fixed stale-frame drop criterion in [src/mapOptmization.cpp](src/mapOptmization.cpp) to compare each frame stamp against newest seen CloudInfo stamp (queue backlog age), avoiding false full-drop behavior when bag timestamps and node current time are in different clock domains.
- Enabled the two rebuild-trigger parameters by default across dataset profiles:
	- [config/lio_sam_default.yaml](config/lio_sam_default.yaml)
	- [config/lio_sam_identity.yaml](config/lio_sam_identity.yaml)
	- [config/lio_sam_livox.yaml](config/lio_sam_livox.yaml)
	- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
	- [config/kitti.yaml](config/kitti.yaml)
	- [config/M2DGR.yaml](config/M2DGR.yaml)
	- [config/mulran.yaml](config/mulran.yaml)
	- [config/ubran_hongkong.yaml](config/ubran_hongkong.yaml)

### Migration/runtime risk

- Low to moderate: GPS constraints now optimize a global-to-local anchor transform instead of directly snapping local key poses to global coordinates.
