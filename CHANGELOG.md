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

For active iterative work, prefer updating the current top entry instead of appending a new entry for each small adjustment.

---

## 2026-04-09 — Publish LiDAR-estimated GPS fix + ENU orientation

### Files changed

- [CMakeLists.txt](CMakeLists.txt)
- [package.xml](package.xml)
- [srv/SaveMap.srv](srv/SaveMap.srv)
- [include/utility.h](include/utility.h)
- [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp)
- [include/export/MapExporter.hpp](include/export/MapExporter.hpp)
- [include/export/map_types.hpp](include/export/map_types.hpp)
- [src/mapOptimization/main.cpp](src/mapOptimization/main.cpp)
- [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp)
- [src/mapOptimization/mapOptimization_map.cpp](src/mapOptimization/mapOptimization_map.cpp)
- [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp)
- [src/mapOptimization/mapOptimization_gps.cpp](src/mapOptimization/mapOptimization_gps.cpp)
- [src/mapOptimization/mapOptimization_loop.cpp](src/mapOptimization/mapOptimization_loop.cpp)
- [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp)
- [src/export/MapExporter.cpp](src/export/MapExporter.cpp)
- [src/imuPreintegration.cpp](src/imuPreintegration.cpp)
- [include/liorf_diagnostics.h](include/liorf_diagnostics.h)
- [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml)
- [scripts/save_map.sh](scripts/save_map.sh)
- [scripts/build_liorf.sh](scripts/build_liorf.sh)
- [scripts/saved_map_output_README.md](scripts/saved_map_output_README.md)
- [scripts/visualize_saved_map_satellite.py](scripts/visualize_saved_map_satellite.py)
- [README.md](README.md)
- [ARCHITECTURE.md](ARCHITECTURE.md)
- [CHANGELOG.md](CHANGELOG.md)
- [AGENTS.md](AGENTS.md)


### Behavior impact

- split monolithic map optimization implementation into a dedicated module layout under [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp) and [src/mapOptimization/main.cpp](src/mapOptimization/main.cpp), [src/mapOptimization/mapOptimization_core.cpp](src/mapOptimization/mapOptimization_core.cpp), [src/mapOptimization/mapOptimization_map.cpp](src/mapOptimization/mapOptimization_map.cpp), [src/mapOptimization/mapOptimization_scan.cpp](src/mapOptimization/mapOptimization_scan.cpp), [src/mapOptimization/mapOptimization_gps.cpp](src/mapOptimization/mapOptimization_gps.cpp), [src/mapOptimization/mapOptimization_loop.cpp](src/mapOptimization/mapOptimization_loop.cpp), and [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp).
- moved the build target for `liorf_mapOptmization` in [CMakeLists.txt](CMakeLists.txt) to compile from the new split sources; runtime behavior is intended to remain unchanged with lower refactor-risk by preserving the original `mapOptimization` state and method logic.
- fixed post-split linker ODR issues in [include/utility.h](include/utility.h) by marking header-defined shared symbols as `inline` (`common_lib_` variable and `QosPolicy(...)`), allowing safe inclusion from multiple `mapOptimization` translation units.
- fixed compiler warning cleanup in split sources: `yawDiffRad` marked `[[maybe_unused]]` in [src/mapOptimization/mapOptimization_loop.cpp](src/mapOptimization/mapOptimization_loop.cpp), removed unused `lastSLAMInfoPubSize` in [src/mapOptimization/mapOptimization_publish.cpp](src/mapOptimization/mapOptimization_publish.cpp), and initialized QoS profile from `rmw_qos_profile_default` in [include/utility.h](include/utility.h) to avoid `-Wmaybe-uninitialized`.
- added GCC-only suppression `-Wno-array-bounds` for target `liorf_mapOptmization` in [CMakeLists.txt](CMakeLists.txt) to silence known Eigen/PCL template false positives emitted from external headers during optimization builds.
- fixed `-Wreturn-type` warning in [include/Scancontext.cpp](include/Scancontext.cpp) by adding an explicit fallback return path in `xy2theta(...)` for degenerate/unexpected numeric cases.
- follow-up review hardening: [include/Scancontext.cpp](include/Scancontext.cpp) now computes heading with `atan2` and normalizes to `[0, 360)` to avoid division-by-zero/NaN-prone quadrant math.
- follow-up review hardening: removed global `using namespace gtsam;` and symbol-shorthand `using` declarations from public header [include/mapOptimization/mapOptimization.hpp](include/mapOptimization/mapOptimization.hpp), and qualified remaining GTSAM member types.
- follow-up review hardening: [CMakeLists.txt](CMakeLists.txt) now gates `-Wno-array-bounds` behind `LIORF_SUPPRESS_GNU_ARRAY_BOUNDS_WARNINGS` and Release+GNU conditions instead of unconditional GNU application.

Adds fused GPS publishers to `mapOptimization`:

- added translation-prediction safety params in [include/utility.h](include/utility.h): `maxTranslationPrediction` (default `5.0 m`) and `minTranslationPredictionSpeed` (default `0.0 m/s`, disabled).
- constant-velocity translation prediction in [src/mapOptmization.cpp](src/mapOptmization.cpp) now clamps to zero on threshold violations and emits explicit logs (`[TRANSLATION_PREDICTION_EXCEEDED]`, `[TRANSLATION_PREDICTION_SPEED_TOO_LOW]`).
- added warning publication topic `/liorf/warnings` in [include/liorf_diagnostics.h](include/liorf_diagnostics.h), and wired guard messages to publish there.
- diagnostics telemetry JSON now includes `max_translation_delta_last_batch_m`, and the max is tracked per diagnostics publish batch.
- diagnostics now write per-processed-frame time deltas to `time_deltas.csv` under each run folder.
- diagnostics now write unified frame metrics to `frame_metrics.csv` with columns: `stamp_sec,time_delta_s,prediction_delta_m,optimized_delta_m` — logging the frame processing interval, predicted motion magnitude, and actual optimized motion magnitude per frame.
- diagnostics now publish per-frame metrics as `std_msgs/msg/String` on `/liorf/frame_metrics` with JSON fields: `stamp_sec`, `time_delta_s`, `prediction_delta_m`, `optimized_delta_m`.
- `/liorf/frame_metrics` now publishes all numeric fields with fixed dot-decimal formatting at 3 digits after the decimal point.
- `stamp_sec` in `/liorf/frame_metrics` and `frame_metrics.csv` now uses LiDAR header time (frame stamp) instead of node/system wall time.
- split diagnostics implementation into [include/liorf_diagnostics.h](include/liorf_diagnostics.h) declarations + [src/liorf_diagnostics.cpp](src/liorf_diagnostics.cpp) definitions.
- added dedicated CMake target `liorf_diagnostics` and linked it to node executables so diagnostics-only changes rebuild a smaller compilation unit.
- unified diagnostics debug topics under `/liorf/debug/<log_name>` naming: telemetry, timing_stats, time_deltas, frame_metrics, event, warnings.
- diagnostics event log file renamed from `events.log` to `event.txt`; each event line now corresponds to one published message on `/liorf/debug/event`.
- each diagnostics stream now has one-to-one topic/file correspondence and publishes one message per appended file row/line.
- added ROS params for diagnostics file-writing control: master switch `diagnostics_write_files_master` and per-log switches `diagnostics_write_timing_stats`, `diagnostics_write_event`, `diagnostics_write_warnings`, `diagnostics_write_telemetry`, `diagnostics_write_time_deltas`, `diagnostics_write_frame_metrics`.
- parameter metadata file (`run_parameters.yaml`) remains always written regardless of diagnostics file-write switches.
- moved `[FRAME_TIME_DELTA]` details from event stream into per-frame diagnostics (`frame_metrics`) to reduce event noise.
- per-frame diagnostics now include `last_time_delta_s` and `estimated_velocity_mps` (`estimated_velocity_mps = last_optimized_delta_m / last_time_delta_s` when last dt > 0).
- configured primary Ouster profile [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml) with `maxTranslationPrediction: 5.0` and `minTranslationPredictionSpeed: 0.0`.

 - diagnostics now persist runtime staleness telemetry in [include/liorf_diagnostics.h](include/liorf_diagnostics.h) to `telemetry.csv` (`time_since_last_lidar_s`, `time_since_last_gps_s`) inside each run folder under `~/.ros/liorf_logs/run_*`.
 - added [scripts/plot_telemetry_staleness.py](scripts/plot_telemetry_staleness.py) to visualize LiDAR/GPS staleness signals from `telemetry.csv` (latest-run auto-discovery supported).
 - documented telemetry staleness log location and plotting usage in [README.md](README.md).
`liorf/mapping/lidar_gps_ned_pose` (`geometry_msgs/PoseStamped`, frame = `mapFrameNed`) is also published as the NED-frame equivalent.

Publishing now uses a two-stage policy tied to GPS factor observability:


GPS-derived transform/offset publications are also gated by the same readiness condition.

 updated [launch/save_map.launch.py](launch/save_map.launch.py) to invoke `scripts/save_map.sh` after service-availability wait, so launch-triggered exports print the same clean response summary as the helper script.
 updated [CMakeLists.txt](CMakeLists.txt) to install `scripts/` into package share so `save_map.launch.py` can resolve and run `save_map.sh` from installed package paths.
 updated [scripts/save_map.sh](scripts/save_map.sh) with explicit default variables for resolution/destination (plus env overrides) and clearer runtime messaging that node `savePCDDirectory` defaults are interpreted as HOME-relative for compatibility.

Added RViz visualization topic `/liorf/mapping/gps_constraints` (`visualization_msgs/MarkerArray`) with:

- received GPS ENU points (`SPHERE_LIST`)
- associated LiDAR key poses transformed to ENU (`SPHERE_LIST`)
- line connections from each accepted GPS factor measurement to its corresponding LiDAR key pose (`LINE_LIST`)

Improved GPS-LiDAR synchronization for GPS factor insertion and visualization:

- each stored GPS-LiDAR association now carries GPS timestamp metadata,
- GPS constraints are associated to the closest recent LiDAR keyframe by timestamp (instead of always the latest keyframe index),
- throttled warnings are emitted when GPS-to-keyframe association offset exceeds 2.0 seconds.
- each accepted GPS factor now emits a structured `[GPS_CONSTRAINT_ADDED]` log line to both terminal and diagnostics `events.log`, including GPS time, matched keyframe time, and their delta.
- matching now explicitly includes the current in-flight keyframe candidate (`timeLaserInfoCur`) so constraints can bind to the just-created factor-graph key instead of only previously saved keyframes.
- a hard gate now rejects GPS constraints when `|gps_t - keyframe_t| > 0.30 s`; skipped constraints are logged as `[GPS_CONSTRAINT_SKIPPED_TIME]` in diagnostics, and warnings are emitted in terminal.
- added `gps_processing_delay_sec` parameter (loaded in `ParamServer`): GPS measurements are now processed only after `gps_stamp + gps_processing_delay_sec` enters the processing window, enabling intentional ROS-time holdback before factor insertion.
- added configurable `gps_covariance_inflation_m` (default `2.0`) to inflate GPS factor variance on XYZ by `gps_covariance_inflation_m^2`, reducing short-term GPS pull when timing misalignment is present while still bounding long-term drift.
- set `gps_processing_delay_sec: 1.0` in the primary Ouster profile [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml) for immediate testing.
- set `gps_covariance_inflation_m: 2.0` in [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml).
- propagated remaining declared transport QoS parameters into [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml): `history_policy` and `reliability_policy`.
- local-frame map save outputs now use `_local` suffix for unambiguous naming: `SurfMap_local.pcd`, `GlobalMap_local.pcd`, `trajectory_local.pcd`, `transformations_local.pcd`.
- map save now also exports ENU-frame artifacts next to local-frame outputs when `T_enu_local` is initialized: `SurfaceMap_ENU.pcd`, `FullMap_ENU.pcd`, and `trajectory_ENU.pcd`.
- if `T_enu_local` is not initialized, ENU export is skipped with a warning while local-frame exports remain unchanged.
- `saveMapService()` path resolution now uses `std::filesystem`: `req->destination` is treated as absolute if it starts with `/`, HOME-expanded if it starts with `~/`, or HOME-relative otherwise; `getenv("HOME")` null-safety added; `system()` calls replaced with `std::filesystem::remove_all` / `create_directories`.
- added [scripts/save_map.sh](scripts/save_map.sh) helper to call `liorf/save_map` with CLI arguments for map resolution (`-r/--resolution`) and destination path (`-d/--destination`).
- [scripts/save_map.sh](scripts/save_map.sh) now prints the resolved save directory after a successful response; when destination is empty, it queries `/liorf_mapOptimization` parameter `savePCDDirectory` and resolves it with the same HOME-relative semantics as `saveMapService()`.
- extended [srv/SaveMap.srv](srv/SaveMap.srv) response with useful save metadata: `save_directory`, `enu_map_saved`, `keyframes_used`, `surf_points_local`, `surf_points_enu`, and `message`.
- [src/mapOptmization.cpp](src/mapOptmization.cpp) now populates these response fields from actual save execution state, including absolute destination path and ENU export status.
- on successful map save, [src/mapOptmization.cpp](src/mapOptmization.cpp) now writes the absolute map directory to `~/.liorf_last_saved_map_path`.
- [scripts/save_map.sh](scripts/save_map.sh) adds `-a/--absolute-path` and prints the detailed response summary after success.
- added global map point counters to [srv/SaveMap.srv](srv/SaveMap.srv) response: `global_points_local` and `global_points_enu`, and populated them in [src/mapOptmization.cpp](src/mapOptmization.cpp).
- added [scripts/visualize_saved_map_satellite.py](scripts/visualize_saved_map_satellite.py) to render saved map trajectory and surf density over satellite imagery into an interactive HTML map.
- [scripts/visualize_saved_map_satellite.py](scripts/visualize_saved_map_satellite.py) now supports omitted `--map-dir` and resolves in order: `~/.liorf_last_saved_map_path`, then default `~/Downloads/LOAM`, otherwise exits with a clear error.
- [scripts/visualize_saved_map_satellite.py](scripts/visualize_saved_map_satellite.py) now logs how map directory was resolved (user input, last-saved file, or default path), including a resolution trace.
- added Poetry environment file [scripts/pyproject.toml](scripts/pyproject.toml) for map tools dependencies (`folium`, `numpy`) and console script entrypoint `visualize-saved-map-satellite`.
- documented satellite overlay usage in [README.md](README.md).
- refreshed [README.md](README.md) map-saving section with `scripts/save_map.sh` usage, detailed `SaveMap` response fields, and persisted last-saved-path behavior (`~/.liorf_last_saved_map_path`).
- added [launch/save_map.launch.py](launch/save_map.launch.py) to trigger `liorf/save_map` via `ros2 launch` with arguments `resolution`, `destination`, `service_name`, and `wait_timeout_sec`.
- map-save outputs are now structured into subfolders: `maps/` and `trajectories/`.
- local/global map naming now uses `SurfaceMap_*` and `FullMap_*` files (replacing previous `SurfMap_*` / `GlobalMap_*` names in new exports).
- root georeference file is now `goereference.yaml`.
- georeference keys are now explicit: `gps_origin_enu` and `T_enu_local`.
- dedicated `gps_origin.yaml` export was removed (origin is now represented directly in `goereference.yaml`).
- save-map now also writes `save_summary.yaml` with ROS save time, keyframe count, dense trajectory count, GPS count, and map point counters.
- save-map default `resolution` is now controlled at the call sites (`scripts/save_map.sh` and `launch/save_map.launch.py`) with default value `0.2` (overridable via `LIORF_SAVE_MAP_DEFAULT_RESOLUTION`), and request value `0` remains a literal value meaning no downsampling.
- save-map now copies project template [scripts/saved_map_output_README.md](scripts/saved_map_output_README.md) into output root as `README.md`.
- added dense export toggles in ROS params: `save_dense_gps_trajectory` and `save_dense_odom_trajectory` (default `true`), loaded by `ParamServer` and set in [config/lio_sam_ouster.yaml](config/lio_sam_ouster.yaml).
- added save-time trajectory exports in `trajectories/`:
	- `gps_raw_geodetic.csv`
	- `trajectory_keyframes_local.csv`
	- `trajectory_dense_local.csv`
- `mapOptimization` now buffers full raw GPS and dense odometry histories in RAM and writes them at save time; older poses are not popped from these buffers.
- SaveMap response counters were renamed from `global_points_local/global_points_enu` to `full_points_local/full_points_enu`, and helper script parsing/output was updated accordingly.
- SaveMap response now also reports dense saved lengths: `trajectory_points_saved` and `gps_points_saved`.
- [scripts/visualize_saved_map_satellite.py](scripts/visualize_saved_map_satellite.py) now reads `goereference.yaml`, supports `gps_origin_enu`/`T_enu_local`, and keeps legacy metadata/path fallbacks.
- `goereference.yaml` now stores `T_enu_local` orientation as quaternion (`qx/qy/qz/qw`) instead of RPY, and [scripts/visualize_saved_map_satellite.py](scripts/visualize_saved_map_satellite.py) now supports quaternion metadata with legacy RPY fallback.
- fixed TF publication regression in [src/mapOptmization.cpp](src/mapOptmization.cpp): `odometryFrame -> lidar_link` is now built directly from `transformTobeMapped` (matching `publishOdometry()`), and `odometryFrame -> baselinkFrame` is derived from that LiDAR pose using `lidar2Baselink`.
- refactored [src/mapOptmization.cpp](src/mapOptmization.cpp) so smooth odom-side TF now comes from the incremental LiDAR odometry accumulator instead of the jump-prone optimized pose, and `mapFrameLocal -> odometryFrame` absorbs loop-closure/GPS corrections upstream.
- promoted incremental odometry publication state in [src/mapOptmization.cpp](src/mapOptmization.cpp) from static locals to class members so TF publication and odometry topics share a single smooth-motion source.
- removed IMU roll/pitch blending from incremental LiDAR odometry in [src/mapOptmization.cpp](src/mapOptmization.cpp), so `liorf/mapping/odometry_incremental` now reflects pure LiDAR scan-to-scan motion.
- `liorf/mapping/odometry_incremental` now keeps LiDAR-frame semantics but uses `child_frame_id = lidar_link` for consistency with the published TF branch.
- `liorf/mapping/odometry` now publishes the graph-optimized LiDAR pose in `mapFrameLocal -> lidar_link`.
- added new base-link odometry topics in [src/mapOptmization.cpp](src/mapOptmization.cpp): `liorf/mapping/baselink_odometry` (`mapFrameLocal -> baselinkFrame`) and `liorf/mapping/baselink_odometry_incremental` (`odometryFrame -> baselinkFrame`).
- added GPS-fused base-link odometry topics in [src/mapOptmization.cpp](src/mapOptmization.cpp): `liorf/mapping/baselink_gps_enu_odometry` and `liorf/mapping/baselink_gps_ned_odometry`.
- relabeled map-local visualization and map products in [src/mapOptmization.cpp](src/mapOptmization.cpp) to `mapFrameLocal` so published clouds, path, and loop-closure markers remain numerically consistent after the odometry split.
- updated [src/imuPreintegration.cpp](src/imuPreintegration.cpp) `TransformFusion` output headers/path frame to follow the incoming LiDAR odometry frame instead of hardcoding `odometryFrame`, keeping the fused topic labeling correct after `liorf/mapping/odometry` moved to `mapFrameLocal`.
- documented the LiDAR/base-link odometry split and smooth-odom TF behavior in [README.md](README.md) and [ARCHITECTURE.md](ARCHITECTURE.md).
- added manual GPS datum bootstrap in [include/utility.h](include/utility.h) and [src/mapOptmization.cpp](src/mapOptmization.cpp): new params `force_initial_gps`, `manual_gps_origin`, and `manual_global_heading` allow early ENU/LLA publishing and TF readiness before first sensor `NavSatFix`; first real GPS factor now inserts the floating-anchor prior exactly once while preserving manual yaw when enabled.
- documented manual GPS bootstrap usage and troubleshooting in [README.md](README.md), including parameter semantics (`force_initial_gps`, `manual_gps_origin`, `manual_global_heading`) and note about launching from the correct built/installed config.
- map-save file I/O/formatting logic was extracted from `mapOptimization::saveMapService()` into dedicated exporter utility files: [include/export/MapExporter.hpp](include/export/MapExporter.hpp) and [src/export/MapExporter.cpp](src/export/MapExporter.cpp).
- `mapOptimization::saveMapService()` now performs lightweight snapshot/locking and delegates heavy export work through `MapExporter`.
- shared keyframe pose point type was moved to [include/export/map_types.hpp](include/export/map_types.hpp) so exporter and map optimization use a common definition.
- CMake target graph now builds exporter implementation as a separate library target (`liorf_mapExporter`) linked into `liorf_mapOptmization`, so exporter `.cpp` changes avoid recompiling `mapOptmization.cpp` (relink still required).
- fixed `liorf_mapExporter` build wiring to link ROS interface typesupport target so generated headers like `liorf/srv/save_map.hpp` resolve during exporter-library compilation.
- added mandatory commit-message guidance in [AGENTS.md](AGENTS.md): short scoped header, required blank-line separator, concise body bullets for algorithmic/architectural changes, and max body line length of 72 characters.
- added explicit agent hint in [AGENTS.md](AGENTS.md) to avoid literal `\\n` in `git commit -m` messages and prefer `-F`/multi-`-m` usage for reliable bullet formatting.
- [scripts/visualize_saved_map_satellite.py](scripts/visualize_saved_map_satellite.py) now matches current save outputs by preferring `trajectories/trajectory_dense_local.csv` and `trajectory_keyframes_local.csv` before falling back to legacy trajectory PCD files.
- fixed [scripts/visualize_saved_map_satellite.py](scripts/visualize_saved_map_satellite.py) legacy georeference parsing: RPY-only `goereference.yaml` files are no longer misdetected as quaternion exports, which previously caused identity rotation to be applied to trajectories.

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
