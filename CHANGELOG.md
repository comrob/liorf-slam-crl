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

 [CMakeLists.txt](CMakeLists.txt)

### Behavior impact

Adds fused GPS publishers to `mapOptimization`:


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
- map save now also exports ENU-frame artifacts next to local-frame outputs when `T_global_local` is initialized: `SurfMap_ENU.pcd`, `GlobalMap_ENU.pcd`, and `trajectory_ENU.pcd`.
- if `T_global_local` is not initialized, ENU export is skipped with a warning while local-frame exports remain unchanged.
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
	- added `liorf/earth_to_map_offset` PoseWithCovariance topic,
	- switched map-save GPS metadata output from `map_origin.txt` to structured `map_metadata.yaml` including datum + `T_global_local`.
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
