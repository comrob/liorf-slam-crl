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
