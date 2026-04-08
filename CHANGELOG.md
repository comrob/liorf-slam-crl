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

### Migration/runtime risk

- Low to moderate: GPS constraints now optimize a global-to-local anchor transform instead of directly snapping local key poses to global coordinates.
