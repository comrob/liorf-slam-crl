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

## Archived changelogs

This changelog starts at the LILI-SAM rename. Entries from before it live in
[doc/changelogs/](doc/changelogs) and still use the old `liorf` naming as it
was at the time:

- [CHANGELOG-liorf-2026-04-08_2026-08-03.md](doc/changelogs/CHANGELOG-liorf-2026-04-08_2026-08-03.md)

---

## 2026-08-05 - Rename the method from LIORF to LILI-SAM (short name `lili`)

### Files changed

Repository-wide rename across 53 tracked files, including 8 renames:
`{include,src}/liorf_diagnostics.{h,cpp}` -> `lili_diagnostics.*`,
`launch/liorf.launch.py` -> `launch/lili.launch.py`,
`launch/run_lio_sam_{ouster.launch.py,bench.py}` -> `launch/run_lili_*`,
`config/lio_sam_{default,ouster}.yaml` -> `config/lili_{default,ouster}.yaml`,
`tools/scripts/build_liorf.sh` -> `tools/scripts/build_lili.sh`.

### Behavior impact

- ROS package `liorf` -> `lili`; interface namespaces become `lili::msg::*` and `lili::srv::*`.
- All topics/services move from `liorf/` to `lili/` (e.g. `liorf/save_map` -> `lili/save_map`).
- Executables and node names -> `lili_imageProjection`, `lili_imuPreintegration`,
	`lili_mapOptimization`, `lili_transformFusion`, `lili_imu_preintegration`.
- `LiorfDiagnostics` -> `LiliDiagnostics`; log root `~/.ros/liorf_logs` -> `~/.ros/lili_logs`;
	pointer file `~/.liorf_last_saved_map_path` -> `~/.lili_last_saved_map_path`.
- `LIORF_*` environment variables and the CMake option -> `LILI_*`.
- Docker images/services/containers `liorf_*` -> `lili_*`; container mount
	`src/liorf` -> `src/lili`; published image `liorf-crl` -> `lili-sam`.
- Fixed the `mapOptmization` typo: the CMake target, launch `executable=`/`name=`, the C++
	default node name, and [tools/scripts/save_map.sh](tools/scripts/save_map.sh) now all agree
	on `lili_mapOptimization`. Previously launch set `name='liorf_mapOptmization'` while
	`save_map.sh` queried `/liorf_mapOptimization`, so its `savePCDDirectory` probe always
	failed and silently fell back.

### Not renamed (deliberate)

- Upstream attribution: [LICENSE](LICENSE), `package.xml` maintainer/author, the upstream URL
	printed by [lib/common_lib.cpp](lib/common_lib.cpp), README acknowledgments, and the inline
	`liorf_yjz_lucky_boy` provenance markers.
- Third-party dataset profiles, matching their `kitti`/`mulran`/`M2DGR` siblings:
	`config/datasets/lio_sam_{identity,livox}.yaml` and
	`launch/datasets/run_lio_sam_{identity,livox}.launch.py`.
- Algorithm citations (`// copy from sc-lio-sam`).

### Migration/runtime risk notes

- Full rebuild required: drop stale `build/`, `install/`, `log/` and rebuild docker images from
	scratch, since the old cache refers to `src/liorf`.
- Rename the workspace source directory to `lili` (ament expects it to match the package name).
- Rosbags recorded earlier carry `/liorf/...` topics and need remapping to replay.
- Existing `~/.ros/liorf_logs` runs are no longer discovered by the plotting tools; move or
	symlink them to `~/.ros/lili_logs`.
- Git remotes and the repo directory still carry the old name. Clone paths in
	[docker/README.md](docker/README.md) were updated to `lili-sam` ahead of that and will not
	resolve until the remotes are renamed.
