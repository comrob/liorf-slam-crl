# LIORF (ROS 2 Jazzy) — Ubuntu 24.04 Setup

This repository provides a ROS 2 Jazzy port of LIORF/LIO-SAM style lidar-inertial odometry and mapping.

This guide is simplified for **Ubuntu 24.04** and uses **Zenoh (`rmw_zenoh_cpp`)** as the default ROS middleware.

---

## 1) System requirements

- Ubuntu 24.04
- ROS 2 Jazzy installed (`ros-base` or `desktop`)
- Build tools: `colcon`, `cmake`, `gcc/g++`

If ROS 2 Jazzy is not installed yet, follow the official ROS 2 Jazzy install first.

---

## 2) Install dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  libpcl-dev \
  libopencv-dev \
  libyaml-cpp-dev \
  libgeographiclib-dev \
  libtbb-dev \
  libboost-all-dev \
  ros-jazzy-rmw-zenoh-cpp
```

Initialize rosdep once (if needed):

```bash
sudo rosdep init || true
rosdep update
```

---

## 3) Install GTSAM (from source)

> The old `apt`/PPA instructions are intentionally removed. Build from source for reproducibility on 24.04.

```bash
mkdir -p ~/repos
cd ~/repos

git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.2.0

# Optional: faster build
sed -i 's/add_subdirectory(examples)/# add_subdirectory(examples)/g' CMakeLists.txt
sed -i 's/add_subdirectory(tests)/# add_subdirectory(tests)/g' CMakeLists.txt

mkdir -p build
cd build
cmake .. \
  -DGTSAM_WITH_TBB=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_UNSTABLE=ON \
  -DCMAKE_BUILD_TYPE=Release

make -j"$(nproc)"
sudo make install
sudo ldconfig
```

---

## 4) Build this repository

```bash
mkdir -p ~/liorf-ros2/src
cd ~/liorf-ros2/src
git clone <YOUR_FORK_OR_THIS_REPO_URL> liorf
cd ..

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 5) Configure ROS 2 to use Zenoh

Add to your shell config (`~/.bashrc`):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Apply it:

```bash
source ~/.bashrc
```

Quick check:

```bash
echo $RMW_IMPLEMENTATION
# expected: rmw_zenoh_cpp
```

---

## 6) Run

```bash
cd ~/liorf-ros2
source install/setup.bash
ros2 launch liorf run_lio_sam_default.launch.py
```

In another terminal (same environment), play a bag:

```bash
cd ~/liorf-ros2
source install/setup.bash
ros2 bag play <path_to_ros2_bag>
```

For bagfile replay (especially when collecting diagnostics/telemetry), prefer playing with ROS time:

```bash
ros2 bag play <path_to_ros2_bag> --clock
```

And set `use_sim_time` to `true` either:

- as a launch argument (recommended for ad-hoc runs), e.g. `use_sim_time:=true`, or
- in your parameter YAML file.

Launch argument behavior is override-only: if `use_sim_time` is not specified in launch, YAML parameter values remain unchanged.

### Plot time-slicing statistics

Diagnostics logging also maintains a stable symlink:

- `~/.ros/liorf_logs/latest` -> newest `run_YYYYMMDD_HHMMSS` directory

After a run, plot `timing_stats.csv` from diagnostics logs:

```bash
python3 scripts/plot_time_slicing_stats.py
```

By default, the script reads the latest run (via `~/.ros/liorf_logs/latest` when available), saves the PNG plot, and displays it.

Useful options:

```bash
# specific run directory or CSV
python3 scripts/plot_time_slicing_stats.py --input ~/.ros/liorf_logs/run_YYYYMMDD_HHMMSS

# smoothing + custom output
python3 scripts/plot_time_slicing_stats.py --window 10 --output /tmp/timing_plot.png
```

---

## 7) Frame model overview

LIORF follows ROS frame guidance from REP-105 ([map/odom/base_link](https://www.ros.org/reps/rep-0105.html#map)) and uses a layered variant so local smooth odometry and global georeferencing remain cleanly separated.

### Core frames (always used)

- `mapFrameLocal`: local SLAM optimization frame (always present; identity-aligned to `mapFrameEnu` until GPS anchor estimation is available).
- `odometryFrame`: compatibility/output frame used by odometry and map products consumed by downstream tools.
- `baselinkFrame`: robot body frame.
- `lidarFrame`: lidar sensor frame.

### GPS-enabled frames (used when GNSS is fused)

- `mapFrameEnu`: ENU frame anchored at the first accepted GNSS datum.
- `ECEFframe`: Earth-centered global frame.

### Two-layer frame scheme

- Global/georeferencing layer:
  - `ECEFframe -> mapFrameEnu -> mapFrameLocal`
- Local motion/robot layer:
  - `mapFrameLocal -> odometryFrame -> baselinkFrame`
  - Alternative compatibility branch (always published): `odometryFrame -> lidar_link`

### Implementation notes

- `odometryFrame -> lidar_link` is hardcoded and always published as a compatibility branch.
- To keep TF direction consistent from parent to child, the node computes `odometryFrame -> lidar_link` from optimized odometry and, when `lidarFrame != baselinkFrame`, applies the inverse of the looked-up `lidarFrame -> baselinkFrame` transform (effectively subtracting that offset from the base pose).
- For now, `mapFrameLocal` and `odometryFrame` are practically the same in nominal operation (their relative transform is initialized as identity and usually remains near identity).
- Relative to REP-105 terminology, this implementation intentionally splits the usual “map/odom” behavior into `mapFrameLocal` and `odometryFrame` so future loop-closure/global alignment corrections can be represented upstream without degrading odometry smoothness; georeferencing-related jumps are isolated at the `mapFrameEnu -> mapFrameLocal` joint.

See [ARCHITECTURE.md](ARCHITECTURE.md) for full transform chain, TF ownership, and publication behavior details.

---

## Optional: GPS integration notes (Floating Anchor)

This repository uses a **Floating Anchor** GPS fusion strategy:

- The local SLAM trajectory remains in its native `mapFrameLocal` frame (default: `map_local`).
- GPS does **not** hard-snap local key poses to global ENU.
- A separate global-to-local transform $T_{G\_L}$ (published as `mapFrameEnu -> mapFrameLocal`) is optimized in the graph.

### Inputs

- GNSS input topic type: `sensor_msgs/msg/NavSatFix`
- Configure `gpsTopic` in your selected YAML under [config](config)

Example:

```yaml
gpsTopic: "gps/fix"
```

### Outputs related to GPS fusion

- `liorf/mapping/gps_odom` (`nav_msgs/msg/Odometry`): local Cartesian projection of NavSatFix.
- `liorf/gps_origin` (`sensor_msgs/msg/NavSatFix`): captured datum origin used for local projection.
- TF `mapFrameEnu -> mapFrameLocal`: optimized global offset/rotation from floating-anchor fusion.
- `liorf/earth_to_map_offset` (`geometry_msgs/msg/PoseWithCovarianceStamped`): same offset as topic, including covariance when available.

### Map saving metadata

You can trigger map export via ROS service directly or with the helper script:

```bash
# default destination/resolution
./scripts/save_map.sh

# set voxel resolution
./scripts/save_map.sh -r 0.2

# HOME-relative destination
./scripts/save_map.sh -d Downloads/my_map

# absolute destination
./scripts/save_map.sh -a /tmp/liorf_map
```

You can also trigger map export via launch arguments:

```bash
# default values (resolution=0.0, destination='')
ros2 launch liorf save_map.launch.py

# custom resolution
ros2 launch liorf save_map.launch.py resolution:=0.2

# custom destination
ros2 launch liorf save_map.launch.py destination:=/tmp/liorf_map

# custom wait timeout for service availability
ros2 launch liorf save_map.launch.py wait_timeout_sec:=60.0
```

When calling `liorf/save_map`, GPS metadata is saved to:

- `map_metadata.yaml`

It includes:

- `global_datum` (latitude/longitude/altitude when available)
- `T_global_local` (`x,y,z,roll,pitch,yaw`)

Save response also reports useful export stats:

- `save_directory`: resolved absolute save path
- `enu_map_saved`: whether ENU artifacts were exported
- `keyframes_used`: number of keyframes used for map construction
- `surf_points_local` / `surf_points_enu`: surf map point counts
- `global_points_local` / `global_points_enu`: global map point counts
- `message`: status details (success or skip reason)

After successful map save, the node writes the absolute path to:

- `~/.liorf_last_saved_map_path`

### Satellite overlay visualization (saved maps)

You can visualize saved map outputs over satellite imagery with:

```bash
python3 scripts/visualize_saved_map_satellite.py --map-dir <saved_map_directory>
```

If `--map-dir` is omitted, the script resolves map directory in this order:

1. `~/.liorf_last_saved_map_path` (written by `liorf/save_map` on successful save)
2. default `~/Downloads/LOAM`
3. fail with an error message

Optional controls:

- `--output <html_path>`: output HTML file path
- `--max-surf-points <N>`: limit sampled surf points on overlay heatmap
- `--max-traj-points <N>`: limit sampled trajectory points

The script reads `map_metadata.yaml` and map PCD outputs (`*_ENU.pcd` preferred, or `*_local.pcd` transformed by `T_global_local`) and produces an interactive HTML map with satellite basemap and trajectory/surf overlays.

On success, it prints both the generated HTML path and a `file://...` URI that can be opened from terminal links.

---

## Docker users

Zenoh-based Docker setup is documented in [docker/README.md](docker/README.md).

---

## Acknowledgments

Thanks to the original projects and datasets:

- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
- [FAST_LIO2](https://github.com/hku-mars/FAST_LIO)
- [UrbanNavDataset](https://github.com/weisongwen/UrbanNavDataset)
- [M2DGR](https://github.com/SJTU-ViSYS/M2DGR)
- [MulRanDataset](https://sites.google.com/view/mulran-pr/?pli=1)
