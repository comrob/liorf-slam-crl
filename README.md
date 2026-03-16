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

---

## Optional: GPS factor notes

- GNSS topic type should be `sensor_msgs/msg/NavSatFix`
- Set `gpsTopic` in your selected config file in [config](config)

Example:

```yaml
gpsTopic: "gps/fix"
```

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
