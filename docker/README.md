# Docker Infrastructure for LIORF SLAM

A complete containerized development environment for the **LIORF SLAM** package, optimized for LiDAR data processing with CycloneDDS middleware.

**Key Features:**
- 🚀 **Two deployment modes** - PC-based bagfile testing OR live robot driver integration
- 📦 **Pre-compiled images** - Golden Image with GTSAM & all dependencies
- 📡 **CycloneDDS** - High-performance middleware for pointcloud data
- 🔄 **Hybrid workflow** - Develop in container, mount code from host
- 🎯 **Reproducible** - Same environment across machines via GitHub Container Registry

---

## Choose Your Use Case

### 1️⃣ **PC-Based Development** (Testing with Bagfiles)
**Best for:** Algorithm development, debugging, offline testing
- Run SLAM in Docker
- Play bagfiles (local or in Docker)
- Edit code, rebuild, test
- **No live sensor hardware needed**

→ [Jump to PC-Based Setup](#use-case-1-pc-based-development)

### 2️⃣ **Onboard Robot/Edge Computer** (Live Driver Integration)
**Best for:** Real-world deployment, live Ouster LiDAR driver
- Ouster driver runs on host machine
- SLAM runs in Docker container
- Both communicate via shared DDS
- **Clone and run everything from repository**

→ [Jump to Robot/Onboard Setup](#use-case-2-onboard-computerrobot-live-driver)

---

# USE CASE 1: PC-Based Development

## Quick Start (5 minutes)

### Prerequisites
```bash
docker --version        # Docker Engine 20.10+
docker compose version  # Docker Compose V2
make --version         # GNU Make 4.0+
which nvidia-smi       # GPU support (optional)
```

### Setup

```bash
cd /home/seva/repos/liorf-orig-2/docker

# Copy template and edit
cp .env.example .env
nano .env
```

**`.env` configuration:**
```dotenv
# Point to your bagfiles
HOST_BAGS_PATH=~/data/bags
HOST_MAPS_PATH=~/data/maps

# Which bagfile to play
DEFAULT_BAG_FILENAME=dataset_0.mcap
BAG_FILENAME=dataset_0.mcap

# Your user ID
USER_UID=1000    # run: id -u
USER_GID=1000    # run: id -g
```

### Run

**Terminal 1:**
```bash
cd /home/seva/repos/liorf-orig-2
make slam
```

**Terminal 2:**
```bash
cd /home/seva/repos/liorf-orig-2
make play
```

Monitor RViz on your host for SLAM results!

---

## PC-Based: Host Machine DDS Setup

The Docker containers use **CycloneDDS** for efficient pointcloud transport. Your host must also use CycloneDDS to see container topics in RViz.

### Install CycloneDDS on Host

```bash
# Install DDS implementation
sudo apt-get install ros-jazzy-rmw-cyclonedds-cpp

# Copy the optimized config from the repository
mkdir -p ~/.ros
cp /path/to/liorf-orig-2/docker/cyclonedds.xml ~/.cyclonedds.xml
```

**Note:** The repository includes an optimized `docker/cyclonedds.xml` configured for LiDAR performance. Use this for both host and container.

### Enable in Shell

Add to `~/.bashrc`:
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
```

Then:
```bash
source ~/.bashrc
```

### Override DDS Config (Optional)

If you want to use a custom DDS configuration instead of the repo default:

**Option 1: Set in `.env`**
```bash
# Edit docker/.env
HOST_DDS_CONFIG=/path/to/your/custom/cyclonedds.xml
```

**Option 2: Copy to expected location**
```bash
cp /your/config/cyclonedds.xml ~/.cyclonedds.xml
```

The docker-compose will use `${HOST_DDS_CONFIG:-./cyclonedds.xml}`, meaning:
- If `HOST_DDS_CONFIG` is set, use that
- Otherwise, use `docker/cyclonedds.xml` (default)

### Verify

```bash
echo $RMW_IMPLEMENTATION
# Output: rmw_cyclonedds_cpp

ros2 doctor
# Should show CycloneDDS
```

---

## PC-Based: Complete Developer Guide

### Workflow

1. **Edit code on host:**
   ```bash
   nano src/imageProjection.cpp
   ```

2. **Build in container:**
   ```bash
   make build
   ```

3. **Test with bagfile:**
   ```bash
   make slam      # Terminal 1
   make play      # Terminal 2
   ```

4. **Iterate:**
   - Stop SLAM (Ctrl+C)
   - Modify code
   - `make build` again
   - Restart SLAM

### Container Shell Access

```bash
make shell
# Inside: edit files, run commands, debug
```

### Change Sensor Configuration

```bash
nano config/lio_sam_ouster.yaml
# Edit: N_SCAN, Horizon_SCAN, etc.
# No rebuild needed - applies on restart
```

### Monitor ROS Topics

```bash
# From host (with CycloneDDS enabled)
source ~/.bashrc
ros2 topic list
ros2 topic echo /odometry/filtered
rviz2
```

---

# USE CASE 2: Onboard Computer/Robot (Live Driver)

## Architecture

```
┌─────────────────────────────────────────────┐
│ Robot / Edge Computer                       │
│                                             │
│  ┌─────────────────────────────────────┐  │
│  │ Host Machine (ROS 2)                │  │
│  │                                     │  │
│  │  Ouster Driver (liorf repo)        │  │
│  │  └─ Publishes: /os1/points, /os1/imu
│  │  └─ CycloneDDS enabled             │  │
│  │                                     │  │
│  └─────────────────────────────────────┘  │
│                  ↓ (DDS)                   │
│  ┌─────────────────────────────────────┐  │
│  │ Docker Container (Same Repo)        │  │
│  │                                     │  │
│  │  SLAM (liorf)                       │  │
│  │  ├─ imageProjection                │  │
│  │  ├─ imuPreintegration              │  │
│  │  ├─ mapOptmization                 │  │
│  │  └─ CycloneDDS (same config)       │  │
│  │                                     │  │
│  │  Subscribes: /os1/points, /os1/imu│  │
│  │  Publishes: odometry, map          │  │
│  └─────────────────────────────────────┘  │
│                                             │
│  **Same .cyclonedds.xml for both**         │
└─────────────────────────────────────────────┘
```

## Setup

### Prerequisites

```bash
# On robot/edge computer
Ubuntu 24.04 + ROS 2 Jazzy
docker installed
sudo usermod -aG docker $USER  # Run docker without sudo
```

### Step 1: Clone Repository

```bash
# Clone to your robot
git clone https://github.com/comrob/liorf-orig-2.git
cd liorf-orig-2
```

### Step 2: Create Shared DDS Configuration

```bash
# Copy the optimized config from the cloned repository to host
mkdir -p ~/.ros
cp liorf-orig-2/docker/cyclonedds.xml ~/.cyclonedds.xml
```

Both host and container will use this identical configuration for DDS communication.

### Step 3: Install Ouster Drivers + CycloneDDS

```bash
# Install Ouster driver
sudo apt-get update
sudo apt-get install ros-jazzy-ouster-ros

# Install CycloneDDS
sudo apt-get install ros-jazzy-rmw-cyclonedds-cpp
```

### Step 4: Configure Host Environment

Add to `~/.bashrc`:

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# DDS Configuration (must match Docker)
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml

# Ouster settings
export OUSTER_SENSOR_HOSTNAME=192.168.1.101  # Your lidar IP
```

Apply:
```bash
source ~/.bashrc
```

### Step 5: Configure Docker

```bash
# In the cloned repository
cd liorf-orig-2/docker

# Copy environment file
cp .env.example .env

# Edit if needed (usually defaults are fine)
nano .env
```

**Default `.env` is fine for onboard:**
```dotenv
HOST_BAGS_PATH=~/bags          # Not used with live driver
HOST_MAPS_PATH=/home/user/maps # Where to save maps

# Optional: use custom DDS config
# HOST_DDS_CONFIG=/path/to/custom/cyclonedds.xml

USER_UID=1000
USER_GID=1000
```

**Optional:** To use a custom DDS config instead of the repo default:
```bash
# Edit .env and set:
HOST_DDS_CONFIG=~/.cyclonedds.xml
```

### Step 6: Build Docker Image

```bash
# From repository root
cd /path/to/liorf-orig-2
make image
```

---

## Robot: Running Live SLAM

### Terminal 1: Start Ouster Driver (Host)

```bash
# On robot, in any directory
source ~/.bashrc
ros2 launch ouster_ros driver.launch.py

# Output:
# Publishing /os1/points
# Publishing /os1/imu
# ...
```

### Terminal 2: Start SLAM (Container)

```bash
# From the repository root
cd /path/to/liorf-orig-2
make slam

# Output:
# Launching SLAM...
# Subscribing to /os1/points
# Subscribing to /os1/imu
# Processing...
```

### Terminal 3: Monitor (Host)

```bash
# On robot
source ~/.bashrc

# View topics
ros2 topic list

# View odometry
ros2 topic echo /odometry/filtered

# Launch RViz
rviz2
```

---

## Robot: Verify DDS Configuration

**Critical:** Host and Docker must use identical DDS config.

```bash
# Check host is configured
echo $RMW_IMPLEMENTATION
# Output: rmw_cyclonedds_cpp

echo $CYCLONEDDS_URI
# Output: file://$HOME/.cyclonedds.xml

# Enter container
make shell

# Check container has same settings
echo $RMW_IMPLEMENTATION
# Output: rmw_cyclonedds_cpp (same!)

echo $CYCLONEDDS_URI
# Output: file:///home/dev/cyclonedds.xml

# Verify configs match
diff ~/.cyclonedds.xml <(docker exec $(docker ps -q -f ancestor=liorf_jazzy_dev) cat /home/dev/cyclonedds.xml)
# Should be identical
```

---

## Robot: Configuration Table

| Component | Setting | Host | Container |
|-----------|---------|------|-----------|
| RMW Implementation | `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | `rmw_cyclonedds_cpp` |
| DDS Config URI | `CYCLONEDDS_URI` | `file://$HOME/.cyclonedds.xml` | `file:///home/dev/cyclonedds.xml` |
| ROS Domain | `ROS_DOMAIN_ID` | `0` | `0` |
| Network | `network_mode` | N/A | `host` |
| Config Content | XML | `/home/user/.cyclonedds.xml` | `/home/dev/cyclonedds.xml` |
| **Content must match** | ✓ | Identical | Identical |

---

## Robot: Troubleshooting

### Container doesn't receive /os1/points

```bash
# 1. Verify driver publishing
ros2 topic list | grep os1
# Should show: /os1/points, /os1/imu

# 2. Check RMW on both sides
echo "Host: $RMW_IMPLEMENTATION"
docker exec $(docker ps -q -f ancestor=liorf_jazzy_dev) sh -c "echo Container: \$RMW_IMPLEMENTATION"
# Both should be: rmw_cyclonedds_cpp

# 3. Compare DDS configs
diff ~/.cyclonedds.xml docker/cyclonedds.xml
# Should output nothing (identical)

# 4. If different, update container config
docker exec $(docker ps -q -f ancestor=liorf_jazzy_dev) cp /home/dev/cyclonedds.xml /tmp/backup.xml
docker cp ~/.cyclonedds.xml $(docker ps -q -f ancestor=liorf_jazzy_dev):/home/dev/cyclonedds.xml
docker exec $(docker ps -q -f ancestor=liorf_jazzy_dev) chown dev:dev /home/dev/cyclonedds.xml
```

### High latency or dropped messages

Edit `~/.cyclonedds.xml` and increase buffers:
```xml
<SocketReceiveBufferSize min="50MB" />  <!-- Was 10MB -->
<WhcHigh>1MB</WhcHigh>                  <!-- Was 500kB -->
```

Then restart both driver and SLAM.

### Ouster sensor connection fails

```bash
# Verify sensor IP and connectivity
ping 192.168.1.101

# Check ROS environment
echo $OUSTER_SENSOR_HOSTNAME

# Launch with verbose output
ros2 launch ouster_ros driver.launch.py log_level:=debug
```

---

## Troubleshooting

### Issue: Permission Denied on Cache

```
PermissionError: [Errno 13] Permission denied: 'log/build_...'
```

**Solution:**
```bash
cd docker/
sudo chown -R $USER:$USER cache/
```

### Issue: NaN/Inf Points in Pointcloud

```
[ERROR] Assertion `point_representation_->isValid (point)...' failed
```

**Solution:** Check config matches sensor:
```bash
# Verify your sensor type and parameters
cat config/lio_sam_ouster.yaml | grep -E "sensor:|N_SCAN:|Horizon_SCAN:"

# For Ouster OS1:
# sensor: ouster
# N_SCAN: 128
# Horizon_SCAN: 1024
```

### Issue: Dropped Messages / High Latency

**Symptom:** SLAM processes few points, runs slowly

**Solution:** Verify CycloneDDS is active:
```bash
make shell
echo $RMW_IMPLEMENTATION
# Should output: rmw_cyclonedds_cpp
```

If not set, CycloneDDS isn't configured. Rebuild with:
```bash
make reimage
```

### Issue: RViz Not Displaying

**Symptom:** RViz window doesn't appear

**Solution:** Grant X11 access:
```bash
xhost +local:docker
make slam
```

### Issue: Bagfile Won't Play

```
[ERROR] No such file or directory
```

**Solution:** Verify bagfile path:
```bash
# Check .env
cat .env | grep BAG_FILENAME

# Verify file exists
ls -lh $HOST_BAGS_PATH/$BAG_FILENAME

# Check mount inside container
make shell
ls -lh /bag_data/
```

---

## Architecture

### Services

| Service | Image | Purpose | Mount Mode |
|---------|-------|---------|-----------|
| **liorf_dev** | `liorf_jazzy_dev` | Live development | Source mounted from host |
| **liorf_run** | `liorf_jazzy_prod` | Pre-built production | Code baked in image |
| **bag_player** | `liorf_jazzy_dev` | Bagfile playback | Data mounted from host |

### Networking & IPC

- **Network:** `host` - Direct network access (zero-copy for pointclouds)
- **IPC:** `host` - Shared memory (SLAM ↔ RViz communication)
- **PID:** `host` - Process namespace (debugging)
- **DDS:** CycloneDDS with optimized config for LiDAR

### File System

```
Host Machine              Docker Container
├── src/                  /home/dev/ros2_ws/src/liorf
├── config/               /home/dev/ros2_ws/src/liorf/config
├── launch/               /home/dev/ros2_ws/src/liorf/launch
└── docker/cache/
    ├── build/       ←→   /home/dev/ros2_ws/build
    ├── install/     ←→   /home/dev/ros2_ws/install
    └── log/         ←→   /home/dev/ros2_ws/log
```

---

## Available Commands

### Infrastructure
```bash
make up          # Start containers
make down        # Stop containers
make clean       # Remove containers & network
make image       # Build image (cached)
make reimage     # Rebuild image (no cache)
make prune       # System cleanup
```

### Development
```bash
make build       # Compile C++ code
make rebuild     # Clean + build
make slam        # Launch SLAM
make prod        # Launch production SLAM
make play        # Play bagfile
make shell       # Enter dev container
make bag_shell   # Enter bag_player container
```

### Publishing
```bash
make tag         # Tag image (TAG=v1.0.0)
make push        # Push to registry (TAG=v1.0.0)
make publish     # Tag + Push
make info        # Show registry config
```

### Direct Makefile Access
```bash
# If you prefer direct access to .mk files:
make -f docker/mk/infra.mk clean-build
make -f docker/mk/dev.mk slam
make -f docker/mk/publish.mk help-publish
```

---

## Performance Tips

### Increase Build Speed
```bash
# Use mold linker (already in Dockerfile)
# Parallel builds
colcon build -j$(nproc)
```

### Optimize Pointcloud Processing
```yaml
# In config/lio_sam_ouster.yaml
mappingSurfLeafSize: 0.4      # Larger = faster but less detailed
point_filter_num: 5           # Skip points (1 = use all)
downsampleRate: 1             # Downsample scan
```

### Monitor Resource Usage
```bash
# Inside container
make shell
watch -n 1 'ps aux | head -20'
```

---

## FAQ

**Q: Can I modify code and immediately test?**  
A: Yes! Edit code, run `make build`, restart SLAM with `make slam`.

**Q: Do I need to rebuild for config changes?**  
A: No, just edit `.yaml` and restart `make slam`.

**Q: How do I use a different launch file?**  
A: Edit `docker/mk/dev.mk` line for `slam` target:
```makefile
slam: up
	ros2 launch liorf run_kitti.launch.py  # Change here
```

**Q: What if I want to run on GPU?**  
A: GPU support is automatic if you have NVIDIA Container Toolkit installed. No changes needed!

**Q: Can I access ROS 2 topics from my host?**  
A: Yes! DDS is in `host` network mode, so:
```bash
# On host
export ROS_DOMAIN_ID=0
ros2 topic list  # See container topics
```

