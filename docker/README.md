# Docker Infrastructure for LIORF SLAM

This directory contains the containerized environment for the `liorf` project. It is designed to support a hybrid workflow:
1.  **Development:** Live code editing, cached builds, and hot-reloading.
2.  **Production:** Running the pre-compiled "Golden Image" with injected configurations.

---

## 1. Setup (One-Time)

### Prerequisites
* **Docker Engine** & **Docker Compose V2**
* **Make**
* **NVIDIA Container Toolkit** (if using GPU acceleration)

### Configuration
1.  Copy the template to a live configuration file:
    ```bash
    cp .env.example .env
    ```
2.  Edit `.env` to match your host machine paths:
    * `HOST_BAGS_PATH`: Directory where your `.mcap` or `.bag` files live.
    * `HOST_MAPS_PATH`: Directory where your global `.pcd` maps live.
    * `DEFAULT_BAG_FILENAME`: The specific file to play when running `make play`.
    * `USER_UID` / `USER_GID`: Set these to match your host user (run `id -u` to find out) to prevent permission issues.

---

## 2. Usage

You can run all commands from the **Project Root** (recommended) or from inside this `docker/` directory.

### A. Development Workflow
*Best for modifying C++ code, debugging, and testing.*

1.  **Start SLAM:**
    Builds the image (if missing), starts the container, and launches the node + Rviz.
    ```bash
    make slam
    ```

2.  **Play Data:**
    In a separate terminal, play the bag file defined in `.env`.
    ```bash
    make play
    ```

3.  **Build Code:**
    If you modify `.cpp` or `.h` files, recompile inside the running container:
    ```bash
    make build
    ```
    *Note: You do not need to rebuild for changes to Python scripts, Launch files, or YAML configs.*

4.  **Debug:**
    Enter the container shell to run manual ROS 2 commands.
    ```bash
    make shell
    ```

### B. Production Workflow (Pre-Built)
*Best for testing the standalone image or deploying.*

The `liorf_run` service runs the code **baked into the Docker image**, ignoring your local `src` folder. However, it still mounts your local `config/` and `launch/` folders, allowing you to tweak parameters without rebuilding.

*Note: Ensure you have a `prod` target in your Makefile to launch `liorf_run`.*

```bash
# Example command (if added to Makefile)
make prod
````

-----

## 3\. Architecture Overview

### Services

| Service | Docker Name | Description |
| :--- | :--- | :--- |
| **liorf\_dev** | `liorf_dev` | **Development Mode.** Mounts local source code (`../src`) and uses a persistent build cache (`../docker/cache`). Code changes are reflected immediately after `make build`. |
| **liorf\_run** | `liorf_run` | **Production Mode.** Runs the code compiled inside the Docker image. Ignores local source code but accepts local `config/` injections. |
| **bag\_player**| `bag_player` | **Data Replay.** A raw ROS 2 image configured to run as your user ID. Plays data onto the host network interface. |

### Networking

  * **Mode:** `host` (All services)
  * **IPC:** `host` (Shared Memory enabled)
  * **Rationale:** This ensures zero-copy transport for heavy Lidar data and guarantees that Rviz (on host) can see topics (in container) without complex DDS discovery configuration.

### File System & Caching

  * **Source Code:** Mapped to `/home/dev/ros2_ws/src/liorf`.
  * **Build Artifacts:** To allow hybrid development (Local + Docker), Docker build artifacts are stored in `docker/cache/` (build/install/log). This prevents conflicts with any `build/` folders at the project root created by local compiles.

-----

## 4\. Maintenance

| Command | Action | Use When... |
| :--- | :--- | :--- |
| `make stop` | Stops containers. | You are done for the day. |
| `make clean` | Removes containers & network. | Things are acting weird; you want a fresh start. |
| `make image` | Rebuilds Docker Image (Cached). | You changed `Dockerfile` or dependencies. |
| `make rebuild`| **Force** Rebuild (No Cache). | GTSAM or system libs are corrupted. (Slow\!) |
| `make prune` | System-wide cleanup. | You need to free up disk space from old containers. |

```