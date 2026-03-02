# Settings
DOCKER_DIR := .
COMPOSE := docker compose -f docker-compose.yaml --env-file .env

.PHONY: help up down build shell play slam stop image rebuild

help:
	@echo "LIORF SLAM Docker Environment"
	@echo "========================================"
	@echo "  make up      - Start containers in background"
	@echo "  make down    - Stop and remove containers"
	@echo "  make image   - Build Docker Image (Uses Cache - Fast)"
	@echo "  make reimage - Force Rebuild Docker Image (No Cache - Slow)"
	@echo "  make build   - Compile the ROS 2 package (Colcon)"
	@echo "  make slam    - Run SLAM (Restarts container & launches Rviz)"
	@echo "  make play    - Play the configured bag file"
	@echo "  make shell   - Enter the dev container"

# ----------------------------------------------------------------
#  Infrastructure Commands (Docker)
# ----------------------------------------------------------------

up:
	$(Q)$(COMPOSE) up -d liorf_dev bag_player

down:
	$(Q)$(COMPOSE) down liorf_dev bag_player

clean:
	@echo "Removing project containers and network..."
	$(Q)$(COMPOSE) down --remove-orphans liorf_dev bag_player liorf_run

prune:
	@echo "Pruning all stopped containers system-wide..."
	$(Q)docker container prune -f

clean-build:
	@echo "Cleaning ros liorf package build cache (preserving .empty files)..."
	$(Q)find $(DOCKER_DIR)/cache/build -mindepth 1 ! -name ".empty" -exec rm -rf {} +
	$(Q)find $(DOCKER_DIR)/cache/install -mindepth 1 ! -name ".empty" -exec rm -rf {} +
	$(Q)find $(DOCKER_DIR)/cache/log -mindepth 1 ! -name ".empty" -exec rm -rf {} +
	@echo "Cache cleaned."

# [NEW] Builds the image using existing cache (Fast)
image:
	@echo "Building Docker Image (Cached)..."
	$(Q)$(COMPOSE) build

reimage:
	@echo "⚠️  WARNING: You are about to rebuild the Docker image with --no-cache."
	@echo "   This will recompile GTSAM and heavy dependencies (10+ minutes)."
	@echo -n "   Are you sure you want to proceed? [y/N] " && read ans && [ $${ans:-N} = y ]
	@echo "Rebuilding from scratch..."
	$(Q)$(COMPOSE) build --no-cache
	$(Q)$(COMPOSE) up -d

stop:
	$(Q)$(COMPOSE) down

# ----------------------------------------------------------------
#  Development Commands (ROS)
# ----------------------------------------------------------------

# Compiles the C++ code inside the container
build: up
	@echo "Building the ROS2 liorf package..."
	# We must explicitly source setup.bash because 'bash -c' skips .bashrc
	$(Q)$(COMPOSE) exec -u dev liorf_dev bash -c \
		"source /opt/ros/jazzy/setup.bash && \
		 cd ~/ros2_ws && \
		 colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS='-w' -Wno-dev"
	@echo "Build success!"

rebuild: clean-build build

# Runs the SLAM node
slam: up
	@echo "Launching SLAM..."
	$(Q)xhost +local:docker > /dev/null 2>&1 || true
	$(Q)$(COMPOSE) exec -u dev liorf_dev bash -c \
		"source ~/ros2_ws/install/setup.bash && \
		ros2 launch liorf run_lio_sam_ouster.launch.py"

prod: up
	@echo "Launching Production SLAM..."
	$(Q)xhost +local:docker > /dev/null 2>&1 || true
	$(Q)$(COMPOSE) exec liorf_run bash -c \
		"source ~/ros2_ws/install/setup.bash && \
		ros2 launch liorf run_lio_sam_ouster.launch.py"

# Plays the rosbag
play:
	@echo "Playing Bag..."
	$(Q)$(COMPOSE) up -d bag_player
	$(Q)$(COMPOSE) exec -u dev bag_player bash -c \
					"source /opt/ros/jazzy/setup.bash && \
					ros2 bag play /bag_data/\$$BAG_FILENAME --clock --exclude-topics /tf"

# Enters the container shell
shell: up
	$(Q)$(COMPOSE) exec -u dev liorf_dev bash

# Enter bag_player shell
bag_shell: up
	$(Q)$(COMPOSE) exec -u dev bag_player bash