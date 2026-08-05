# ================================================================
#  ROS 2 Development Commands
#  build, rebuild, slam, prod, play, shell, bag_shell, verify-distcc
# ================================================================

# Include infrastructure targets that dev depends on
include $(dir $(abspath $(lastword $(MAKEFILE_LIST))))/infra.mk

Q := @

.PHONY: build rebuild slam prod play shell bag_shell verify-distcc

# Compiles the C++ code inside the container using ccache and distcc over Tailscale
build: up
	@echo "Building the ROS 2 lili package with distributed compilation..."
	# Explicitly source setup.bash because 'bash -c' skips .bashrc
	$(Q)$(COMPOSE) exec -u dev lili_dev bash -c \
		"source /opt/ros/jazzy/setup.bash && \
		 cd ~/ros2_ws && \
		 colcon build --symlink-install \
		   --parallel-workers \$${COLCON_PARALLEL_WORKERS:-16} \
		   --cmake-args \
		     -DCMAKE_C_COMPILER_LAUNCHER=ccache \
		     -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
		     -DCMAKE_CXX_FLAGS='-w' -Wno-dev"
	@echo "Build success!"

# [NEW] Verifies distcc connectivity and prints ccache statistics
verify-distcc: up
	@echo "=== Checking ccache and distcc status inside lili_dev ==="
	$(Q)$(COMPOSE) exec -u dev lili_dev bash -c \
		"echo 'Effective DISTCC_HOSTS:' \$$DISTCC_HOSTS && \
		 echo 'Effective CCACHE_PREFIX:' \$$CCACHE_PREFIX && \
		 echo '--- Tailscale Remote Host Connectivity ---' && \
		 (distcc --show-hosts 2>/dev/null || echo 'Hosts configured in environment') && \
		 echo '--- ccache statistics ---' && \
		 ccache -s"

rebuild: clean-build build

# Runs the SLAM node
slam: up
	@echo "Launching SLAM..."
	$(Q)xhost +local:docker > /dev/null 2>&1 || true
	$(Q)$(COMPOSE) exec -u dev lili_dev bash -c \
		"source ~/ros2_ws/install/setup.bash && \
		 ros2 launch lili run_lili_ouster.launch.py \
		   config_override:=/home/dev/ros2_ws/install/lili/share/lili/config/docker_override.yaml"

prod: up
	@echo "Launching Production SLAM..."
	$(Q)xhost +local:docker > /dev/null 2>&1 || true
	$(Q)$(COMPOSE) up -d lili_run
	$(Q)$(COMPOSE) exec lili_run bash -c \
		"source ~/ros2_ws/install/setup.bash && \
		 ros2 launch lili run_lili_ouster.launch.py \
		   config_override:=/home/dev/ros2_ws/install/lili/share/lili/config/docker_override.yaml"

# Plays the rosbag
play:
	@echo "Playing Bag..."
	$(Q)$(COMPOSE) up -d bag_player
	$(Q)$(COMPOSE) exec -u dev bag_player bash -c \
		"source /opt/ros/jazzy/setup.bash && \
		 ros2 bag play /bag_data/\$$BAG_FILENAME --clock --exclude-topics /tf"

# Enters the container shell
shell: up
	$(Q)$(COMPOSE) exec -u dev lili_dev bash

# Enter bag_player shell
bag_shell: up
	$(Q)$(COMPOSE) exec -u dev bag_player bash