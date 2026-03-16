# ================================================================
#  ROS 2 Development Commands
#  build, slam, play, shell, debugging
# ================================================================

# Include infrastructure targets that dev depends on
include $(dir $(abspath $(lastword $(MAKEFILE_LIST))))/infra.mk

Q := @

.PHONY: build rebuild slam prod play shell bag_shell

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
	$(Q)$(COMPOSE) up -d liorf_run
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
