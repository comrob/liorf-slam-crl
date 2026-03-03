#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace if it exists
if [ -f /home/dev/ros2_ws/install/setup.bash ]; then
    source /home/dev/ros2_ws/install/setup.bash
fi

# Set default CYCLONEDDS_URI if not already set
# Framework can override this via environment variables
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file:///config/dds/cyclonedds.xml}

# Execute the command passed to the container
exec "$@"
