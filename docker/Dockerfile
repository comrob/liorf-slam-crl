FROM osrf/ros:jazzy-desktop-full AS base

USER root

RUN sed -i 's/archive.ubuntu.com/us.archive.ubuntu.com/g' /etc/apt/sources.list

# 1. HEAVY DEPENDENCIES (GTSAM) + distcc added to install list
RUN apt-get update && apt-get install -y \
    software-properties-common \
    libpcl-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libgeographiclib-dev \
    libtbb-dev \
    libboost-all-dev \
    python3-pip \
    nano \
    git \
    wget \
    x11-apps \
    ccache \
    distcc \
    mold \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# Accept optional distributed build arguments with safe local fallbacks
ARG DISTCC_HOSTS=""
ARG CCACHE_PREFIX=""
ARG PARALLEL_WORKERS=""
ENV DISTCC_HOSTS=${DISTCC_HOSTS}
ENV CCACHE_PREFIX=${CCACHE_PREFIX}

WORKDIR /tmp
# Compile GTSAM leveraging ccache/distcc over Tailscale if configured
RUN git clone https://github.com/borglab/gtsam.git \
    && cd gtsam && git checkout 4.2.0 \
    && sed -i 's/add_subdirectory(examples)/# add_subdirectory(examples)/g' CMakeLists.txt \
    && sed -i 's/add_subdirectory(tests)/# add_subdirectory(tests)/g' CMakeLists.txt \
    && mkdir build && cd build \
    && cmake .. -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
       -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_UNSTABLE=ON -DCMAKE_BUILD_TYPE=Release \
       -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    && make -j${PARALLEL_WORKERS:-$(nproc)} && make install && ldconfig \
    && cd /tmp && rm -rf gtsam

# 3. USER SETUP
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

RUN userdel -r ubuntu || true \
    && groupdel ubuntu || true \
    && userdel -r $USERNAME || true \
    && groupdel $USERNAME || true \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 4. AUTO-SOURCING
USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc \
    && echo "if [ -f /home/$USERNAME/ros2_ws/install/setup.bash ]; then source /home/$USERNAME/ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# --- STAGE: deps (Development) ---
# This stage STOPS here. It has no source code.
# Perfect for dev containers that mount code via volumes.
FROM base AS deps
CMD ["sleep", "infinity"]

# --- STAGE: prod (Production) ---
# This stage continues and copies code into the image.
FROM base AS prod

# Re-declare ARGs for this build stage
ARG DISTCC_HOSTS=""
ARG CCACHE_PREFIX=""
ARG COLCON_PARALLEL_WORKERS=""
ENV DISTCC_HOSTS=${DISTCC_HOSTS}
ENV CCACHE_PREFIX=${CCACHE_PREFIX}

# 5. PRE-BUILD APPLICATION
RUN mkdir -p /home/$USERNAME/ros2_ws/src/lili

WORKDIR /home/$USERNAME/ros2_ws/src/lili

# Layer 1: Copy C++ source and build metadata
# Changes here trigger full C++ rebuild
COPY package.xml CMakeLists.txt ./
COPY src/ ./src
COPY include/ ./include
COPY msg/ ./msg
COPY srv/ ./srv
COPY lib/ ./lib
COPY cmake/ ./cmake

# Layer 1b: Copy placeholder files for config/launch/rviz
# These are empty placeholders to preserve Docker cache layer
COPY config/.empty config/
COPY launch/.empty launch/
COPY rviz/.empty rviz/

WORKDIR /home/$USERNAME/ros2_ws

# Build C++ code with ccache/distcc and dynamic parallelism
RUN echo "Building C++ code"
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /home/$USERNAME/ros2_ws && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                   -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      --parallel-workers ${COLCON_PARALLEL_WORKERS:-$(nproc)}"

WORKDIR /home/$USERNAME/ros2_ws/src/lili

# Layer 2: Copy actual configuration and launch files
# Changes here DO NOT trigger C++ rebuild - just file copy
COPY config/ ./config
COPY launch/ ./launch
COPY rviz/ ./rviz

WORKDIR /home/$USERNAME/ros2_ws

# Reinstall only the changed files (colcon incremental build)
RUN echo "Updating config/launch/rviz files"
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /home/$USERNAME/ros2_ws && \
    colcon build \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
                   -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      --parallel-workers ${COLCON_PARALLEL_WORKERS:-$(nproc)}"

# Add entrypoint for benchmark compatibility
COPY --chown=$USERNAME:$USERNAME docker/entrypoint.sh /home/$USERNAME/ros2_ws/entrypoint.sh
RUN chmod +x /home/$USERNAME/ros2_ws/entrypoint.sh

ENTRYPOINT ["/home/dev/ros2_ws/entrypoint.sh"]
CMD ["ros2", "launch", "lili", "run_lili_bench.py"]