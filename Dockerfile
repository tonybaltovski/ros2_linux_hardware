# syntax=docker/dockerfile:1.7
# Multi-arch (amd64, arm64) image for ros2_linux_hardware.
#
# Build:
#   docker build -t ros2_linux_hardware .
#
# Run on a Raspberry Pi (or any host with /dev/i2c-N):
#   docker run --rm -it \
#     --device=/dev/i2c-1 \
#     --group-add "$(getent group i2c | cut -d: -f3)" \
#     --network host \
#     ros2_linux_hardware \
#     ros2 run linux_i2c_demos pca9685_servo_control

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS builder
ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      python3-colcon-common-extensions \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY . src/ros2_linux_hardware

# Resolve and install rosdep keys (libi2c-dev, rclcpp, std_msgs, ...).
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/*

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build \
         --merge-install \
         --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---- Runtime image ----------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

# i2c-tools is optional but useful for in-container debugging (i2cdetect, i2cget).
RUN apt-get update && apt-get install -y --no-install-recommends \
      i2c-tools \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ros2_ws/install /ros2_ws/install

# Source the overlay automatically in interactive shells and via the entrypoint.
RUN echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

# Reuse the upstream ROS entrypoint, then layer our overlay on top.
COPY <<'EOF' /ros_entrypoint.sh
#!/bin/bash
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /ros2_ws/install/setup.bash
exec "$@"
EOF
RUN chmod +x /ros_entrypoint.sh

WORKDIR /ros2_ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
