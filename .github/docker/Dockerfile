# Base image with minimal ROS 2
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Install essential build tools + rosdep
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Set working directory
WORKDIR /workspace

# Copy metadata for dependency installation
COPY . .

# Install dependencies via rosdep
RUN rosdep install --from-paths . --ignore-src -r -y

# Build entrypoint (can be overridden at runtime)
CMD ["bash"]
