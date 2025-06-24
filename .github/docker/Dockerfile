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
    curl \
    gnupg \
    lsb-release

# Add ROS 2 APT source and key (modern secure version)
ARG ROS_DISTRO
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update

# Set working directory
WORKDIR /workspace

# Initialize rosdep
RUN rosdep init || true && \
    rosdep update --rosdistro=${ROS_DISTRO}

# Copy workspace in
COPY . .

# Install dependencies
RUN rosdep install --from-paths . --ignore-src -r -y

# Default command
CMD ["bash"]
