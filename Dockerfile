# # Multi-stage Dockerfile for Dual UR5e Configuration  
# FROM osrf/ros:humble-desktop-full AS base  
  
# # Install system dependencies  
# RUN apt-get update && apt-get install -y \  
#     python3-pip \  
#     python3-colcon-common-extensions \  
#     python3-rosdep \  
#     git \  
#     wget \  
#     curl \  
#     vim \  
#     && rm -rf /var/lib/apt/lists/*  
  
# # Install CMake 3.22+ (if not already present)  
# RUN apt-get update && apt-get install -y \  
#     cmake \  
#     && cmake --version \  
#     && rm -rf /var/lib/apt/lists/*  
  
# # Install ROS 2 Humble MoveIt and related packages  
# RUN apt-get update && apt-get install -y \  
#     ros-humble-moveit \  
#     ros-humble-moveit-ros-move-group \  
#     ros-humble-moveit-ros-planning-interface \  
#     ros-humble-moveit-kinematics \  
#     ros-humble-moveit-planners \  
#     ros-humble-moveit-simple-controller-manager \  
#     ros-humble-moveit-configs-utils \  
#     ros-humble-moveit-ros-visualization \  
#     ros-humble-moveit-ros-warehouse \  
#     ros-humble-moveit-setup-assistant \  
#     ros-humble-moveit-core \  
#     ros-humble-moveit-msgs \  
#     && rm -rf /var/lib/apt/lists/*  
  
# # Install Ignition Gazebo (Gazebo Sim) and ros_gz packages  
# RUN apt-get update && apt-get install -y \  
#     ros-humble-ros-gz-sim \  
#     ros-humble-ros-gz-bridge \  
#     ros-humble-gazebo-ros-pkgs \  
#     ros-humble-gazebo-ros2-control \  
#     && rm -rf /var/lib/apt/lists/*  
  
# # Install RViz2 and visualization tools  
# RUN apt-get update && apt-get install -y \  
#     ros-humble-rviz2 \  
#     ros-humble-rviz-common \  
#     ros-humble-rviz-default-plugins \  
#     && rm -rf /var/lib/apt/lists/*  
  
# # Install MongoDB warehouse support  
# RUN apt-get update && apt-get install -y \  
#     ros-humble-warehouse-ros-mongo \  
#     mongodb \  
#     && rm -rf /var/lib/apt/lists/*  
  
# # Install BIM dependencies  
# RUN apt-get update && apt-get install -y \  
#     python3-numpy \  
#     python3-ifcopenshell \  
#     && rm -rf /var/lib/apt/lists/*  
  
# # Install ROS 2 control and other dependencies  
# RUN apt-get update && apt-get install -y \  
#     ros-humble-controller-manager \  
#     ros-humble-joint-state-publisher \  
#     ros-humble-joint-state-publisher-gui \  
#     ros-humble-joint-trajectory-controller \  
#     ros-humble-robot-state-publisher \  
#     ros-humble-tf2-ros \  
#     ros-humble-xacro \  
#     ros-humble-control-msgs \  
#     ros-humble-trajectory-msgs \  
#     ros-humble-geometry-msgs \  
#     ros-humble-shape-msgs \  
#     ros-humble-std-msgs \  
#     ros-humble-visualization-msgs \  
#     ros-humble-launch \  
#     ros-humble-launch-ros \  
#     && rm -rf /var/lib/apt/lists/*  
  
# # Install ament build tools and linting  
# RUN apt-get update && apt-get install -y \  
#     ros-humble-ament-cmake \  
#     ros-humble-ament-cmake-python \  
#     ros-humble-ament-lint-auto \  
#     ros-humble-ament-lint-common \  
#     python3-pytest \  
#     && rm -rf /var/lib/apt/lists/*  
  
# # Install additional Python dependencies  
# RUN pip3 install --no-cache-dir \  
#     ament-index-python  
  
# # Install Boost library  
# RUN apt-get update && apt-get install -y \  
#     libboost-all-dev \  
#     && rm -rf /var/lib/apt/lists/*  
  
# # Create workspace  
# WORKDIR /ros2_ws/src  
  
# # Copy the repository  
# COPY . /ros2_ws/src/Dual_ur5e_configuration/  
  
# # Install rosdep dependencies  
# WORKDIR /ros2_ws  
# RUN apt-get update && \  
#     rosdep update && \  
#     rosdep install --from-paths src --ignore-src -r -y && \  
#     rm -rf /var/lib/apt/lists/*  
  
# # Build the workspace  
# RUN . /opt/ros/humble/setup.sh && \  
#     colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release  
  
# # Setup environment  
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \  
#     echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc  
  
# # Set environment variables for Gazebo  
# ENV GZ_SIM_RESOURCE_PATH=/ros2_ws/install  
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  
  
# WORKDIR /ros2_ws  
  
# CMD ["/bin/bash"]

# Multi-stage Dockerfile for Dual UR5e Configuration
# Optimized for distribution as a downloadable package

# Multi-stage Dockerfile for Dual UR5e Configuration
# Optimized for distribution as a downloadable package

FROM osrf/ros:humble-desktop-full AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/ros2_ws

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    wget \
    curl \
    vim \
    nano \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install CMake 3.22+ (if not already present)
RUN apt-get update && apt-get install -y \
    cmake \
    && cmake --version \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble MoveIt and related packages
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-planners \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-setup-assistant \
    ros-humble-moveit-core \
    ros-humble-moveit-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Ignition Gazebo (Gazebo Sim) and ros_gz packages
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Install RViz2 and visualization tools
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rviz-default-plugins \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 control and other dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joint-trajectory-controller \
    ros-humble-robot-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-xacro \
    ros-humble-control-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-geometry-msgs \
    ros-humble-shape-msgs \
    ros-humble-std-msgs \
    ros-humble-visualization-msgs \
    ros-humble-launch \
    ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# Install ament build tools and linting
RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# Install Boost library
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies including ifcopenshell via pip
RUN apt-get update && apt-get install -y \
    python3-numpy \
    && pip3 install ifcopenshell \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# Builder Stage: Clone and Build from GitHub
# ============================================
FROM base AS builder

# Set GitHub repository URL
ARG GITHUB_REPO=https://github.com/sumit280188/Dual_ur5e_configuration.git
ARG GITHUB_BRANCH=main

# Create workspace
WORKDIR ${WORKSPACE}/src

# Clone the repository from GitHub
RUN echo "Cloning repository from ${GITHUB_REPO}..." && \
    git clone --branch ${GITHUB_BRANCH} --depth 1 ${GITHUB_REPO} && \
    echo "Repository cloned successfully!"

# List packages found
RUN echo "Found packages in workspace:" && \
    find ${WORKSPACE}/src -name "package.xml" -exec dirname {} \; | sort

# Install rosdep dependencies (skip problematic packages)
WORKDIR ${WORKSPACE}
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "python3-ifcopenshell gazebo_ros_control warehouse_ros_mongo" && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN echo "Building workspace..." && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo "Build completed successfully!"

# ============================================
# Production Stage: Slim runtime image
# ============================================
FROM base AS production

# Copy built workspace from builder stage
COPY --from=builder ${WORKSPACE}/install ${WORKSPACE}/install
COPY --from=builder ${WORKSPACE}/src ${WORKSPACE}/src

# Setup environment in bashrc
RUN echo "# ROS 2 Humble Setup" >> /root/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=${WORKSPACE}/install" >> /root/.bashrc && \
    echo "" >> /root/.bashrc && \
    echo "# Welcome message" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo '╔════════════════════════════════════════════════╗'" >> /root/.bashrc && \
    echo "echo '║   Dual UR5e Configuration - ROS2 Humble       ║'" >> /root/.bashrc && \
    echo "echo '╚════════════════════════════════════════════════╝'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo 'Available packages:'" >> /root/.bashrc && \
    echo "colcon list 2>/dev/null || echo '  Run: colcon list'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc && \
    echo "echo 'Quick Start Commands:'" >> /root/.bashrc && \
    echo "echo '  Launch RViz:  ros2 launch <package> display.launch.py'" >> /root/.bashrc && \
    echo "echo '  Launch Gazebo: ros2 launch <package> gazebo.launch.py'" >> /root/.bashrc && \
    echo "echo '  MoveIt:       ros2 launch <package> move_group.launch.py'" >> /root/.bashrc && \
    echo "echo ''" >> /root/.bashrc

# Set environment variables for Gazebo and ROS
ENV GZ_SIM_RESOURCE_PATH=${WORKSPACE}/install
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

# Create entrypoint script
RUN echo '#!/bin/bash' > /entrypoint.sh && \
    echo 'set -e' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Source ROS 2 setup' >> /entrypoint.sh && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /entrypoint.sh && \
    echo 'source ${WORKSPACE}/install/setup.bash' >> /entrypoint.sh && \
    echo '' >> /entrypoint.sh && \
    echo '# Execute command' >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set working directory
WORKDIR ${WORKSPACE}

# Add metadata labels
LABEL maintainer="Your Name <your.email@example.com>"
LABEL description="Dual UR5e Robot Configuration for ROS2 Humble"
LABEL version="1.0"
LABEL ros.distro="humble"

# Expose common ROS ports (optional)
EXPOSE 11311 11345

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["/bin/bash"]