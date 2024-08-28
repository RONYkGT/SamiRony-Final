# Use a ROS 2 base image
FROM ros:galactic-ros-base

#### Installs are seperated to save the caching and save time.

# Grouped installation of stable packages that are unlikely to change 
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-opencv \
    ros-galactic-behaviortree-cpp-v3 \
    && rm -rf /var/lib/apt/lists/*

# Separate installation of packages you might want to modify or add later
RUN apt-get update && apt-get install -y \
    # Add any additional packages you might modify here
    python3-pip \
    ros-galactic-cv-bridge \
    ros-galactic-rclpy \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies in a separate step
RUN pip3 install --no-cache-dir \
    roboflow \
    supervision

RUN pip3 install --no-cache-dir ultralytics
RUN pip3 install --no-cache-dir pyzbar

RUN apt-get update && apt-get install -y \
    libzbar0 \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && apt-get install -y \
    ros-galactic-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory inside the container
RUN mkdir -p /root/ros_ws/src

# Set the working directory to the ROS workspace
WORKDIR /root/ros_ws

# Source the ROS 2 setup script
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# Set entrypoint to bash
ENTRYPOINT ["/bin/bash"]