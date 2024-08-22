# Use a ROS 2 base image
FROM ros:galactic-ros-base

RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-opencv \
    ros-galactic-behaviortree-cpp-v3 \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace directory inside the container
RUN mkdir -p /root/ros_ws/src

# Set the working directory to the ROS workspace
WORKDIR /root/ros_ws

# Source the ROS 2 setup script
RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# Set entrypoint to bash
ENTRYPOINT ["/bin/bash"]