FROM ros:jazzy-ros-core

RUN apt-get update && apt-get install -y \
    ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup in bashrc for interactive shells
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc