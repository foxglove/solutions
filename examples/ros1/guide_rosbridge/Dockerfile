FROM ros:noetic-ros-core

# Install rosbridge-server
RUN apt-get update && apt-get install -y \
    ros-noetic-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Source ROS setup in bashrc for non-interactive shells
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Verify the package is installed and find the launch file
RUN bash -c "source /opt/ros/noetic/setup.bash && rospack find rosbridge_server"
