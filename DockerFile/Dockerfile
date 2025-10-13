# Use the official OSRF ROS image
FROM osrf/ros:humble-desktop

# Set environment variables for ROS
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV PATH=$ROS_ROOT/bin:$PATH
ENV LD_LIBRARY_PATH=$ROS_ROOT/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=$ROS_ROOT/lib/python3/dist-packages:$PYTHONPATH

# Update and install common tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    vim \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create a workspace (optional)
RUN mkdir -p ~/ros2_ws/src
WORKDIR ~/ros2_ws/src

# Source ROS setup automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]