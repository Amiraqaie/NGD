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
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src

# Source ROS setup automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

# change to workspace src directory
WORKDIR /root

# download px4 autopilot recursively
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# setup ubuntu toolchain
RUN bash PX4-Autopilot/Tools/setup/ubuntu.sh

# change simulation to gazebo classic
RUN apt remove gz-harmonic
RUN apt install aptitude
RUN aptitude install gazebo libgazebo11 libgazebo-dev

# Default command
CMD ["/bin/bash"]