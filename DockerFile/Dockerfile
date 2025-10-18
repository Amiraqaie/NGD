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

# install bash and some requirements
RUN apt-get update && apt-get install -y bash
RUN apt-get update && apt-get install -y \
build-essential \
cmake \
ninja-build \
python3-venv \
python3-pip \
wget \
unzip \
zip \
&& rm -rf /var/lib/apt/lists/*
SHELL ["/bin/bash", "-c"]

# setup ubuntu toolchain
RUN bash /root/PX4-Autopilot/Tools/setup/ubuntu.sh

# change simulation to gazebo classic
RUN apt remove -y gz-harmonic
RUN apt install -y aptitude
RUN aptitude install -y gazebo libgazebo11 libgazebo-dev

# copy local droneai gazebo model into PX4 gazebo-classic models directory
COPY ../gazebo_models/droneai /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/droneai
COPY ../gazebo_models/apt.world /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/apt.world

# ensure user's gazebo models dir exists and copy apt model into it
RUN mkdir -p /root/.gazebo/models
COPY ../gazebo_models/apt /root/.gazebo/models/apt

# ensure PX4 ROMFS airframes dir exists and copy the local airframe into it
COPY ../airframe/1046_gazebo-classic_droneai /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/1046_gazebo-classic_droneai

# insert the airframe name into the PX4 ROMFS CMakeLists so it's picked up by builds
# the airframes directory and CMakeLists.txt already exist in the repository, so just insert the line
RUN sed -i '/1045_gazebo-classic_quadtailsitter/a 1046_gazebo-classic_droneai' /root/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt

# ensure the droneai target is present for gazebo-classic SITL targets (idempotent)
RUN FILE=/root/PX4-Autopilot/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake && \
    grep -Fxq 'droneai' "$FILE" || sed -i '/iris_vision/a droneai' "$FILE"

# install rtabmap and nav2 packages and also ros2 gazebo plugins
RUN apt install ros-humble-rtabmap-ros ros-humble-nav2-bringup ros-humble-gazebo-ros-* -y

# modify rtabmap.launch.py
RUN FILE=/opt/ros/humble/share/rtabmap_ros/launch/rtabmap.launch.py && \
    if [ -f "$FILE" ]; then \
        sed -i "s/DeclareLaunchArgument('namespace',\s*default_value='rtabmap',\s*description='')/DeclareLaunchArgument('namespace', default_value='', description='')/" "$FILE"; \
    else \
        echo "⚠️ File not found: $FILE, skipping modification"; \
    fi

# install tmux
RUN apt install tmux -y

# install MicroXRCE-DDS
RUN pip install --user -U empy==3.3.4 pyros-genmsg setuptools
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /root/Micro-XRCE-DDS-Agent
RUN mkdir build
WORKDIR /root/Micro-XRCE-DDS-Agent/build
RUN cmake ..
RUN make
RUN sudo make install
RUN sudo ldconfig /usr/local/lib/

# make px4_sitl and clone neccesory github repos
WORKDIR /root/PX4-Autopilot
RUN make px4_sitl

# Default command
CMD ["/bin/bash"]