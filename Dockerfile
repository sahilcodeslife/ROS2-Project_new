# Use an official ROS 2 Humble base image (amd64)
FROM osrf/ros:humble-desktop

# Set environment variables
ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:1
# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    make \
    g++ \
    libx11-dev \
    libxt-dev \
    libxext-dev \
    libgl1-mesa-dev \
    doxygen \
    && rm -rf /var/lib/apt/lists/*


# Copy your ROS 2 project into the container (you don't want to change WORKDIR)
COPY . /root/ROS2-Project


# Source workspace automatically
RUN echo "source /root/ROS2-Project/install/setup.bash" >> /root/.bashrc
RUN apt-get update && \
    apt-get install -y ros-humble-depthai-ros && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
apt-get install -y python3-pip && \
pip3 install --upgrade pip ultralytics && \
rm -rf /var/lib/apt/lists/*
# Set entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]


