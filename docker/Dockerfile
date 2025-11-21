# Base image
FROM nvcr.io/nvidia/isaac-sim:4.2.0

ENV DEBIAN_FRONTEND=noninteractive

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    wget \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble
RUN apt-get update && apt-get install -y locales

RUN locale-gen en_US en_US.UTF-8

RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

RUN apt-get install -y software-properties-common

RUN add-apt-repository universe
   
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt-get update && apt-get install -y ros-humble-desktop

RUN apt-get clean

RUN rm -rf /var/lib/apt/lists/*

# Install ROS2 bridge for Isaac Sim
RUN apt-get update && apt-get install -y \
    ros-humble-ros-ign-interfaces \
    ros-humble-ros-gz-bridge \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=Y
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble
ENV ROS_VERSION=2

# Set up volumes for caches and data
VOLUME ["/isaac-sim/kit/cache", "/root/.cache/ov", "/root/.cache/pip", "/root/.cache/nvidia/GLCache", "/root/.nv/ComputeCache", "/root/.nvidia-omniverse/logs", "/root/.local/share/ov/data", "/root/Documents"]

# Source ROS2
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN apt-get update && apt-get install -y python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y python3-pip

RUN pip3 install torch torchvision torchaudio


# Install SSH client (choose based on your base image)
RUN apt-get update && \
    apt-get install -y openssh-client && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p ~/.ssh && \
    chmod 700 ~/.ssh && \
    touch ~/.ssh/known_hosts && \
    chmod 600 ~/.ssh/known_hosts

RUN git config --global user.name "Fourier PC" && \
    git config --global user.email "blinky.unswathome@gmail.com" && \
    git config --global pull.rebase false && \
    git config --global init.defaultBranch main

RUN apt-get update && apt-get install -y \
    libssl-dev \
    libffi-dev \
    python3-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# README!: the statements below, especially the second one, should run Isaac Sim every time the container is initialized. However, it does not work.
# Command to run Isaac Sim in headless mode
# CMD ["./runheadless.native.sh", "-v"]

# Command to run Isaac Sim with GUI
# CMD ["./runapp.sh", "-v"]
