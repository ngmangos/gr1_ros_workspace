FROM nvcr.io/nvidia/isaac-sim:5.1.0

# Switch to root for installation
USER root

# Install system dependencies including python3-venv
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-venv \
    openssh-client \
    git \
    curl \
    lsb-release \
    gnupg \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create and activate virtual environment
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install GPU PyTorch in virtual environment
RUN pip install --upgrade pip && \
    pip install torch torchvision torchaudio \
        --index-url https://download.pytorch.org/whl/cu124
    
# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add sources list
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
        > /etc/apt/sources.list.d/ros2.list

# Install ROS Jazzy Desktop
RUN apt-get update && \
    apt-get install -y \
    ros-jazzy-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (required for ROS package dependencies)
RUN rosdep init && rosdep update

# Install 
RUN /opt/venv/bin/pip install catkin_pkg

# Source ROS each shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# Set up SSH directory and permissions
RUN mkdir -p ~/.ssh && \
    chmod 700 ~/.ssh && \
    touch ~/.ssh/known_hosts && \
    chmod 600 ~/.ssh/known_hosts

# Configure Git
RUN git config --global user.name "Fourier PC" && \
    git config --global user.email "blinky.unswathome@gmail.com" && \
    git config --global pull.rebase false

# Set working directory
WORKDIR /fourier-sim