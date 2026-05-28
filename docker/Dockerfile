FROM --platform=linux/arm64 osrf/ros:humble-desktop-full

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=0
# Increase pip timeout for slow connections
ENV PIP_DEFAULT_TIMEOUT=300

# Core ROS2 + Nav2 + TurtleBot3 + SLAM
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-rviz2 \
    ros-humble-tf2-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-teleop-twist-keyboard \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    wget \
    curl \
    vim \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libgl1-mesa-dev \
    xvfb \
    x11vnc \
    openbox \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip first
RUN pip3 install --no-cache-dir --upgrade pip

# Core utilities (small, fast)
RUN pip3 install --no-cache-dir --timeout 300 \
    networkx \
    opencv-python-headless \
    Pillow \
    numpy \
    scipy \
    addict \
    yapf

# Remove distutils-managed sympy that blocks torch install
RUN apt-get remove -y python3-sympy 2>/dev/null || true && \
    pip3 install --no-cache-dir --timeout 300 sympy

# PyTorch CPU-only (large download — separate layer with high timeout)
RUN pip3 install --no-cache-dir --timeout 600 \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Transformers + vision libs
RUN pip3 install --no-cache-dir --timeout 300 \
    transformers \
    timm \
    supervision \
    pycocotools

# CLIP
RUN pip3 install --no-cache-dir --timeout 300 \
    open-clip-torch

# spaCy
RUN pip3 install --no-cache-dir --timeout 300 spacy && \
    python3 -m spacy download en_core_web_sm

# Grounding DINO (try pip package first, fall back to source)
RUN pip3 install --no-cache-dir --timeout 300 groundingdino-py || \
    pip3 install --no-cache-dir --timeout 300 \
    "git+https://github.com/IDEA-Research/GroundingDINO.git"

# Clone explore_lite (not in apt for Humble)
RUN mkdir -p /ros2_ws/src && \
    git clone https://github.com/robo-friends/m-explore-ros2.git \
    /ros2_ws/src/m-explore-ros2

# Pre-build explore_lite baked into image
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --packages-select explore_lite_msgs explore_lite"

# Workspace dirs
RUN mkdir -p /ros2_ws/maps

WORKDIR /ros2_ws

# rosdep update
RUN rosdep update

# Shell configuration
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:\$GAZEBO_MODEL_PATH" >> /root/.bashrc && \
    echo "alias cb='cd /ros2_ws && colcon build --symlink-install'" >> /root/.bashrc && \
    echo "alias src='source /ros2_ws/install/setup.bash'" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
