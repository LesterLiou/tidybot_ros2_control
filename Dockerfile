# syntax=docker/dockerfile:1

# ROS 2 Jazzy's official binary platform is Ubuntu 24.04 (Noble).
FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    TIDYBOT_CAN_INTERFACE=can0

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install --no-install-recommends -y \
        can-utils \
        iproute2 \
        libgl1 \
        libusb-1.0-0 \
        python3-colcon-common-extensions \
        python3-opencv \
        python3-pip \
        python3-pytest \
        python3-yaml \
        ros-jazzy-cv-bridge \
        ros-jazzy-geometry-msgs \
        ros-jazzy-joy \
        ros-jazzy-nav-msgs \
        ros-jazzy-realsense2-camera \
        ros-jazzy-realsense2-description \
        ros-jazzy-rosbag2 \
        ros-jazzy-rosbag2-compression-zstd \
        ros-jazzy-rqt-image-view \
        ros-jazzy-sensor-msgs \
        ros-jazzy-std-msgs \
        ros-jazzy-std-srvs \
        ros-jazzy-tf2-ros \
        tesseract-ocr \
        tesseract-ocr-eng \
        usbutils \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/tidybot_ws/src/tidybot_ros2_control

# Install only the ROS 2 control dependencies. The larger requirements.txt is
# for the original simulation/training application and is intentionally outside
# this hardware-control image.
COPY requirements-ros2.txt ./
RUN python3 -m pip install --break-system-packages --no-cache-dir \
        -r requirements-ros2.txt \
        opencv-contrib-python-headless==4.10.0.84

COPY . .

WORKDIR /opt/tidybot_ws
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --packages-select tidybot_ros2_control \
    && source install/setup.bash \
    && python3 -c "import cv2, numpy, phoenix6, rclpy, ruckig; assert hasattr(cv2, 'aruco')" \
    && tesseract --version \
    && python3 -m pytest -q src/tidybot_ros2_control/tests

COPY scripts/docker-entrypoint.sh /usr/local/bin/tidybot-entrypoint
COPY scripts/docker-hardware-check.sh /usr/local/bin/tidybot-hardware-check
COPY scripts/docker-can-init.py /usr/local/bin/tidybot-can-init
RUN chmod 0755 \
        /usr/local/bin/tidybot-entrypoint \
        /usr/local/bin/tidybot-hardware-check \
        /usr/local/bin/tidybot-can-init \
    && mkdir -p /home/tidybot/.ros /home/tidybot/jerry/tidybot_ros2_control/rosbags

ENV HOME=/home/tidybot
WORKDIR /opt/tidybot_ws
ENTRYPOINT ["/usr/local/bin/tidybot-entrypoint"]
CMD ["bash"]
