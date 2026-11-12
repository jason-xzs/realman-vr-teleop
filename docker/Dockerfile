FROM ros:jazzy-ros-base-noble
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv python3-colcon-common-extensions python3-scipy \
    ros-jazzy-cv-bridge ros-jazzy-message-filters ros-jazzy-sensor-msgs \
    ros-jazzy-std-srvs adb usbutils libusb-1.0-0 libgl1 libglib2.0-0 ffmpeg \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/teleop-venv
ENV PATH="/opt/teleop-venv/bin:${PATH}"
COPY requirements.txt /tmp/teleop-requirements.txt
RUN pip install --no-cache-dir torch==2.7.1+cpu torchvision==0.22.1+cpu \
    --index-url https://download.pytorch.org/whl/cpu
RUN pip install --no-cache-dir -r /tmp/teleop-requirements.txt
ENV PYTHONPATH="/opt/teleop-venv/lib/python3.12/site-packages:/workspace:/workspace/r3kit"
ENV PYNPUT_BACKEND=dummy
WORKDIR /workspace/spacemouse_control_arm/ros2_realman_ws
CMD ["sleep", "infinity"]
