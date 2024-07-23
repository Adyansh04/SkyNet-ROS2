# Use the official ROS2 Humble Desktop full image
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Update and install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
    python3-dev \
    git \
    libusb-1.0-0-dev \
    libglib2.0-dev \
    udev \
    sudo \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install crazyflie-clients-python
RUN pip3 install --upgrade pip
RUN pip3 install cfclient

# Install crazyflie-lib-python
RUN pip3 install cflib

# Setup USB permissions for Crazyradio
RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0666"' > /etc/udev/rules.d/99-crazyradio.rules

# Add user for non-root operation
RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo

# Configure sudo for the docker user without requiring a password
RUN echo "docker ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN usermod -aG plugdev docker

# Switch to non-root user
USER docker
WORKDIR /home/docker

# Allow access to X server
ENV DISPLAY=:0

# Source the ROS2 setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Expose the necessary ports
EXPOSE 11311

# Forward all USB ports to the container
ENV UDEV=1

# Start with a shell
CMD ["bash"]
