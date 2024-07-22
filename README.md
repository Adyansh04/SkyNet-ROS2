
# SkyNet-ROS2

SkyNet-ROS2 is a ROS 2 package designed for controlling and managing Crazyflie 2.1 drones. This project includes functionalities such as drone streaming, mapping, and navigation using the ROS 2 Humble framework.

## Index

- [Project Overview](#project-overview)
- [Prerequisites](#prerequisites)
- [Project Setup](#project-setup)
- [Running the Project](#running-the-project)
  - [Teleoperation with `teleop_twist_keyboard`](#teleoperation-with-teleop_twist_keyboard)
- [Explanation of Important Scripts and Configurations](#explanation-of-important-scripts-and-configurations)
  - [Configuration Files](#configuration-files)
  - [Launch Files](#launch-files)
  - [Scripts](#scripts)
  - [Source Code](#source-code)
- [ROS 2 Messages and Services](#ros-2-messages-and-services)
  - [Messages](#messages)
  - [Services](#services)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Project Overview

SkyNet-ROS2 provides a comprehensive solution for Crazyflie 2.1 drones, offering features like:
- AIDECK streaming
- Multi-ranger mapping
- Navigation with Nav2
- Teleoperation

## Prerequisites

1. **Crazyflie 2.1 Setup**
   - Ensure you have a Crazyflie 2.1 drone with the necessary firmware.
   - Install the [Crazyflie Python library](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/).

2. **ROS 2 Humble Installation**
   - Follow the official [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html) for your operating system.

## Project Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Adyansh04/SkyNet-ROS2.git
   cd SkyNet-ROS2
   ```
2. **Build the workspace:**
   ```bash
   colcon build
   ```
3. **Source the setup file:**
   ```bash
   source install/setup.bash
   ```

## Running the Project
To launch the various functionalities, use the provided launch files:

### AIDECK Streaming:
```bash
ros2 launch skynet aideck_streamer.yaml
```

### Multi-ranger Mapping:
```bash
ros2 launch skynet multiranger_mapping_launch.py
```

### Multi-ranger Navigation:
```bash
ros2 launch skynet multiranger_nav2_launch.py
```

### Simple Mapper:
```bash
ros2 launch skynet multiranger_simple_mapper_launch.py
```

### Startup:
```bash
ros2 launch skynet startup.launch.py
```

### Teleoperation with `teleop_twist_keyboard`
You can control the drone manually using the teleop_twist_keyboard package. This package allows you to send velocity commands to the drone from your keyboard.

Install `teleop_twist_keyboard`:
```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

Launch the startup file:
```bash
ros2 launch skynet startup.launch.py
```

Run `teleop_twist_keyboard`:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Control the Drone:
- Use the arrow keys to move the drone.
- Press `t` to take off.
- Press `b` to land.

## Explanation of Important Scripts and Configurations

### Configuration Files
- `config/aideck_streamer.yaml`: Configuration for AIDECK streamer.
- `config/crazyflies.yaml`: Configuration file listing Crazyflie drones.
- `config/nav2_params.yaml`: Parameters for Nav2 navigation stack.
- `config/slam.rviz`: RViz configuration for SLAM.

### Launch Files
- `launch/multiranger_mapping_launch.py`: Launch file for multi-ranger mapping.
- `launch/multiranger_nav2_launch.py`: Launch file for multi-ranger navigation.
- `launch/multiranger_simple_mapper_launch.py`: Launch file for simple mapping with multi-ranger.
- `launch/startup.launch.py`: General startup launch file.

### Scripts
- `scripts/aideck_streamer.py`: Script for streaming data from the AIDECK.
- `scripts/crazyflie_server.py`: Server script for handling Crazyflie communication.
- `scripts/simple_mapper_multiranger.py`: Script for simple mapping using multi-ranger sensors.
- `scripts/vel_mux.py`: Script for velocity multiplexer.

### Source Code
- `src/crazyflie_server.cpp`: C++ implementation of the Crazyflie server.
- `src/teleop.cpp`: Teleoperation control implementation.

## ROS 2 Messages and Services

### Messages
- `skynet_interfaces/msg/ConnectionStatistics.msg`: Message definition for connection statistics.
- `skynet_interfaces/msg/FullState.msg`: Full state message definition.
- `skynet_interfaces/msg/Hover.msg`: Hover control message.
- `skynet_interfaces/msg/Position.msg`: Position message definition.

### Services
- `skynet_interfaces/srv/Arm.srv`: Service for arming the drone.
- `skynet_interfaces/srv/GoTo.srv`: Service for navigating to a specified position.
- `skynet_interfaces/srv/Land.srv`: Service for landing the drone.
- `skynet_interfaces/srv/Takeoff.srv`: Service for drone takeoff.
- `skynet_interfaces/srv/UploadTrajectory.srv`: Service for uploading a trajectory.

## Contributing
Feel free to submit issues, fork the repository, and create pull requests. Contributions are welcome!

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements
This project was inspired by the need for efficient and reliable drone control using ROS 2 and Crazyflie.