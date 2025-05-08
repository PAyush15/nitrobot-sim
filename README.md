# Nitrobot

Table of Contents:

  - [Overview](#overview)
  - [Demo](#demo)
  - [Features](#features)
  - [Project Structure](#projectstructure)
  - [Installation](#installation)
  - [Usage](#usage)
  - [License](#license)
  - [Acknowledgments](#acknowledgments)

## Overview

A ROS2 simulation of differential-drive robot created using URDF with sensors such as camera, depth camera and LiDAR. This project is for getting acquainted with ROS2 following the series of 'Building a mobile robot' by Articulated Robotics (Josh Newans) on Youtube.


## Demo


https://github.com/user-attachments/assets/20811afe-c478-42f3-b71b-dbfdf7436685


## Features

- **Autonomous  Navigation** using ROS Nav2 Stack and Cartographer SLAM

- **Real-Time Obstacle Avoidance** with 2D LiDAR and camera (optionally depth cam)

- **Multiple Control Modes**: teleoperation, joystick, scripted navigation, autonomous mode

- **Custom ROS2 Controller**: Implemented a custom ROS2 controller for controlling the robot

- **Custom Environment**: Custom envrionment creation and mapping it using slam_toolbox



## Project Structure

### 📁 Project Structure

```plaintext
autonomous-indoor-delivery-robot-main/
├── 📜 README.md                     # Project overview and documentation
├── 📜 qrcode_scan.py               # QR code detection script (Jetson Nano + Pi Cam)
├── 📜 rosserial.ino                # Arduino code for servo/buzzer control

├── 📂 adbot_description/           # URDF and package description for adbot
│   ├── 📂 config/                      # ROS configuration and parameter files
│   ├── 📂 launch/                      # Launch files for simulation and real-world runs
│   ├── 📂 maps/                        # Saved maps for navigation
│   ├── 📂 meshes/                      # STL files for 3D components
│   ├── 📂 params/                      # Navigation parameter files (e.g., global_costmap_params.yaml)
│   ├── 📂 scripts/                     # ROS Python nodes for robot behavior
│   ├── 📂 urdf/                        # URDF files of the robot
│   ├── 📂 worlds/                      # Custom Gazebo world files
│   ├── 📜 CMakeLists.txt               # Build instructions for catkin
│   ├── 📜 package.xml                  # ROS package metadata

├── 📂 rmp_bot_description/         # URDF and package description for rmp_bot
│   ├── 📜 CMakeLists.txt
│   ├── 📜 LICENSE
│   ├── 📜 package.xml

├── 📂 ros_controllers-melodic-devel/  # External ROS control package fork
│   ├── 📜 .gitignore
│   ├── 📜 .travis.yml
│   ├── 📜 README.md
```



## Installation

**Prerequisites**

Ensure you have ROS Humble installed on Ubuntu 22.04. If not, install it using this tutorial:
```
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
  ```
  
Clone and Build the Package

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/PAyush15/nitrobot-sim

# Build the workspace
cd ~/catkin_ws
catkin build

# Source the setup file
source devel/setup.bash
```

## Usage

Run QR Code Authentication System
```rosrun autonomous-indoor-delivery-robot-main qrcode_scan.py```

This script (for Jetson Nano + Raspberry Pi Camera):

Captures QR/aruco markers\
Validates user delivery code\
Sends a signal to the Arduino (via serial) to unlock the correct compartment\

Run Arduino Firmware
1. Upload the following code to your Arduino Uno:\
2. Open Arduino IDE\
3. Connect your Arduino board\
4. Open the file: autonomous-indoor-delivery-robot-main/rosserial.ino\
5. Select board & port\
6. Click Upload

The Arduino script controls:

Servo motor (for locking mechanism)\
Buzzer (for unauthorized access)

Launch URDF (example for RViz test)
Then, to launch the robot in the gazebo world:\
```roslaunch adbot_description gazebo.launch```\

Then, to start SLAM for the robot, launch:
```roslaunch adbot_description slam.launch map:=maps/trs_lab_thin_b.yaml use_sim_time:=true```

Finally, launch the navigation launch file with the parameter file path
```ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true```


## License

This project is licensed under the MIT License.
Feel free to use, modify, and distribute — just give credit where it’s due!

## Acknowledgments

Thanks to Josh Newans for such an amazing series and getting starting with ROS2!


