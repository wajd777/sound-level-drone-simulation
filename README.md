# Sound Level Monitoring Using Drone Simulation (Final Project)

## Overview
This project presents a simulation-based prototype for monitoring sound levels inside a virtual environment representing the Two Holy Mosques.
A simulated drone is used as a mobile platform to represent sound-level measurements across different locations.

The project integrates:
- PX4 SITL
- Gazebo Harmonic
- ROS 2 Humble
- ROS ↔ Gazebo Bridge (ros_gz_bridge)
- Python scripts for mission execution and data handling

The project is executed using Visual Studio Code (VS Code) through the integrated terminal and Python run features.

## Project Idea
Using a simulated drone as a mobile sensing platform to collect sound level data at specific locations, allowing better analysis compared to fixed sensor systems.

## Project Purpose
To collect sound level data (simulated) from multiple points within the environment and analyze it to support improvements in sound balance and overall sound quality.

## Functionality
- Simulate a drone operating inside a custom Gazebo world (myworld) using PX4 SITL
- Execute predefined flight missions
- Navigate the drone autonomously
- Integrate ROS 2 for sensor data processing
- Bridge Gazebo sensor topics to ROS 2 using ros_gz_bridge
- Represent sound level measurements using simulated data

## Intended Outcomes
- Provide reliable simulated sound level data
- Support sound distribution analysis
- Improve sound balance and clarity
- Assist technical decision-making
- Enhance user experience

## Prototype
This project is a working simulation prototype demonstrating drone spawning, PX4–Gazebo–ROS 2 integration, sensor topic availability, and autonomous mission execution.

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic
- PX4-Autopilot (SITL)
  
sudo apt install ros-humble-ros-gzharmonic

## World and Model Details
Gazebo World: myworld
PX4 Model: gz_x500_gimbal
Camera Topic:
/world/myworld/model/x500_gimbal_0/link/camera_link/sensor/camera/image

## How to Run (Using VS Code)


## Step 1: Run PX4 SITL with Gazebo and Custom World
```bash
cd ~/PX4-Autopilot

export PX4_HOME_LAT=21.4225
export PX4_HOME_LON=39.8262
export PX4_HOME_ALT=50.63

PX4_GZ_WORLD=myworld \
PX4_GZ_MODEL_POSE="-39.54,-29.14,19.5" \
make px4_sitl gz_x500_gimbal
```
## Step 2: Run ROS ↔ Gazebo Bridge (Camera)
ros2 run ros_gz_bridge parameter_bridge /world/myworld/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image

## Step 3: Run Project Scripts
python3 test.py
python3 gimbal_mission.py

## Verification
ros2 topic list

## Challenges & Solutions
World model loading with colors → Converting the model format to ensure correct visualization in Gazebo.
Drone operation using PX4 → Running PX4 with the same converted model format to ensure compatibility.
No real sound sensor → Combining drone models and generating simulated sound data to represent sound level measurements.

## Demo Video
A 1–2 minute demo video demonstrates drone spawning, ROS 2 topic availability, and mission execution.
