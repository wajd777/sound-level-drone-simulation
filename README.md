
# Sound Level Monitoring Using Drone Simulation (Final Project)

## Overview
This project presents a **simulation-based prototype** for monitoring sound levels inside a virtual environment representing the **Two Holy Mosques**.
A simulated drone is used as a mobile platform to represent sound level measurements across different locations, reducing reliance on fixed sensors.

The project integrates:
- PX4 SITL
- Gazebo Harmonic
- ROS 2 Humble
- ROS ↔ Gazebo Bridge (ros_gz_bridge)
- Python scripts for mission execution and data handling

The project is executed using **Visual Studio Code (VS Code)** through its integrated terminal and Python run features.

---

## Project Idea
Using a simulated drone as a mobile sensing platform to collect more accurate sound level data at specific locations, enabling better analysis of sound distribution compared to fixed sensor systems.

---

## Project Purpose
To collect sound level data (simulated) from multiple points within the environment and analyze it to support improvements in sound balance and overall sound quality.

---

## Functionality
- Simulate a drone operating inside a custom Gazebo world (**myworld**) using PX4 SITL
- Execute predefined flight missions
- Navigate the drone autonomously
- Integrate ROS 2 for sensor data processing
- Bridge Gazebo sensor topics to ROS 2 using `ros_gz_bridge`
- Represent sound level measurements using simulated data
- Support verification of data flow through ROS 2 topics

---

## Intended Outcomes
- Provide reliable simulated sound level data
- Support sound distribution analysis
- Improve sound balance and quality
- Assist technical decision-making
- Enhance user experience through better sound clarity

---

## Prototype
This project is a **working simulation prototype** that demonstrates:
- Drone spawning and operation in Gazebo
- Integration between PX4, Gazebo, and ROS 2
- Sensor topic bridging and data availability
- Autonomous mission execution

---

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic
- PX4-Autopilot (SITL)
- ROS–Gazebo bridge:
```bash
sudo apt install ros-humble-ros-gzharmonic


⸻

World and Model Details
	•	Gazebo World: myworld
	•	PX4 Model: gz_x500_gimbal
	•	Camera Topic:
/world/myworld/model/x500_gimbal_0/link/camera_link/sensor/camera/image

⸻

How to Run (Using VS Code)

One-Time ROS 2 Setup (Optional)

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc


⸻

Step 1: Run PX4 SITL with Gazebo and Custom World

cd ~/PX4-Autopilot

export PX4_HOME_LAT=21.4225
export PX4_HOME_LON=39.8262
export PX4_HOME_ALT=50.63

PX4_GZ_WORLD=myworld \
PX4_GZ_MODEL_POSE="-39.54,-29.14,19.5" \
make px4_sitl gz_x500_gimbal


⸻

Step 2: Run ROS ↔ Gazebo Bridge (Camera)

ros2 run ros_gz_bridge parameter_bridge \
/world/myworld/model/x500_gimbal_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image


⸻

Step 3: Run Project Scripts

Run directly from VS Code (Run ▶) or terminal:

python3 test.py

python3 gimbal_mission.py


⸻

Verification

To confirm that ROS 2 is receiving data:

ros2 topic list

(Optional)

ros2 topic echo /<topic_name>


⸻

Challenges & Solutions
	•	World model loading with colors → Converting the model format to ensure correct visualization in Gazebo.
	•	Drone operation using PX4 → Running PX4 with the same converted model format to ensure compatibility.
	•	No real sound sensor → Combining drone models and generating simulated sound data to represent sound level measurements.

⸻

Demo Video

A 1–2 minute demo video demonstrates:
	1.	Drone spawning in the custom Gazebo world
	2.	ROS 2 topic availability
	3.	Drone response and mission execution

⸻

Notes
	•	ROS 2 demo nodes (demo_nodes_py, demo_nodes_cpp) were used only to verify ROS 2 installation.
	•	Sound data is simulated; no physical microphone sensor is used.
