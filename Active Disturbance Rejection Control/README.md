# ADRC Controller for Pioneer 3-DX (ROS + Gazebo)
# 📌 Overview

This project implements an Active Disturbance Rejection Controller (ADRC) for the Pioneer 3-DX differential drive robot in ROS. The controller ensures accurate trajectory tracking while rejecting disturbances.

The robot is commanded to follow a circular trajectory, and the controller dynamically adjusts linear and angular velocities to minimize tracking errors.

# ✨ Features

✅ ADRC for linear and angular velocity control

✅ Circular trajectory tracking

✅ Real-time odometry feedback (/pioneer/odom)

✅ Velocity command publishing (/pioneer/cmd_vel)

✅ Trajectory logging (desired vs. actual path)

✅ Gnuplot visualization for:

Desired vs. actual trajectory

Linear & angular velocities

Desired vs. actual orientation

ADRC estimated states

# 📂 File Structure
adrc_pioneer3dx/
├── src/
│   └── adrc.cpp          # Main ROS node
├── CMakeLists.txt        # (example ROS CMake file)
├── package.xml           # (ROS package definition)
└── README.md             # Documentation

# ⚙️ Requirements

ROS 1 (Noetic/Melodic recommended)

Gazebo with Pioneer 3-DX model

Gnuplot for plotting results

Install Gnuplot if missing:

sudo apt-get install gnuplot

# ▶️ How to Run

1. Clone the repo into your ROS workspace:

cd ~/catkin_ws/src
git clone https://github.com/<your-username>/<repo-name>.git
cd ..
catkin_make
source devel/setup.bash


2. Launch Gazebo with Pioneer 3-DX:

3. roslaunch p3dx_gazebo p3dx_gazebo.launch


4. Run the ADRC controller node:

rosrun <your-package> adrc


Plots (trajectory, velocities, ADRC states) will open in Gnuplot automatically.

# 📊 Example Output

Trajectory Tracking → Blue = desired path, Red = actual robot path

Linear/Angular Velocity Tracking

ADRC Estimated States (x1v, x1w)
