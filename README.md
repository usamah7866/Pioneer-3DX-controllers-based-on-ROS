# 🤖 Pioneer 3-DX Control Algorithms (ROS + Gazebo)
# 📌 Overview

This repository contains a collection of control algorithms implemented for the Pioneer 3-DX differential drive robot using ROS 1 and Gazebo.
The goal is to compare and evaluate different control strategies for trajectory tracking under disturbances, focusing on robustness and accuracy.

Each controller is implemented as a ROS C++ node and tested in simulation. The robot is commanded to follow a circular trajectory, and results are visualized using Gnuplot (desired vs. actual paths, velocities, and orientation).

# ✨ Controllers Implemented

This repository includes multiple advanced controllers, each in its own folder:

🔹 1. ADRC (Active Disturbance Rejection Control)

🔹 2. Type-1 Fuzzy Logic Controller

🔹 3. Type-2 Fuzzy Logic Controller

🔹 4. Simple Kinematic Controller
