# 🤖 Type-1 Fuzzy Logic Controller for Pioneer 3-DX (ROS + Gazebo)
# 📌 Overview

This project implements a Type-1 Fuzzy Logic Controller (FLC) for the Pioneer 3-DX differential drive robot in ROS.
The controller computes velocity commands using error (E) and change of error (CE) as inputs, applying fuzzification, inference rules, and defuzzification to achieve stable and robust trajectory tracking.

The robot is commanded to follow a circular trajectory, and controller performance is compared between desired and actual paths.

# ✨ Features

✅ Type-1 Fuzzy Controller with:

5 linguistic terms (NL, NS, Z, PS, PL)

Membership functions for error & change of error

Fuzzy rule base for control decisions

Centroid-based defuzzification

✅ Trajectory tracking (circular path)

✅ Real-time odometry feedback (/pioneer/odom)

✅ Velocity control output (/pioneer/cmd_vel)

✅ Performance visualization with Gnuplot:

Desired vs. actual trajectory

Linear & angular velocity tracking

Orientation (theta) comparison

# 📂 File Structure
fuzzy1_pioneer3dx/
├── src/
│   └── fuzzy1.cpp          # Main ROS node
├── CMakeLists.txt          # (ROS CMake build file)
├── package.xml             # (ROS package definition)
└── README.md               # Documentation

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


4. Run the Fuzzy Controller node:

rosrun <your-package> fuzzy1


5. Gnuplot will automatically show:

Desired vs. Actual Trajectory

Linear Velocity Tracking

Angular Velocity Tracking

Orientation (Theta)

# 📊 Example Output

Trajectory Tracking → Blue = desired path, Red = actual robot path

Linear Velocity Tracking → Desired vs. Actual curves

Angular Velocity Tracking → Desired vs. Actual curves

Theta Tracking → Orientation performance
